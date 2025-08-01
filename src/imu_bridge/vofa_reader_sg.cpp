#include "vofa_reader_sg.hpp"

vofa_reader::vofa_reader(const std::string &device,
                         std::string _lcm_topic,
                         speed_t baud,
                         int timeout_ms)
    : device_(device),
      baud_(baud),
      timeout_ms_(timeout_ms),
      fd_(-1),
      frame_nr_(0),
      lcm_topic_(std::move(_lcm_topic))
{
    if (!lcm_.good())
    {
        std::cerr << "ERROR: LCM initialization failed." << std::endl;
    }
}

vofa_reader::~vofa_reader()
{
    close_port();
}

bool vofa_reader::open_port()
{
    fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
        std::cerr << "Error opening " << device_ << ": "
                  << strerror(errno) << "\n";
        return false;
    }
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0)
    {
        std::cerr << "Tcgetattr() failed: " << strerror(errno) << "\n";
        close(fd_);
        fd_ = -1;
        return false;
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, baud_);
    cfsetospeed(&tty, baud_);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = timeout_ms_ / 100;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        std::cerr << "Tcsetattr() failed: " << strerror(errno) << "\n";
        close(fd_);
        fd_ = -1;
        return false;
    }
    return true;
}

void vofa_reader::close_port()
{
    if (fd_ >= 0)
    {
        close(fd_);
        fd_ = -1;
    }
}

void vofa_reader::run_once()
{
    if (fd_ < 0)
        return;

    // Constants for frame parsing
    constexpr std::array<uint8_t, 4> tail = {0x00, 0x00, 0x80, 0x7f};
    constexpr size_t expected_bytes = 9 * sizeof(float); // 36 bytes
    constexpr size_t max_buffer_size = 1024;

    // Read any available bytes from the serial port into our buffer
    uint8_t tmp[256];
    ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n > 0)
    {
        buffer_.insert(buffer_.end(), tmp, tmp + n);
    }
    else if (n < 0 && errno != EAGAIN)
    {
        std::cerr << "WARN: Serial read error: " << strerror(errno) << std::endl;
    }

    // Process all complete frames currently in the buffer
    while (true)
    {
        // Search for the frame tail
        auto it = std::search(buffer_.begin(), buffer_.end(), tail.begin(), tail.end());

        // If no tail is found, we're done for now.
        if (it == buffer_.end())
        {
            break;
        }

        size_t frame_end_pos = std::distance(buffer_.begin(), it);

        // Check if the frame has the expected length
        if (frame_end_pos == expected_bytes)
        {
            std::vector<uint8_t> frame(buffer_.begin(), it);
            publish_frame(frame);
        }
        else
        {
            std::cerr << "WARN: Malformed frame detected. Length was "
                      << frame_end_pos << ", expected " << expected_bytes << ".\n";
        }

        // Erase the processed frame and the tail from the buffer to prepare for the next
        buffer_.erase(buffer_.begin(), it + tail.size());
    }

    // Prevent buffer from growing indefinitely if malformed data is received
    if (buffer_.size() > max_buffer_size)
    {
        std::cout << "WARN: Buffer overflow, clearing half of the buffer." << std::endl;
        buffer_.erase(buffer_.begin(), buffer_.begin() + buffer_.size() / 2);
    }
}

void vofa_reader::decode_frame(const std::vector<uint8_t> &frame)
{
    float values[9];
    std::memcpy(values, frame.data(), sizeof(values));

    std::cout << "Decoded: "
              << "roll=" << values[0]
              << ", pitch=" << values[1]
              << ", yaw=" << values[2] << std::endl;
}

void vofa_reader::publish_frame(const std::vector<uint8_t> &frame)
{
    float values[9];
    std::memcpy(values, frame.data(), sizeof(values));

    exlcm::imu_cmd imu_msg;
    // Get current timestamp (microseconds since epoch)
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    imu_msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

    for (int i = 0; i < 3; i++)
    {
        imu_msg.rpy[i] = values[i];
        imu_msg.gyro[i] = values[i + 3];
        imu_msg.acc[i] = values[i + 6];
    }

    lcm_.publish(lcm_topic_, &imu_msg);

    std::cout << "Decoded: "
          << "roll=" << values[0]
          << ", pitch=" << values[1]
          << ", yaw=" << values[2] << std::endl;
}

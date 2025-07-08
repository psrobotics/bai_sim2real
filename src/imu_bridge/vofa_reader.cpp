#include "vofa_reader.hpp"

vofa_reader::vofa_reader(const std::string &device,
                         std::string _lcm_topic,
                         speed_t baud,
                         int timeout_ms)
    : device_(device),
      baud_(baud),
      timeout_ms_(timeout_ms),
      fd_(-1),
      stop_requested_(false),
      frame_nr_(0),
      lcm_topic_(_lcm_topic)
{
    if (!lcm_.good())
        std::cerr << "LCM init error" << std::endl;
}

vofa_reader::~vofa_reader()
{
    stop();
    if (read_thread_.joinable())
        read_thread_.join();
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

void vofa_reader::start(int hz)
{
    stop_requested_ = false;
    read_thread_ = std::thread(&vofa_reader::read_loop_hz, this, hz);
}

void vofa_reader::stop()
{
    stop_requested_ = true;
}

bool vofa_reader::is_stoped()
{
    return stop_requested_;
}

void vofa_reader::close_port()
{
    if (fd_ >= 0)
    {
        close(fd_);
        fd_ = -1;
    }
}

void vofa_reader::read_loop_hz(int hz)
{
    using clk = std::chrono::steady_clock;
    auto period = std::chrono::microseconds(1000000 / hz);
    auto next = clk::now() + period;

    constexpr std::array<uint8_t, 4> tail = {0x00, 0x00, 0x80, 0x7f};
    constexpr size_t expected_bytes = 9 * sizeof(float); // 36 bytes
    constexpr size_t max_buffer_size = 1024;             // Increased buffer size for safety

    while (!stop_requested_)
    {
        // Read incoming bytes
        uint8_t tmp[256];
        ssize_t n = ::read(fd_, tmp, sizeof(tmp));
        if (n > 0)
        {
            buffer_.insert(buffer_.end(), tmp, tmp + n);
        }
        // Frame extraction
        auto it = std::search(buffer_.begin(), buffer_.end(), tail.begin(), tail.end());
        if (it != buffer_.end())
        {
            size_t frame_end_pos = std::distance(buffer_.begin(), it);
            // Find frame tail
            if (frame_end_pos == expected_bytes)
            {
                std::vector<uint8_t> frame(buffer_.begin(), it);
                // decode_frame(frame);
                publish_frame(frame);
            }
            // Erase the processed frame and the tail from the buffer
            buffer_.erase(buffer_.begin(), it + tail.size());
        }
        // Prevent buffer from growing indefinitely if no tail is found
        if (buffer_.size() > max_buffer_size)
        {
            buffer_.erase(buffer_.begin(), buffer_.begin() + buffer_.size() / 2);
        }
        std::this_thread::sleep_until(next);
        next += period;
    }
}

void vofa_reader::decode_frame(const std::vector<uint8_t> &frame)
{
    float values[9];
    std::memcpy(values, frame.data(), sizeof(values));

    float pitch = values[0];
    float roll = values[1];
    float yaw = values[2];

    std::cout << "pitch - " << pitch << std::endl;
    std::cout << "roll - " << roll << std::endl;
    std::cout << "yaw - " << yaw << std::endl;
}

void vofa_reader::publish_frame(const std::vector<uint8_t> &frame)
{
    float values[9];
    std::memcpy(values, frame.data(), sizeof(values));

    exlcm::imu_cmd imu_msg;

    // Get current timestamp (microseconds since epoch)
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    imu_msg.timestamp = micros;

    for (int i = 0; i < 3; i++)
    {
        imu_msg.rpy[i] = values[i];
        imu_msg.gyro[i] = values[i + 3];
        imu_msg.acc[i] = values[i + 6];
    }
    // Push via LCM
    lcm_.publish(lcm_topic_, &imu_msg);

    std::cout << "Published IMU data: "
              << "  roll=" << imu_msg.rpy[0]
              << ", pitch=" << imu_msg.rpy[1]
              << ", yaw=" << imu_msg.rpy[2] << std::endl;
}

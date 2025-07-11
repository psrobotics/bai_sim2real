#include "motor_driver/CANInterface.hpp"
#include <fcntl.h> // Required for fcntl
#include <cerrno>  // Required for errno

namespace CAN_interface
{
    CANInterface::CANInterface(const char *socketName)
    {
        struct sockaddr_can addr;
        struct ifreq ifr;

        if ((socket_descrp_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            perror("CANInterface: Error While Opening CAN Socket");
            return; // Exit if socket creation fails
        }

        // --- Set socket to NON-BLOCKING mode ---
        int flags = fcntl(socket_descrp_, F_GETFL, 0);
        if (flags == -1)
        {
            perror("CANInterface: fcntl F_GETFL");
        }
        else
        {
            if (fcntl(socket_descrp_, F_SETFL, flags | O_NONBLOCK) == -1)
            {
                perror("CANInterface: fcntl F_SETFL O_NONBLOCK");
            }
        }
        // ---

        int loopback = 0;
        setsockopt(socket_descrp_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

        strcpy(ifr.ifr_name, socketName);
        ioctl(socket_descrp_, SIOCGIFINDEX, &ifr);

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_descrp_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            perror("CANInterface: Error while binding to the CAN Socket.");
        }
        else
        {
            std::cout << "The Socket Descriptor is: " << socket_descrp_ << " for " << socketName << std::endl;
        }
    }

    bool CANInterface::sendCANFrame(int can_id, const unsigned char *CANMsg)
    {
        struct can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = 8;
        memcpy(frame.data, CANMsg, 8);

        if (write(socket_descrp_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        {
            perror("CANInterface: Error writing to CAN Interface.");
            return false;
        }
        return true;
    }

    bool CANInterface::receiveCANFrame(unsigned char *CANMsg)
    {
        struct can_frame frame;
        ssize_t nbytes = read(socket_descrp_, &frame, sizeof(struct can_frame));

        if (nbytes < 0)
        {
            // EAGAIN or EWOULDBLOCK means no data was available. This is expected and NOT an error.
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                return false;
            }
            // An actual error occurred
            perror("CANInterface: Error Reading Data.");
            return false;
        }

        if (nbytes < sizeof(struct can_frame))
        {
            // Did not receive a full frame, treat as no data
            return false;
        }

        memcpy(CANMsg, frame.data, frame.can_dlc);
        return true;
    }

    CANInterface::~CANInterface()
    {
        if (close(socket_descrp_) < 0)
        {
            perror("CANInterface: Error Closing CAN Socket.");
        }
    }
}

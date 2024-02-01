#include "starq/can/can_socket.hpp"

#include <iostream>

namespace starq::can
{

    CANSocket::CANSocket(const std::string &interface)
        : interface_(interface), socket_(-1)
    {
    }

    CANSocket::~CANSocket()
    {
        if (socket_ >= 0)
        {
            close(socket_);
        }
    }

    bool CANSocket::connect()
    {

        if (socket_ >= 0)
        {
            std::cerr << "CAN socket already initialized." << std::endl;
            return false;
        }

        socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (socket_ < 0)
        {
            std::cerr << "Could not create CAN socket." << std::endl;
            return false;
        }

        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, interface_.c_str());
        ioctl(socket_, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            std::cerr << "Could not bind to CAN socket." << std::endl;
            return false;
        }

        std::cout << "CAN socket initialized." << std::endl;
        return true;
    }

    bool CANSocket::send(const uint8_t can_id, const uint8_t *data, const uint8_t size)
    {
        if (socket_ < 0)
        {
            std::cerr << "CAN socket is not initialized." << std::endl;
            return false;
        }

        if (size > CAN_MAX_DLEN)
        {
            std::cerr << "CAN frame data size is too large." << std::endl;
            return false;
        }

        struct can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = size;
        std::memcpy(frame.data, data, size);

        if (write(socket_, &frame, sizeof(frame)) <= 0)
        {
            std::cerr << "Could not send CAN frame." << std::endl;
            return false;
        }
        return true;
    }

    ssize_t CANSocket::receive(struct can_frame &frame)
    {
        if (socket_ < 0)
        {
            std::cerr << "CAN socket is not initialized." << std::endl;
            return -1;
        }

        ssize_t nbytes = read(socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0)
        {
            std::cerr << "Could not read CAN frame." << std::endl;
            return -1;
        }
        return nbytes;
    }

}
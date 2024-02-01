#ifndef STARQ_CAN__SOCKET_HPP_
#define STARQ_CAN__SOCKET_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <unistd.h>

#include <cstring>
#include <cstdint>
#include <memory>
#include <string>

#define MAX_CAN_ID 0x3F

namespace starq::can
{
    /// @brief Connect to a CAN interface
    class CANSocket
    {
    public:
        using Ptr = std::shared_ptr<CANSocket>;

        /// @brief Connect to a CAN interface.
        /// @param interface The name of the CAN interface.
        CANSocket(const std::string &interface);

        /// @brief Disconnect from the CAN interface.
        ~CANSocket();

        /// @brief Initialize the CAN interface.
        bool connect();

        /// @brief Send a CAN frame.
        /// @param can_id CAN ID.
        /// @param data Data to be sent.
        /// @param size Size of data.
        /// @return If the CAN frame was sent successfully.
        bool send(const uint8_t can_id, const uint8_t *data, const uint8_t size);

        /// @brief Receive a CAN frame.
        /// @param frame Frame to be filled.
        /// @return Size of the received frame.
        ssize_t receive(struct can_frame &frame);

    private:
        const std::string interface_;
        int socket_;
    };

}

#endif
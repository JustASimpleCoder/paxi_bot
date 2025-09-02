// Protocol support file
// Based on Arduino sample https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/blob/master/Arduino/hoverserial/hoverserial.ino
// From Emanuel FERU's hoverboard-firmware-hack-FOC firmware

#ifndef _HOVERBOARD_PROTOCOL_H
#define _HOVERBOARD_PROTOCOL_H

#include "paxi_hardware/serial_communication.hpp"
#include "paxi_hardware/hoverboard_protocol_struct.hpp"

#include "rclcpp/rclcpp.hpp"

#include <memory>

using paxi_serial::SerialPort;

class HoverboardProtocol{
    public:
        HoverboardProtocol() = default;
        HoverboardProtocol(const std::shared_ptr<SerialPort> & serial);
        HoverboardProtocol(const HoverboardProtocol &) = delete;
        HoverboardProtocol(HoverboardProtocol && other) noexcept;

        HoverboardProtocol& operator=(const HoverboardProtocol &) = delete;
        HoverboardProtocol& operator=(HoverboardProtocol &&) noexcept;


        bool send(const int16_t& steer, const int16_t& speed);
        //TODO: update to recieve error messages
        void receive();

        const SerialCommand& get_command() const;
        const SerialFeedback& get_feedback() const;

    private:
        SerialCommand command_;
        SerialFeedback feedback_;
        std::shared_ptr<SerialPort> serial_ptr_;


        uint16_t  start_frame;

        std::size_t msg_len = 0; 
        char* p_ = nullptr;
        char incoming_byte;
        char prev_byte;        
};

#endif
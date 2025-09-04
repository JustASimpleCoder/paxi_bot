// Protocol support file
// Based on Arduino sample https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC/blob/master/Arduino/hoverserial/hoverserial.ino
// From Emanuel FERU's hoverboard-firmware-hack-FOC firmware

#ifndef _HOVERBOARD_PROTOCOL_H
#define _HOVERBOARD_PROTOCOL_H


#include "paxi_hardware/serial_communication.hpp"
#include "paxi_hardware/hoverboard_protocol_struct.hpp"


#include "rclcpp/rclcpp.hpp"

#include <memory>

namespace paxi_hardware{


    class HoverboardProtocol{
        public:
            HoverboardProtocol() = default;
            HoverboardProtocol(const std::shared_ptr<SerialPort> & serial);

            HoverboardProtocol(const HoverboardProtocol &) = delete;
            HoverboardProtocol(HoverboardProtocol && other) noexcept;
            HoverboardProtocol& operator=(const HoverboardProtocol &) = delete;
            HoverboardProtocol& operator=(HoverboardProtocol &&) noexcept;


            bool send(const int16_t& steer, const int16_t& speed);
            bool process_byte(uint8_t incoming_byte);
            

            const SerialCommand& get_command() const noexcept;
            const SerialFeedback& get_feedback() const noexcept;
            
        private:
            SerialCommand command_;
            SerialFeedback feedback_;
            SerialFeedback new_feedback_;
            std::shared_ptr<SerialPort> serial_ptr_;


            uint8_t* p_ = nullptr;
            uint16_t  start_frame_;
            uint8_t prev_byte_;
            std::size_t msg_len_ = 0;       
    };

}// end of namespace paxi_hardware
#endif

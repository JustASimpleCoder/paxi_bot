#include "paxi_hardware/hoverboard_protocol.hpp"

namespace paxi_hardware{
    HoverboardProtocol::HoverboardProtocol(const std::shared_ptr<SerialPort> & serial) 
    :   command_{}, 
        feedback_{},
        new_feedback_{},
        serial_ptr_{serial},
        p_{0},
        start_frame_{0},
        prev_byte_{0},
        msg_len_{0}
    {}

    HoverboardProtocol::HoverboardProtocol(HoverboardProtocol && other) noexcept 
    :   command_(std::move(other.command_)), 
        feedback_(std::move(other.feedback_)),
        new_feedback_(std::move(other.new_feedback_)),
        serial_ptr_(std::move(other.serial_ptr_)),
        p_(std::move(other.p_)),
        start_frame_(std::move(other.start_frame_)),
        prev_byte_(std::move(other.prev_byte_)),
        msg_len_(std::move(other.msg_len_))
    {}

    HoverboardProtocol& HoverboardProtocol::operator=(HoverboardProtocol && other ) noexcept
    {
        if(this != &other){


            command_        =  std::move(other.command_);
            feedback_       =  std::move(other.feedback_);
            new_feedback_   =  std::move(other.new_feedback_);
            serial_ptr_     =  std::move(other.serial_ptr_);

            p_              =  std::move(other.p_);
            start_frame_    =  std::move(other.start_frame_);
            prev_byte_      =  std::move(other.prev_byte_);
            msg_len_        =  std::move(other.msg_len_);
        }

        return *this;
    }

    const SerialFeedback& HoverboardProtocol::get_feedback() const noexcept { return feedback_;}
    const SerialCommand& HoverboardProtocol::get_command() const noexcept   { return command_;}


    bool HoverboardProtocol::send(const int16_t& steer, const int16_t& speed){
        command_.start    = static_cast<uint16_t>(k_start_frame);
        command_.steer    = static_cast<int16_t>(steer);
        command_.speed    = static_cast<int16_t>(speed);
        command_.checksum = static_cast<uint16_t>(command_.start ^ command_.steer ^ command_.speed);

        if(serial_ptr_->write_port(command_) < 0){
            //std::cerr << "you done messed up\n";
            return false;
        }
        return true;
    }


    bool HoverboardProtocol::process_byte(uint8_t incoming_byte){        

        start_frame_ = (static_cast<uint16_t>(incoming_byte) << 8) | prev_byte_;
        // RCLCPP_DEBUG(rclcpp::get_logger("paxi_interface"),
        //             "byte [0x%02X], prev_byte [0x%02X], start_frame [0x%04X]",
        //             incoming_byte, prev_byte_, start_frame_
        // );

        if (start_frame_ == k_start_frame){
            p_ = reinterpret_cast<uint8_t*>(&new_feedback_);
            *p_++ = prev_byte_;
            *p_++ = incoming_byte;

            msg_len_ = 2;

        }else if (msg_len_ >= 2 && msg_len_ < sizeof(SerialFeedback)){

            *p_++ = incoming_byte;
            msg_len_++;
        }
        
        prev_byte_ = incoming_byte; 

        if (msg_len_ != sizeof(SerialFeedback)){
            if(msg_len_ > sizeof(SerialFeedback)){
                msg_len_ = 0;
            }

            return false;
        }

        uint8_t* raw_data = reinterpret_cast<uint8_t*>(&new_feedback_);
        RCLCPP_DEBUG(   
            rclcpp::get_logger("paxi_interface"),
            "Last 4 bytes: 0x%02X 0x%02X 0x%02X 0x%02X",
            raw_data[sizeof(SerialFeedback)-4],
            raw_data[sizeof(SerialFeedback)-3], 
            raw_data[sizeof(SerialFeedback)-2],
            raw_data[sizeof(SerialFeedback)-1]
        );

        const uint16_t checksum = static_cast<uint16_t>(  
            new_feedback_.start ^ 
            new_feedback_.cmd_l ^
            new_feedback_.cmd_r ^ 
            new_feedback_.speed_r_meas ^
            new_feedback_.speed_l_meas ^ 
            new_feedback_.bat_voltage ^
            new_feedback_.board_temp ^ 
            new_feedback_.cmd_led
        );

            msg_len_ = 0;

        // RCLCPP_DEBUG(
        //     rclcpp::get_logger("paxi_interface"),
        //     "Packet complete! Start: 0x%04X, Expected: 0x%04X, Checksum: 0x%04X, Expected: 0x%04X", 
        //     new_feedback_.start, k_start_frame, feedback_.checksum, checksum
        // );

        if (new_feedback_.start == k_start_frame && new_feedback_.checksum == checksum)
        {
            feedback_ = new_feedback_;
            return true;
        }

        return false;
    }
}
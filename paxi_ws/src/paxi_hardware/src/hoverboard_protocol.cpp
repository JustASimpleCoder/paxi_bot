#include "paxi_hardware/hoverboard_protocol.hpp"

namespace paxi_hardware{
    inline HoverboardProtocol::HoverboardProtocol(const std::shared_ptr<paxi_serial::SerialPort> & serial) 
    :   command_(), 
        feedback_(),
        serial_ptr_(serial)
    {}

    HoverboardProtocol::HoverboardProtocol(HoverboardProtocol && other) noexcept 
    :   command_(std::move(other.command_)), 
        feedback_(std::move(other.feedback_)),
        serial_ptr_(std::move(other.serial_ptr_))
    {}

    HoverboardProtocol& HoverboardProtocol::operator=(HoverboardProtocol && other ) noexcept
    {
        if(this != &other){
            command_    =   std::move(other.command_);
            feedback_   =   std::move(other.feedback_);
            serial_ptr_ =   std::move(other.serial_ptr_);
        }

        return *this;
    }

    const SerialFeedback& HoverboardProtocol::get_feedback() const noexcept { return feedback_;}
    const SerialCommand& HoverboardProtocol::get_command() const noexcept   { return command_;}


    inline bool HoverboardProtocol::send(const int16_t& steer, const int16_t& speed){
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


    inline bool HoverboardProtocol::process_byte(uint8_t incoming_byte){        

        start_frame_ = (static_cast<uint16_t>(incoming_byte) << 8) | prev_byte_;
        
        if (start_frame_ == k_start_frame){
            RCLCPP_INFO(rclcpp::get_logger("paxi_interface"),"Found start frame! 0x%04X", start_frame_);

            p_ = reinterpret_cast<uint8_t*>(&new_feedback_);
            *p_++ = prev_byte_;
            *p_++ = incoming_byte;

            msg_len_ = 2;
            prev_byte_ = incoming_byte;


        }else if (msg_len_ >= 2 && msg_len_ < sizeof(SerialFeedback)){
        // Otherwise just read the message content until the end
            *p_++ = incoming_byte;
            msg_len_++;
        }

        if (msg_len_ != sizeof(SerialFeedback)){
            return false;
        }
        // RCLCPP_INFO(rclcpp::get_logger("paxi_interface"),
        //    "Struct contents: start=0x%04X, cmd1=%d, cmd2=%d, speedR=%d, speedL=%d, wheelR=%d, wheelL=%d, batV=%d, temp=%d",
        //    feedback_.start, feedback_.cmd1, feedback_.cmd2, 
        //    feedback_.speedR_meas, feedback_.speedL_meas,
        //    feedback_.wheelR_cnt, feedback_.wheelL_cnt,
        //    feedback_.batVoltage, feedback_.boardTemp);


        //     uint8_t* raw_data = reinterpret_cast<uint8_t*>(&feedback_);
        //     RCLCPP_INFO(rclcpp::get_logger("paxi_interface"),
        //             "Last 4 bytes: 0x%02X 0x%02X 0x%02X 0x%02X",
        //             raw_data[sizeof(SerialFeedback)-4],
        //             raw_data[sizeof(SerialFeedback)-3], 
        //             raw_data[sizeof(SerialFeedback)-2],
        //             raw_data[sizeof(SerialFeedback)-1]);

        const uint16_t checksum = 
            static_cast<uint16_t>(  new_feedback_.start ^
                                    new_feedback_.cmd_l ^
                                    new_feedback_.cmd_r ^
                                    new_feedback_.speed_r_meas ^
                                    new_feedback_.speed_l_meas ^
                                    new_feedback_.wheel_r_cnt ^
                                    new_feedback_.wheel_l_cnt ^
                                    new_feedback_.left_dc_curr ^
                                    new_feedback_.right_dc_curr ^
                                    new_feedback_.bat_voltage ^
                                    new_feedback_.board_temp ^
                                    new_feedback_.cmd_led);

            msg_len_ = 0;
        RCLCPP_INFO(rclcpp::get_logger("paxi_interface"),"Packet complete! Start: 0x%04X, Expected: 0x%04X, Checksum: 0x%04X, Expected: 0x%04X", new_feedback_.start, k_start_frame, feedback_.checksum, checksum);

        if (new_feedback_.start == k_start_frame && feedback_.checksum == checksum)
        {
            RCLCPP_INFO(rclcpp::get_logger("paxi_interface"), "VALID PACKET!");
            feedback_ = new_feedback_;
            return true;
        }

        
        return false;
    }
}
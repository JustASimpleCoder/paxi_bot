#include "paxi_hardware/hoverboard_protocol.hpp"


    HoverboardProtocol::HoverboardProtocol(const std::shared_ptr<paxi_serial::SerialPort> & serial) 
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

    const SerialFeedback & HoverboardProtocol::get_feedback() const { return feedback_;}
    const SerialCommand& HoverboardProtocol::get_command() const { return command_; }


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


    bool HoverboardProtocol::validate_checksum(uint8_t prev_byte, uint8_t incoming_byte){        

        start_frame = (static_cast<uint16_t>(incoming_byte) << 8) | static_cast<uint8_t>(prev_byte);
        
        if (start_frame == k_start_frame){
            RCLCPP_INFO(rclcpp::get_logger("paxi_interface"),"Found start frame! 0x%04X", start_frame);

            p_ = reinterpret_cast<char*>(&feedback_);
            *p_++ = prev_byte;
            *p_++ = incoming_byte;
            msg_len = 2;

        }else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)){
        // Otherwise just read the message content until the end
            *p_++ = incoming_byte;
            msg_len++;
        }

        if (msg_len == sizeof(SerialFeedback)){
            RCLCPP_INFO(rclcpp::get_logger("paxi_interface"),
               "Struct contents: start=0x%04X, cmd1=%d, cmd2=%d, speedR=%d, speedL=%d, wheelR=%d, wheelL=%d, batV=%d, temp=%d",
               feedback_.start, feedback_.cmd1, feedback_.cmd2, 
               feedback_.speedR_meas, feedback_.speedL_meas,
               feedback_.wheelR_cnt, feedback_.wheelL_cnt,
               feedback_.batVoltage, feedback_.boardTemp);


                uint8_t* raw_data = reinterpret_cast<uint8_t*>(&feedback_);
                RCLCPP_INFO(rclcpp::get_logger("paxi_interface"),
                        "Last 4 bytes: 0x%02X 0x%02X 0x%02X 0x%02X",
                        raw_data[sizeof(SerialFeedback)-4],
                        raw_data[sizeof(SerialFeedback)-3], 
                        raw_data[sizeof(SerialFeedback)-2],
                        raw_data[sizeof(SerialFeedback)-1]);

                        
               const uint16_t checksum = 
                    static_cast<uint16_t>(  feedback_.start ^
                                            feedback_.cmd1 ^
                                            feedback_.cmd2 ^
                                            feedback_.speedR_meas ^
                                            feedback_.speedL_meas ^
                                            feedback_.wheelR_cnt ^
                                            feedback_.wheelL_cnt ^
                                            feedback_.left_dc_curr ^
                                            feedback_.right_dc_curr ^
                                            feedback_.batVoltage ^
                                            feedback_.boardTemp ^
                                            feedback_.cmdLed);


                RCLCPP_INFO(rclcpp::get_logger("paxi_interface"),"Packet complete! Start: 0x%04X, Expected: 0x%04X, Checksum: 0x%04X, Expected: 0x%04X", feedback_.start, k_start_frame, feedback_.checksum, checksum);

            if (feedback_.start == k_start_frame && feedback_.checksum == checksum)
            {
                RCLCPP_INFO(rclcpp::get_logger("paxi_interface"), "VALID PACKET!");
                return true;
            }
            msg_len = 0;
        }

        return false;
    }

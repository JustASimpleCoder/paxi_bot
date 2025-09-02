#include "paxi_hardware/hoverboard_protocol.hpp"



HoverboardProtocol::HoverboardProtocol(SerialPort * serial) 
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
        command_ = std::move(other.command_);
        feedback_ =std::move(other.feedback_);
        serial_ptr_ = std::move(other.serial_ptr_);
    }

    return *this;
}


void HoverboardProtocol::send(const int16_t& steer, const int16_t& speed){
    command_.start    = (uint16_t)k_start_frame;
    command_.steer    = (int16_t)steer;
    command_.speed    = (int16_t)speed;
    command_.checksum = (uint16_t)(command_.start ^ command_.steer ^ command_.speed);

    if(serial_ptr_->write_port(command_) < 0){
        //std::cerr << "you done messed up\n";
    }
}


void HoverboardProtocol::receive(){

    
    char incomingByte = serial_ptr_->read_port_byte();

    if(serial_ptr_->read_port() != "")
        start_frame = ((uint16_t)(incomingByte) << 8) | (uint8_t)prev_byte;

    if (start_frame == k_start_frame)
    {
      p_ = reinterpret_cast<char*>(&feedback_);

      *p_++ = prev_byte;
      *p_++ = incomingByte;
      msg_len = 2;
    }
    else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback))
    {
      // Otherwise just read the message content until the end
      *p_++ = incomingByte;
      msg_len++;
    }

    if (msg_len == sizeof(SerialFeedback))
    {
      uint16_t checksum = (uint16_t)(feedback_.start ^
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

      if (feedback_.start == k_start_frame && feedback_.checksum == checksum)
      {
        //data updated correctlly
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("paxi_hardware"), "Hoverboard checksum mismatch: %d vs %d", feedback_.checksum, checksum);
      }
     
      msg_len = 0;
    }

    prev_byte = incomingByte;

}




const SerialFeedback & HoverboardProtocol::get_feedback() const
{
    return feedback_;
}

const SerialCommand& HoverboardProtocol::get_command() const 
{
    return command_;
}

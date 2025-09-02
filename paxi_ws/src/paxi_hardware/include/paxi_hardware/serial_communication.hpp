#ifndef SERIAL_COMMUNICATION_HPP
#define SERIAL_COMMUNICATION_HPP

#include <iostream>
#include <fcntl.h>     
#include <unistd.h>     
#include <termios.h>    
#include <cstring>  


#include "paxi_hardware/hoverboard_protocol_struct.hpp"

namespace paxi_serial{
    class SerialPort {
        public:
            SerialPort();
            SerialPort(const std::string& port, std::uint32_t baud_rate);
            ~SerialPort();
            
            SerialPort(const SerialPort &) = delete;
            SerialPort& operator=(const SerialPort&) = delete;

            SerialPort(SerialPort&& other) noexcept;
            SerialPort& operator=(SerialPort&& other) noexcept;


            bool open_port();
            void close_port();
            bool is_open() const;
        
            ssize_t write_port(const std::string& data) const;

            ssize_t write_port(const SerialCommand cmd) const;

            std::string read_port();
            char read_port_byte();

            void set_port(const std::string& port_name);
            void set_baud(const std::uint32_t& baud_rate);

            std::string get_port() const;
            std::uint32_t get_baud() const;   
            int get_port_fd() const; 
        
        private:
            std::string m_port_;
            std::uint32_t m_baud_rate_;
            int m_fd_;
    };
}// end of namespace paxi_Serial

#endif
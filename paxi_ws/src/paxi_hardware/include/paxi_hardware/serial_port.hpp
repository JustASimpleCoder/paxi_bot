#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <iostream>
#include <fcntl.h>     
#include <unistd.h>     
#include <termios.h>   
 #include <poll.h>
#include <cstring>  


#include "paxi_hardware/hoverboard_protocol_struct.hpp"
#include "rclcpp/rclcpp.hpp"

namespace paxi_hardware{
    inline constexpr const char* LOGGER_SERIAL = "paxi_hardware_serial";
    class SerialPort {
        public:
            SerialPort();
            SerialPort(const std::string& port, std::uint32_t baud_rate);
            ~SerialPort();
            
            SerialPort(const SerialPort& ) = delete;
            SerialPort& operator=(const SerialPort& ) = delete;

            SerialPort(SerialPort&& other ) noexcept;
            SerialPort& operator=(SerialPort&& other ) noexcept;

            bool open_port();
            void close_port();
            bool is_open() const;
        
            ssize_t write_port(const std::string& data) const;
            ssize_t write_port(const SerialCommand& cmd) const;

            ssize_t read_into_uint8_buf(uint8_t* buffer, std::size_t max_len);

            bool set_port(const std::string& port_name);
            bool set_baud(const std::uint32_t& baud_rate);

            void update_connection();

            inline std::string      get_port_name()          const{return port_;}
            inline std::uint32_t    get_baud()          const{return baud_rate_;}   
            inline int              get_port_fd()       const{return fd_;} 
            inline bool             is_connected()   const{return connected_;} 
       
        private:
            std::string port_;
            std::uint32_t baud_rate_;
            int fd_;
            bool connected_;
    };
}// end of namespace paxi_hardware

#endif
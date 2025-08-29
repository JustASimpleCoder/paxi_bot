#ifndef SERIAL_COMMUNICATION_HPP
#define SERIAL_COMMUNICATION_HPP

#include <iostream>
#include <fcntl.h>     
#include <unistd.h>     
#include <termios.h>    
#include <cstring>  

namespace paxi_serial{
    class SerialCommunication {
        public:
            SerialCommunication();
            SerialCommunication(const std::string& port, uint32_t baud_rate);
            ~SerialCommunication();

            bool open_port();
            void close_port();
            bool is_open() const;
        
            ssize_t write_port(const std::string& data);
            std::string read_port();

            void set_port(std::string port_name);
            void set_baud(u_int32_t baud_rate);

            std::string get_port() const;
            u_int32_t get_baud() const;   
            int get_port_fd() const; 
        
        private:
            std::string m_port_;
            u_int32_t m_baud_rate_;
            int m_fd_;
    };
}// end of namespace paxi_Serial

#endif
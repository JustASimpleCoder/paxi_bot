#ifndef HARDWARE_WORKER_HPP
#define HARDWARE_WORKER_HPP

#include <thread>
#include <mutex>
#include <atomic>

#include "paxi_hardware/encoder.hpp"
#include "paxi_hardware/hoverboard_protocol.hpp"
#include "paxi_hardware/imu.hpp"
#include "paxi_hardware/paxi_interface_node.hpp"
#include "paxi_hardware/serial_port.hpp"
#include "paxi_hardware/utility.hpp"

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/hardware_info.hpp"

namespace paxi_hardware
{
  

  class HardwareWorker
  {
    public:
        HardwareWorker();
        ~HardwareWorker() = default;

        HardwareWorker(const HardwareWorker& ) = delete;
        HardwareWorker& operator=(const HardwareWorker& ) = delete;

        HardwareWorker(HardwareWorker&& ) noexcept = default;
        HardwareWorker& operator=(HardwareWorker&& ) noexcept = delete;

        void init_zero_state_interfaces(const hardware_interface::HardwareInfo& hardware_info);
        void start_worker();  
        void stop_worker();

        void write_command();
        inline SerialCommand update_encoder();

        bool set_hardware_params_from_xacro(const hardware_interface::HardwareInfo hardware_info);
        void write_hover_commmand(const SerialCommand& hover_cmd);

        inline bool open_serial_port(){
            std::scoped_lock lock(mutex_serial_);
            return serial_port_.open_port();
        }

        inline bool is_serial_port_open(){
            std::scoped_lock lock(mutex_serial_);
            return serial_port_.is_open();
        }

        inline void close_serial_port(){
            std::scoped_lock lock(mutex_serial_);
            serial_port_.close_port();
        }

        inline void get_state_interface(
            std::vector<double>& state_positions, 
            std::vector<double>& state_velocities) const
        {
            std::lock_guard lock(mutex_state_);
            state_positions = state_interface_positions_;
            state_velocities = state_interface_velocities_;
        }

        inline double* get_state_interface_position_ptr(size_t index) {
            std::scoped_lock lock(mutex_state_);
            return &state_interface_positions_[index];
        }
    
        inline double* get_state_interface_velocity_ptr(size_t index)  {
            std::scoped_lock lock(mutex_state_);
            return &state_interface_velocities_[index];
        }

        inline double* get_hardware_commands_ptr(size_t index)  {
            std::scoped_lock lock(mutex_state_);
            return &hw_commands_[index];
        }

    private:
    /* 
    * Chosen to place this on stack versus heap with smart pointers.
    * Classses are simple enough with small & mostly primitive type resources
    */  
        SerialPort serial_port_;
        HoverboardProtocol protocol_;
        EncoderKinematics encoder_;
        ImuProcessing imu_;

        std::vector<double> state_interface_positions_;
        std::vector<double> state_interface_velocities_;
        std::vector<double> hw_commands_;    

        std::array<uint8_t, CONTROLLER_FEEDBACK_BUFFER> feedback_buf_;

        std::thread protocol_worker_thread_;
        std::atomic<bool> worker_running_;

        mutable std::mutex mutex_state_;
        mutable std::mutex mutex_serial_;

        std::unique_ptr<PaxiInterfaceNode> paxi_interface_node_;
        rclcpp::Clock::SharedPtr cached_clock_;

        void worker_loop();
        void update_paxi_interface_state();
        void protocol_parsing_loop(const ssize_t & bytes_read);
        void get_new_feedback_buffer(ssize_t & bytes_read);

  };
}  //end of namespace paxi_hardware

#endif
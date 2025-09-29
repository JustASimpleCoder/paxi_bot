#ifndef PROTOCOL_WORKER_HPP
#define PROTOCOL_WORKER_HPP

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
  

  class ProtocolWorker
  {
    public:
        // ProtocolWorker();
        ProtocolWorker( SerialPort& serial_port, 
                        HoverboardProtocol& protocol, 
                        EncoderKinematics& encoder,
                        ImuProcessing& imu
        );

        ~ProtocolWorker() = default;

        ProtocolWorker(const ProtocolWorker& ) = delete;
        ProtocolWorker& operator=(const ProtocolWorker& ) = delete;

        ProtocolWorker(ProtocolWorker&& ) noexcept = default;
        ProtocolWorker& operator=(ProtocolWorker&& ) noexcept;


        void init_zero_state_interfaces(const hardware_interface::HardwareInfo& hardware_info);
        void start_worker();  
        void stop_worker();

        inline std::mutex& get_state_mutex() const { return mutex_state_; }
        inline std::mutex& get_serial_mutex() const { return mutex_serial_; }

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
    
        inline double* get_state_interface_velocity_ptr(size_t index) {
            std::scoped_lock lock(mutex_state_);
            return &state_interface_velocities_[index];
        }


    private:
        /* 
         * puprosefully using raw pointer here. we are not calling new delete
         * all pointers below should point to a stack allocated object in PaxiINterface 
         * (paxi interface is created once and exists for the entire liftime of the paxi_hardware lib plugin)
         * 
         */
        
        SerialPort& serial_port_;
        HoverboardProtocol& protocol_;
        EncoderKinematics& encoder_;
        ImuProcessing& imu_;

        std::vector<double> state_interface_positions_;
        std::vector<double> state_interface_velocities_;    

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
#ifndef PAXI_INTERFACE_HPP
#define PAXI_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_msgs/msg/imu.hpp"

#include <array>
#include <memory>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>


#include "paxi_hardware/serial_communication.hpp"
#include "paxi_hardware/hoverboard_protocol.hpp"
#include "paxi_hardware/paxi_interface_node.hpp"
#include "paxi_hardware/utility.hpp"
#include "paxi_hardware/encoder.hpp"



namespace paxi_hardware{


    inline constexpr const char* LOGGER_HARDWARE = "paxi_hardware";


    // constexpr double Q30 = 1073741824.0;

    // // used to convert values recieved from controller to 
    // // values that make more sense for the hoverboard protoco

    // static constexpr double SPEED_SCALE = 500.0;
    // static constexpr double STEER_SCALE = 500.0;
    
    // static const double PI = 3.14159265358979323846; 
    // static const double RPM_TO_RAD_S = PI / 30.0;


    constexpr int ENCODER_MIN = 0;
    constexpr int ENCODER_MAX = 9000;
    constexpr double ENCODER_LOW_WRAP_FACTOR = 0.3;
    constexpr double ENCODER_HIGH_WRAP_FACTOR = 0.7;

    constexpr int TICKS_PER_ROTATION = 90; 

    class PaxiInterface : public hardware_interface::SystemInterface{

        public:
            PaxiInterface() = default;

            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
            hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
            hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
            hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            
            hardware_interface::return_type prepare_command_mode_switch(    
                const std::vector<std::string> &,
                const std::vector<std::string> &) override;
            
            hardware_interface::return_type perform_command_mode_switch(    
                const std::vector<std::string> &,
                const std::vector<std::string> &) override;

            hardware_interface::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration &period) override;

            hardware_interface::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration &period) override;

            bool get_params_from_xacro(const hardware_interface::HardwareInfo &hardware_info);
            bool check_joints_and_state(const hardware_interface::HardwareInfo &hardware_info);

            void publish_real_time() const;
            void update_imu(const rclcpp::Time time, const SerialFeedback &feedback); 

        private:

            std::shared_ptr<SerialPort> serial_communication_;
            std::shared_ptr<HoverboardProtocol> protocol_;

            std::string serial_port_name_;
            std::string baud_rate_;

            bool is_connected_;
            bool first_read_pass_;

            int port_fd_;


            std::unique_ptr<EncoderKinematics> encoder_; 

            rclcpp::Time last_read_;
            rclcpp::Time last_publish_time_;
            int direction_correction_ = 1;

            std::vector<double> state_interface_positions_;
            std::vector<double> state_interface_velocities_;
            std::vector<double> hw_commands_;

            std::array<uint8_t, 1024> feedback_buf_;
            std::unique_ptr<PaxiInterfaceNode> paxi_interface_node_;
            sensor_msgs::msg::Imu imu_msg_;
    };
}// end of namespace paxi_hardware

#endif
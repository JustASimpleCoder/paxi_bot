#ifndef PAXI_INTERFACE_HPP
#define PAXI_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "hardware_interface/handle.hpp"

//#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp_lifecycle/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"


#include <array>
#include <memory>
#include <numbers>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>



#include "paxi_hardware/serial_communication.hpp"
#include "paxi_hardware/hoverboard_protocol.hpp"


namespace paxi_hardware{

    //using paxi_serial::SerialPort;


    constexpr double SPEED_SCALE = 1000.0;
    constexpr double STEER_SCALE = 1000.0;
    
    const double PI = 3.14159265358979323846; 
    //const double deg_to_rad = PI / 180.0;

    const double RPM_to_rad_s = PI / 30.0;

    constexpr int ENCODER_MIN = 0;
    constexpr int ENCODER_MAX = 9000;
    constexpr double ENCODER_LOW_WRAP_FACTOR = 0.3;
    constexpr double ENCODER_HIGH_WRAP_FACTOR = 0.7;

    constexpr int TICKS_PER_ROTATION = 90; 

    enum class Wheel : std::size_t {
        LEFT = 0,
        RIGHT = 1,
        COUNT = 2
    };

    // template<typename T>
    // constexpr std::size_t enum_to_index(T pos) noexcept{
    //     return static_cast<std::size_t>(pos);
    // }

    // helper function to conver WheelPostion enum to appropriate index
    constexpr std::size_t to_index(Wheel pos) noexcept{
        return static_cast<std::size_t>(pos);

    }

    class PaxiInterfaceNode : public rclcpp::Node{
        public:
            PaxiInterfaceNode();
            ~PaxiInterfaceNode() = default;

            template<typename MsgT, typename ValueT>
            void publish_data(const std::shared_ptr<rclcpp::Publisher<MsgT>>& pub, const ValueT& value){
                MsgT msg;
                msg.data = value;
                pub->publish(msg);
            }

            template<typename MsgT, typename ValueT>
            void publish_data(
                const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> pub, 
                const ValueT& value, 
                std::size_t idx)
            {
                MsgT msg;
                msg.data = value;
                pub[idx]->publish(msg);
            }

            const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> get_position_pubs() const;
            const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> get_velocity_pubs() const;
            const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> get_command_pubs() const;
            const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> get_current_pubs() const;

            const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr get_voltage_pubs() const;
            const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr get_temp_pubs() const;
            const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr get_connected_pubs() const;


        private:

            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> position_pubs_;
            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> velocity_pubs_;
            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> command_pubs_;
            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, to_index(Wheel::COUNT)> current_pubs_;
            
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pubs_;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pubs_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pubs_;

        };

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
            void update_encoders(const rclcpp::Time &time, int16_t right, int16_t left);
                
            void forward_kinematics();
            void inverse_kinematics();

        private:

            std::shared_ptr<SerialPort> serial_communication_;
            std::shared_ptr<HoverboardProtocol> protocol_;

            std::string serial_port_;
            std::string baud_rate_;

            int port_fd_;

            double wheel_radius_;
            double wheel_separation_;
            double max_velocity_;

            double wheel_omega_l_; 
            double wheel_omega_r_; 
            double wheel_vel_l_; 
            double wheel_vel_r_;

            double hoverboard_steer_;
            double hoverboard_speed_;

            rclcpp::Time last_read_;
            int direction_correction_ = 1;
            bool first_read_pass_;
            // Last known encoder values
            int16_t last_wheel_count_r_;
            int16_t last_wheel_count_l_;
            // Count of full encoder wraps
            int mult_r_;
            int mult_l_;
            // Thresholds for calculating the wrap
            int low_wrap_;
            int high_wrap_;




            std::vector<double> state_interface_positions_;
            std::vector<double> state_interface_velocities_;
            std::vector<double> hw_commands_;

            std::unique_ptr<PaxiInterfaceNode> paxi_interface_node_;
    };
}// namespace paxi_hardware

#endif
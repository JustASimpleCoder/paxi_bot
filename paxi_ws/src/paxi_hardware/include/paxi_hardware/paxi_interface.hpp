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

    // used to convert values recieved from controller to 
    // values that make more sense for the hoverboard protocol ()
    static constexpr double SPEED_SCALE = 1000.0;
    static constexpr double STEER_SCALE = 1000.0;
    
    static const double PI = 3.14159265358979323846; 
    static const double RPM_TO_RAD_S = PI / 30.0;


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

    // helper function to conver WheelPostion enum to appropriate index
    constexpr std::size_t to_index(Wheel pos) noexcept{
        return static_cast<std::size_t>(pos);

    }

    //used in templated to ensure arrays have at least two indices
    static constexpr std::size_t WHEEL_COUNT  = to_index(Wheel::COUNT);

    class PaxiInterfaceNode : public rclcpp::Node{
        public:
            PaxiInterfaceNode();
            ~PaxiInterfaceNode() = default;

            template<typename MsgT, typename ValueT>
            void publish_data(const std::shared_ptr<typename rclcpp::Publisher<MsgT>>& pub, const ValueT& value){
                MsgT msg;
                msg.data = value;
                pub->publish(msg);
            }

            template<typename MsgT, typename ValLeftT, typename ValRightT>
            void publish_data(
                const std::array<std::shared_ptr<typename rclcpp::Publisher<MsgT>>, WHEEL_COUNT> &pub, 
                const ValLeftT& l_value, 
                const ValRightT& r_value
            )
            {
                static_assert(WHEEL_COUNT == 2, "Wheel count needs to be 2, please check enum class Wheel");
                publish_data(pub[to_index(Wheel::LEFT)],  l_value);
                publish_data(pub[to_index(Wheel::RIGHT)], r_value);
            }

            const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> get_position_pubs() const;
            const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> get_velocity_pubs() const;
            const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> get_command_pubs() const;
            const std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> get_current_pubs() const;

            const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr get_voltage_pubs() const;
            const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr get_temp_pubs() const;
            const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr get_connected_pubs() const;


        private:

            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> position_pubs_;
            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> velocity_pubs_;
            std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> command_pubs_;
            // std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, WHEEL_COUNT> current_pubs_;
            
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
            void update_encoders(const rclcpp::Time &time, const rclcpp::Duration & duration, int16_t r_rpm, int16_t l_rpm);    
                
            void forward_kinematics();
            void inverse_kinematics();

        private:

            std::shared_ptr<SerialPort> serial_communication_;
            std::shared_ptr<HoverboardProtocol> protocol_;

            std::string serial_port_;
            std::string baud_rate_;

            bool is_connected_;
            bool first_read_pass_;

            int port_fd_;

            double wheel_radius_;
            double wheel_separation_;
            double max_velocity_;

            double wheel_omega_l_ = 0.0; 
            double wheel_omega_r_ = 0.0; 
            double wheel_vel_l_ = 0.0; 
            double wheel_vel_r_ = 0.0;

            double hoverboard_steer_ = 0.0;
            double hoverboard_speed_ = 0.0;

            rclcpp::Time last_read_;
            rclcpp::Time last_read_enc_;
            double prev_l_rad_per_sec_ = 0.0;
            double prev_r_rad_per_sec_ = 0.0;
            bool first_read_enc_;

            int direction_correction_ = 1;

            std::vector<double> state_interface_positions_;
            std::vector<double> state_interface_velocities_;
            std::vector<double> hw_commands_;

            std::unique_ptr<PaxiInterfaceNode> paxi_interface_node_;
    };
}// end of namespace paxi_hardware

#endif
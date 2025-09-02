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



#include "serial_communication.hpp"
#include "hoverboard_protocol.hpp"

namespace paxi_hardware{

    namespace hw = hardware_interface;
    using paxi_serial::SerialPort;
    
    enum class Wheel : std::size_t {
        LEFT = 0,
        RIGHT = 1,
        COUNT = 2
    };


    // template<typename T>
    // constexpr std::size_t enum_to_index(T pos) noexcept{
    //     return static_cast<std::size_t>(pos);
    // }i

    // helper function to conver WheelPostion enum to appropriate index
    constexpr std::size_t to_index(Wheel pos) noexcept{
        return static_cast<std::size_t>(pos);

    }

    const double PI = 3.14159265358979323846; 
    const double deg_to_rad = PI / 180.0;

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

    class PaxiInterface : public hw::SystemInterface{

        public:
            PaxiInterface() = default;


            hw::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
            hw::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
            hw::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
            hw::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
            hw::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
            hw::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
            hw::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

            std::vector<hw::StateInterface> export_state_interfaces() override;
            std::vector<hw::CommandInterface> export_command_interfaces() override;
            
            hw::return_type prepare_command_mode_switch(    
                const std::vector<std::string> &,
                const std::vector<std::string> &) override;
            
            hw::return_type perform_command_mode_switch(    
                const std::vector<std::string> &,
                const std::vector<std::string> &) override;

            hw::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration &period) override;

            hw::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration &period) override;

            bool get_params_from_xacro(const hw::HardwareInfo &hardware_info);
            bool check_joints_and_state(const hw::HardwareInfo &hardware_info);

            void publish_real_time() const;

        private:

            std::shared_ptr<SerialPort> serial_communication_;
            std::shared_ptr<HoverboardProtocol> protocol_;

            std::string serial_port_;
            std::string baud_rate_;

            int port_fd_;

            double wheel_radius_;
            double wheel_separation_;
            double max_velocity_;


            std::vector<double> state_interface_positions_;
            std::vector<double> state_interface_velocities_;
            std::vector<double> command_interface_commands_;

            std::unique_ptr<PaxiInterfaceNode> paxi_interface_node_;
    };
}// namespace paxi_hardware

#endif
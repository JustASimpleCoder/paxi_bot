#ifndef PAXI_INTERFACE_HPP
#define PAXI_INTERFACE_HPP

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "paxi_hardware/encoder.hpp"
#include "paxi_hardware/hoverboard_protocol.hpp"
#include "paxi_hardware/imu.hpp"
#include "paxi_hardware/paxi_interface_node.hpp"
#include "paxi_hardware/serial_port.hpp"
#include "paxi_hardware/utility.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace paxi_hardware
{

inline constexpr const char * LOGGER_HARDWARE = "paxi_hardware";

class PaxiInterface : public hardware_interface::SystemInterface
{
public:
  PaxiInterface() = default;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>&, const std::vector<std::string>&) override;
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>&, const std::vector<std::string>&) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  bool get_params_from_xacro(const hardware_interface::HardwareInfo& hardware_info);
  bool check_joints_and_state(const hardware_interface::HardwareInfo& hardware_info);

private:
  // Chosen to place this on stack versus heap with smart pointers.
  // Classses are simple enough with small & moslty primitive type resources
  SerialPort serial_port_;
  HoverboardProtocol protocol_;
  EncoderKinematics encoder_;
  ImuProcessing imu_;

  std::array<uint8_t, CONTROLLER_FEEDBACK_BUFFER> feedback_buf_;
  std::unique_ptr<PaxiInterfaceNode> paxi_interface_node_;

  std::vector<double> state_interface_positions_;
  std::vector<double> state_interface_velocities_;
  std::vector<double> hw_commands_;
};
}  // end of namespace paxi_hardware

#endif
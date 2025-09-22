#ifndef HOVERBOARD_PROTOCOL_STRUCT_HPP
#define HOVERBOARD_PROTOCOL_STRUCT_HPP

#include <cstdint>
#include <utility>

namespace paxi_hardware
{

struct __attribute__((packed)) SerialCommand
{
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
};

struct __attribute__((packed)) SerialFeedback
{
  uint16_t start;
  int16_t cmd_l;
  int16_t cmd_r;
  int16_t speed_r_meas;
  int16_t speed_l_meas;
  int16_t bat_voltage;
  int16_t board_temp;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t quat_w_low;
  int16_t quat_w_high;
  int16_t quat_x_low;
  int16_t quat_x_high;
  int16_t quat_y_low;
  int16_t quat_y_high;
  int16_t quat_z_low;
  int16_t quat_z_high;
  int16_t euler_pitch;
  int16_t euler_roll;
  int16_t euler_yaw;
  int16_t temperature;
  uint16_t sensors;
  uint16_t cmd_led;
  uint16_t checksum;
};
}  //end of namespace paxi_hardware

#endif
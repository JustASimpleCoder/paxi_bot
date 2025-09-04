#ifndef HOVERBOARD_PROTOCOL_STRUCT_HPP
#define HOVERBOARD_PROTOCOL_STRUCT_HPP

//#define START_FRAME 0xABCD
constexpr uint16_t k_start_frame = 0xABCD;

#include <cstdint>
#include <utility>


namespace paxi_hardware{

   struct __attribute__((packed)) SerialCommand {
      uint16_t start;
      int16_t steer;
      int16_t speed;
      uint16_t checksum;
   };

   struct __attribute__((packed)) SerialFeedback {
      uint16_t start;
      int16_t cmd_l;
      int16_t cmd_r;
      int16_t speed_r_meas;
      int16_t speed_l_meas;
      // int16_t wheel_r_cnt;
      // int16_t wheel_l_cnt;
      // int16_t left_dc_curr;
      // int16_t right_dc_curr;
      int16_t bat_voltage;
      int16_t board_temp;
      uint16_t cmd_led;
      uint16_t checksum;
   };
}

#endif
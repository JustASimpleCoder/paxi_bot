#ifndef HOVERBOARD_PROTOCOL_STRUCT_HPP
#define HOVERBOARD_PROTOCOL_STRUCT_HPP



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
      int16_t bat_voltage;
      int16_t board_temp;
      uint16_t cmd_led;
      uint16_t checksum;
   };
}//end of namespace paxi_hardware

#endif
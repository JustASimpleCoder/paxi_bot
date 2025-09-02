#ifndef HOVERBOARD_PROTOCOL_STRUCT_HPP
#define HOVERBOARD_PROTOCOL_STRUCT_HPP

//#define START_FRAME 0xABCD
constexpr uint16_t k_start_frame = 0xABCD;

#include <cstdint>
#include <utility>

typedef struct {
   uint16_t start{k_start_frame};
   int16_t  steer{0};
   int16_t  speed{0};
   uint16_t checksum{0};
} SerialCommand;

typedef struct {
   uint16_t start{k_start_frame};
   int16_t  cmd1{0};
   int16_t  cmd2{0};
   int16_t  speedR_meas{0};
   int16_t  speedL_meas{0};
   int16_t  wheelR_cnt{0};
   int16_t  wheelL_cnt{0}; 
   int16_t  left_dc_curr{0};
   int16_t  right_dc_curr{0};   
   int16_t  batVoltage{0};
   int16_t  boardTemp{0};
   uint16_t cmdLed{0};
   uint16_t checksum{0};
} SerialFeedback;

#endif
#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <array>

namespace paxi_hardware
{    
    enum class Wheel : std::size_t {
        LEFT = 0,
        RIGHT = 1,
        COUNT = 2
    };

    // helper function to conver WheelPostion enum to appropriate index
    constexpr std::size_t to_index(Wheel pos) noexcept{
        return static_cast<std::size_t>(pos);

    }
    //used in templated to ensure arrays have at least two indices and useful for arrays storing wheel data
    static constexpr std::size_t WHEEL_COUNT  = static_cast<std::size_t>(Wheel::COUNT);

    // comes from sidebaord imu processing and madwick algorithm
    static constexpr double Q30                         = 1073741824.0;

    // Quaternions are passed by FeedbackStruct in hoverboar_protocol_struct.hpp as int16_t variables
    // sidebaord computes quaternions using madgiwck from raw MPU6050 accel/gryro using double internally (QuaternionDouble struct) to maintain precision.
    // quaternion values are then casted to int32_t variables and stored internally there
    // when we pass the feedback, they need to fit into 16 bit data packets, (int16_t)
    // so the feedback sends the int32_T quat shifted down 16 bits (divided by 2^4), ieie. feedback.quat_x = (int16_t)(quat.x >> 16).
    // normally we want to normalize this data by casting the 32 but to a double and dividing by 2^30 (Q30). becuase we bit shiftred
    // we already divided by 2^16, so no we just need to divide by 2^14, to have the same overall effect of division by 2^30
    static constexpr double QUAT_SCALE_FROM_PROTOCOL    = (1 << 14);

    static constexpr double ACCEL_TO_G                  = 16384.00;    
    static constexpr double GYRO_TO_DEG_S               = 16.4;
                                       
    
    // used to convert values recieved from controller to 
    // values that make more sense for the hoverboard protoco
    static constexpr double SPEED_SCALE = 500.0;
    static constexpr double STEER_SCALE = 500.0;
   
    // Useful math stuff
    static const double PI = 3.14159265358979323846; 
    static const double RPM_TO_RAD_S = PI / 30.0;
    
    //internal buffer reads a sample of uint_8t into a buffer, 1024 more than enought, each fedback stuct is about ~44 bytes, so can fit like 20+
    static const std::size_t CONTROLLER_FEEDBACK_BUFFER = 1024;

} // end of namespace paxi_hardware
#endif
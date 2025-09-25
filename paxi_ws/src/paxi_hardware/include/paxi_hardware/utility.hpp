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
    constexpr std::size_t to_index(Wheel pos) noexcept { return static_cast<std::size_t>(pos); }

    //used in templated to ensure arrays have at least two indices and useful for arrays storing wheel data
    static constexpr std::size_t WHEEL_COUNT = static_cast<std::size_t>(Wheel::COUNT);

    // comes from sidebaord imu processing and madwick algorithm, used to scale quaternion down to correct unit
    static constexpr double Q30 = 1073741824.0;

    // converts raw acceleration data (m/s^2) from the MPU6050 to approriate gravity units (16,384 LSB/g)
    static constexpr double ACCEL_TO_G = 16384.00;
    //converts rar gyro data (DPS units) from the MPU6050 to degree persecond (16.4 LSB/(degree/s))
    static constexpr double GYRO_TO_DEG_S = 16.4;

    // used to convert values recieved from controller to
    // values that make more sense for the hoverboard protoco
    static constexpr double SPEED_SCALE = 500.0;
    static constexpr double STEER_SCALE = 500.0;

    // Useful math stuff
    static const double PI = 3.14159265358979323846;
    static const double RPM_TO_RAD_S = PI / 30.0;

    /* 
    Internal buffer reads a sample of uint_8t feedback data into a buffer, 
    1024 more than enought, each feedback stuct is about ~44 bytes, so can fit like 20+
     */

    static const std::size_t CONTROLLER_FEEDBACK_BUFFER = 256;

}  // end of namespace paxi_hardware
#endif
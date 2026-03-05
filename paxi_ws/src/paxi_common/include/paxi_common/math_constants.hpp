#ifndef PAXI_COMMON__MATH_CONSTANTS_HPP_
#define PAXI_COMMON__MATH_CONSTANTS_HPP_

namespace paxi_common::math_constants
{
// Constant for famous PI 3 constant
inline constexpr double PI = 3.14159265358979323846;

// converts RPM to rad/s
inline constexpr double RPM_TO_RAD_S = PI / 30.0;

// converts rad/s to rpm
inline constexpr double RAD_S_TO_RPM = 30.0 / PI;

// Degree to rad unit conversion
inline constexpr double DEG_TO_RAD = PI / 180.0;

// RAD to Degree unit conversion
inline constexpr double RAD_TO_DEG = 180.0 / PI;

}  // namespace paxi_hardware

#endif  // PAXI_COMMON__HARDWARE_TOPIC_NAMES_HPP_
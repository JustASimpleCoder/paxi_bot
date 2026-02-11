#ifndef UTILITY_HPP
#define UTILITY_HPP_

#include <cstdint>
/*
     Enum to store wheel index and total wheel count
          LEFT = 0,
          RIGHT = 1,
          COUNT = 2
*/

enum class Wheel : std::size_t
{
  LEFT = 0,
  RIGHT = 1,
  COUNT = 2
};

/*
     Helper function to conver WheelPostion enum to appropriate index
*/
constexpr std::size_t to_index(Wheel pos) noexcept {return static_cast<std::size_t>(pos);}

/*
     Used in templates to ensure arrays have at least two indices
     and useful for arrays storing wheel data
*/
inline constexpr std::size_t WHEEL_COUNT = static_cast<std::size_t>(Wheel::COUNT);

/*
     useful math stuff
*/
inline constexpr double PI = 3.14159265358979323846;
inline constexpr double RPM_TO_RAD_S = PI / 30.0;
inline constexpr double RAD_S_TO_RPM = 30.0 / PI;

/*
     sample size of data collection
*/
inline constexpr std::size_t SAMPLE_SIZE_RPM = 1000;


/*
     topic names
*/
inline constexpr const char * TOPIC_CONTROLLER_CMD = "cmd_controller";
inline constexpr const char * TOPIC_HOVER_FEEDBACK = "hover/feedback";

#endif  // UTILITY_HPP_

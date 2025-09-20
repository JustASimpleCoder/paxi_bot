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
    static constexpr std::size_t WHEEL_COUNT  = to_index(Wheel::COUNT);



    constexpr double Q30 = 1073741824.0;

    // used to convert values recieved from controller to 
    // values that make more sense for the hoverboard protoco
    static constexpr double SPEED_SCALE = 500.0;
    static constexpr double STEER_SCALE = 500.0;
    

    
    static const double PI = 3.14159265358979323846; 
    static const double RPM_TO_RAD_S = PI / 30.0;


    // constexpr int ENCODER_MIN = 0;
    // constexpr int ENCODER_MAX = 9000;
    // constexpr double ENCODER_LOW_WRAP_FACTOR = 0.3;
    // constexpr double ENCODER_HIGH_WRAP_FACTOR = 0.7;


} // end of namespace paxi_hardware





#endif
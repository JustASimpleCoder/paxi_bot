#ifndef ENCODER_HPP
#define ENCODER_HPP           


#include "rclcpp/rclcpp.hpp"

#include "paxi_hardware/utility.hpp"

namespace paxi_hardware{

    inline constexpr const char* LOGGER_ENCODER = "paxi_hardware_encoder";

    class EncoderKinematics
    {

    public:
        EncoderKinematics();
        ~EncoderKinematics() = default;
        
        void update_encoders(const rclcpp::Time &time, const rclcpp::Duration & duration, int16_t r_rpm, int16_t l_rpm, std::vector<double> &state_positions );
        void forward_kinematics(const std::vector<double> &hw_commands);
        void inverse_kinematics();


        bool set_wheel_radius(const double & radius);
        bool set_wheel_separation(const double & separation);
        bool set_max_velocity(const double & velocity);


    inline double get_hover_steer() const{
        return hoverboard_steer_;
    }
    inline double get_hover_speed() const{
        return hoverboard_speed_;
    }  

    private:
           
        double wheel_radius_;
        double wheel_separation_;
        double max_velocity_;

        double wheel_omega_l_ = 0.0; 
        double wheel_omega_r_ = 0.0; 
        double wheel_vel_l_ = 0.0; 
        double wheel_vel_r_ = 0.0;

        double hoverboard_steer_ = 0.0;
        double hoverboard_speed_ = 0.0;

        rclcpp::Time last_read_;
        rclcpp::Time last_read_enc_;
        double prev_l_rad_per_sec_ = 0.0;
        double prev_r_rad_per_sec_ = 0.0;
        bool first_read_enc_;
    };


}//end of paicxi namespace
#endif
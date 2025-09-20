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

        
        void update_encoders(const rclcpp::Duration & duration, int16_t r_rpm, int16_t l_rpm, std::vector<double> &state_positions );
        void forward_kinematics(const std::vector<double> &hw_commands);
        void inverse_kinematics();


        bool set_wheel_radius(const double & radius);
        bool set_max_velocity(const double & velocity);
        bool set_wheel_separation(const double & separation);

        inline double get_hover_steer() const{return hoverboard_steer_;}
        inline double get_hover_speed() const{return hoverboard_speed_;}
        inline double get_first_read()  const{return hoverboard_speed_;} 

    private:
           
        double wheel_radius_;
        double wheel_separation_;
        double max_velocity_;

        double wheel_omega_l_;
        double wheel_omega_r_; 
        double wheel_vel_l_; 
        double wheel_vel_r_;

        double hoverboard_steer_;
        double hoverboard_speed_;
        double prev_l_rad_per_sec_;
        double prev_r_rad_per_sec_;

        bool first_read_enc_;
    };


}//end of paicxi namespace
#endif
#include "paxi_hardware/protocol_worker.hpp"

namespace paxi_hardware
{
    ProtocolWorker::ProtocolWorker() 
    : 
        serial_port_{},
        protocol_{},
        encoder_{},
        imu_{},
        state_interface_positions_{},
        state_interface_velocities_{},
        feedback_buf_{},
        protocol_worker_thread_{},
        worker_running_{false},
        mutex_state_{},
        mutex_serial_{},
        paxi_interface_node_{std::make_unique<PaxiInterfaceNode>()},
        cached_clock_{paxi_interface_node_->get_clock()}
    {}

    void ProtocolWorker::init_zero_state_interfaces(const hardware_interface::HardwareInfo& hardware_info){

        state_interface_positions_.resize(hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());
        state_interface_velocities_.resize(hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(hardware_info.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (auto i = 0u; i < state_interface_positions_.size(); i++) {
            if (std::isnan(state_interface_positions_[i])) {
       
                state_interface_positions_[i] = 0.0;
            }
        }         
        for (auto i = 0u; i < state_interface_velocities_.size(); i++) {
            if (std::isnan(state_interface_velocities_[i])) {
                state_interface_velocities_[i] = 0.0;
            }
        }

        for (auto i = 0u; i < hw_commands_.size(); i++) {
            if (std::isnan(hw_commands_[i])) {
                hw_commands_[i] = 0.0;
            }
        }
    }

    bool ProtocolWorker::set_hardware_params_from_xacro(const hardware_interface::HardwareInfo hardware_info){
        bool validate_params = true;
        try {
            
            validate_params &= serial_port_.set_port(
                hardware_info.hardware_parameters.at("serial_port")
            );

            validate_params &= serial_port_.set_baud(
                std::stoul(hardware_info.hardware_parameters.at("baud_rate"))
            );

            validate_params &= encoder_.set_wheel_radius(
                std::stod(hardware_info.hardware_parameters.at("wheel_radius"))
            );

            validate_params &= encoder_.set_wheel_separation( 
                std::stod(hardware_info.hardware_parameters.at("wheel_separation"))
            );  

            validate_params &= encoder_.set_max_velocity(
                std::stod(hardware_info.hardware_parameters.at("max_velocity"))
            );

            validate_params &= imu_.set_imu_link_name(
                hardware_info.hardware_parameters.at("imu_link_name")
            );

            ;

      } catch (const std::out_of_range & e) {

          RCLCPP_ERROR(
              rclcpp::get_logger(LOGGER_HARDWARE),
              "Unable to parse parameters required from XACRO file:  %s", 
              e.what()
          );
        
          return false;
      }
      return validate_params;
    }


    void ProtocolWorker::start_worker(){

        worker_running_ = true;
        protocol_worker_thread_ = std::thread(&ProtocolWorker::worker_loop, this);
        RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_PROTOCOL_WORKER),
            "Starting thread for protocol worker!"
        );
    }

    void ProtocolWorker::stop_worker(){

        worker_running_ = false;
        if (protocol_worker_thread_.joinable()) {
            protocol_worker_thread_.join();
            RCLCPP_INFO(
                rclcpp::get_logger(LOGGER_PROTOCOL_WORKER), 
                "Stopped protocol worker thread, no longer processing feedback data!"
            );
        }
    }    

    void ProtocolWorker::worker_loop(){
        while (worker_running_) {
            ssize_t bytes_read = 0;
            get_new_feedback_buffer(bytes_read);

            if (bytes_read == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            if(bytes_read < 0){
                serial_port_.update_connection();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            protocol_parsing_loop(bytes_read);

        }
    }

    void ProtocolWorker::get_new_feedback_buffer(ssize_t& bytes_read){
        
        std::scoped_lock<std::mutex> lock(mutex_serial_);
        bytes_read = serial_port_.read_into_uint8_buf(
            feedback_buf_.data(), feedback_buf_.size()
        );
        
    }

    void ProtocolWorker::protocol_parsing_loop(const ssize_t& bytes_read){
        std::scoped_lock<std::mutex> lock(mutex_state_);
        for (auto i = 0u; i < static_cast<size_t>(bytes_read); ++i) {
              
            if (!protocol_.process_byte(feedback_buf_[i])) {
                continue;
            }

            update_paxi_interface_state();

            paxi_interface_node_->publish_real_time(
                protocol_.get_feedback(), 
                serial_port_.is_connected(), 
                imu_.get_imu_msg(), 
                state_interface_positions_
            );
        }
    }

    void ProtocolWorker::update_paxi_interface_state(){
        // lock should still be active from protocol parsing
        const SerialFeedback& feedback = protocol_.get_feedback();
        rclcpp::Time current_time = cached_clock_->now();

        state_interface_velocities_.at(to_index(Wheel::LEFT)) = feedback.speed_l_meas * RPM_TO_RAD_S;
        state_interface_velocities_.at(to_index(Wheel::RIGHT)) = feedback.speed_r_meas * RPM_TO_RAD_S;

        encoder_.update_encoders(
            current_time,
            feedback.speed_r_meas,
            feedback.speed_l_meas,
            state_interface_positions_
        );
    
        imu_.update_imu(current_time, feedback);
    }


    void ProtocolWorker::write_command(){
        SerialCommand hover_cmd = update_encoder();
        write_hover_commmand(hover_cmd);
    }

    inline SerialCommand ProtocolWorker::update_encoder(){
        std::scoped_lock<std::mutex> lock(mutex_state_);
        encoder_.forward_kinematics(hw_commands_);

        return protocol_.to_serial_command(
          static_cast<int16_t>(encoder_.get_hover_steer() * STEER_SCALE),
          static_cast<int16_t>(encoder_.get_hover_speed() * SPEED_SCALE)
        );
    }

    void ProtocolWorker::write_hover_commmand(const SerialCommand& hover_cmd){
              
        if (serial_port_.write_port(hover_cmd) < 0) {
            RCLCPP_WARN(
                rclcpp::get_logger(LOGGER_HARDWARE),
                "Protocol failed to send feedback command to port [%s], with steer [%d] and speed [%d]",
                serial_port_.get_port_name().c_str(), 
                hover_cmd.steer, 
                hover_cmd.speed
            );
        }
    }    
}  //end of namespace paxi_hardware

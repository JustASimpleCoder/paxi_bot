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


    ProtocolWorker::ProtocolWorker(SerialPort* serial_port, 
                        HoverboardProtocol* protocol, 
                        EncoderKinematics* encoder,
                        ImuProcessing* imu,
                        std::vector<double>* state_positions,
                        std::vector<double>* state_velocities ) 
    : 
        serial_port_{serial_port},
        protocol_{protocol},
        encoder_{encoder},
        imu_{imu},
        state_interface_positions_{state_positions},
        state_interface_velocities_{state_velocities},
        feedback_buf_{},
        protocol_worker_thread_{},
        worker_running_{false},
        mutex_state_{},
        mutex_serial_{},
        paxi_interface_node_{std::make_unique<PaxiInterfaceNode>()},
        cached_clock_{paxi_interface_node_->get_clock()}
    {}

    // assigment automaticcaly turns worker 'off', 
    // cannot move mutex be careful -> expect to only copy once in paxi intergace on_init()! 
    ProtocolWorker& ProtocolWorker::operator=(ProtocolWorker && other) noexcept
    {
        if(this != &other){

            other.stop_worker();

            serial_port_                = std::move(other.serial_port_);
            protocol_                   = std::move(other.protocol_);
            encoder_                    = std::move(other.encoder_);
            imu_                        = std::move(other.imu_);

            state_interface_positions_  = std::move(other.state_interface_positions_);
            state_interface_velocities_ = std::move(other.state_interface_velocities_);
            feedback_buf_               = std::move(other.feedback_buf_);

            protocol_worker_thread_     = std::move(other.protocol_worker_thread_);
            worker_running_             = false;
            paxi_interface_node_        = std::move(other.paxi_interface_node_);
            cached_clock_               = std::move(other.cached_clock_);
        }

        return *this;
    }

    void ProtocolWorker::start_worker(){

        worker_running_ = true;
        protocol_worker_thread_ = std::thread(&ProtocolWorker::worker_loop, this);
        RCLCPP_INFO(
            rclcpp::get_logger(LOGGER_PROTOCOL_WORKER),
            "Satrting thread for protocol worker!"
        );
    }

    void ProtocolWorker::stop_worker(){

        worker_running_ = false;
        if (protocol_worker_thread_.joinable()) {
            protocol_worker_thread_.join();
            RCLCPP_INFO(
                rclcpp::get_logger(LOGGER_PROTOCOL_WORKER), 
                "Stopped protocol workerthread, no longer processing feedback data!"
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
                serial_port_->update_connection();
                continue;
            }

            protocol_parsing_loop(bytes_read);

        }
    }

    void ProtocolWorker::get_new_feedback_buffer(ssize_t& bytes_read){
        {
            std::scoped_lock<std::mutex> lock(mutex_serial_);
            bytes_read = serial_port_->read_into_uint8_buf(
                feedback_buf_.data(), feedback_buf_.size()
            );
        }
    }

    void ProtocolWorker::protocol_parsing_loop(const ssize_t& bytes_read){
  
        for (auto i = 0u; i < static_cast<size_t>(bytes_read); ++i) {
            {
                std::scoped_lock<std::mutex> lock(mutex_state_);
                if (!protocol_->process_byte(feedback_buf_[i])) {
                    continue;
                }

                update_paxi_interface_state();
            }

            paxi_interface_node_->publish_real_time(
                protocol_->get_feedback(), 
                serial_port_->is_connected(), 
                imu_->get_imu_msg(), 
                *state_interface_positions_
            );
        }
    }

    void ProtocolWorker::update_paxi_interface_state(){
        // lock should still be active from protocol parsing
        const SerialFeedback& feedback = protocol_->get_feedback();
        rclcpp::Time current_time = cached_clock_->now();

        state_interface_velocities_->at(to_index(Wheel::LEFT)) = feedback.speed_l_meas * RPM_TO_RAD_S;
        state_interface_velocities_->at(to_index(Wheel::RIGHT)) = feedback.speed_r_meas * RPM_TO_RAD_S;

        encoder_->update_encoders(
            current_time,
            feedback.speed_r_meas,
            feedback.speed_l_meas,
            *state_interface_positions_
        );
    
        imu_->update_imu(current_time, feedback);
    }

}  //end of namespace paxi_hardware

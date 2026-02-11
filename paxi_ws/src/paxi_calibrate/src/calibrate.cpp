#ifndef PAXI_CALIBRATE__CALIBRATE_CPP_
#define PAXI_CALIBRATE__CALIBRATE_CPP_

#include <vector>
#include <utility>

#include "paxi_calibrate/calibrate_subscriber.hpp"
#include "paxi_calibrate/calibrate_calculations.hpp"
#include "paxi_calibrate/calibrate_twist_pub.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  CalibrateCalculations cal_calc;

  std::shared_ptr<CalibrateSubscriber> cal_sub = std::make_shared<CalibrateSubscriber>();
  std::shared_ptr<TwistPub> cal_pub = std::make_shared<TwistPub>();
  
  rclcpp::executors::MultiThreadedExecutor cal_executor;
  cal_executor.add_node(cal_sub);
  cal_executor.add_node(cal_pub);
  cal_executor.spin();
  std::vector<std::pair<double, double>> linear_angular_tests{
    {0.1, 0.0},
    {0.2, 0.0},
    {0.3, 0.0},
    {0.4, 0.0},
    {0.5, 0.0},

    {-0.1, 0.0},
    {-0.2, 0.0},
    {-0.3, 0.0},
    {0.4, 0.0},
    {0.5, 0.0},

    {0.0, 0.1},
    {0.0, 0.2},
    {0.0, 0.3},
    {0.0, 0.4},
    {0.0, 0.5},
    {0.0, 0.6},
    {0.0, 0.7},
    {0.0, 0.8},
    {0.0, 0.9},
    {0.0, 1.0},
  };

  for(std::size_t i = 0u; i < linear_angular_tests.size(); ++i){
    cal_pub->set_linear_and_angular(
      linear_angular_tests[0].first, 
      linear_angular_tests[0].second
    );
    
    cal_pub->publish_twist();
    while(!cal_sub->get_has_max_sample()){
      // wait until sub has gotten its samples
    }
    cal_calc.calculate_l(cal_sub->get_l_cmd_samples(), cal_sub->get_l_feedback_samples());
    cal_calc.calculate_r(cal_sub->get_r_cmd_samples(), cal_sub->get_r_feedback_samples());
    // TO_DO outpit to csv linear,angular,differece,tf_ratio, ft_ratio
  }

  rclcpp::shutdown();
  return 0;
}


#endif  // PAXI_CALIBRATE__CALIBRATE_CPP_

#include "paxi_calibrate/calibrate_test.hpp"

CalibrateTest::CalibrateTest()
: Node("Calibrate_Test"),
  cal_sub{std::make_shared<CalibrateSubscriber>()},
  cal_pub{std::make_shared<TwistPub>()},
  cal_calc{},
  csv{LEFT_FILENAME, RIGHT_FILENAME},
  test_timer_{}
{
  test_timer_ = create_wall_timer(100ms, std::bind(&CalibrateTest::run_test_callback, this));
}

void CalibrateTest::run_test_callback()
{

  std::vector<std::pair<double, double>> linear_angular_tests{
    {0.1, 0.0},
    {0.2, 0.0},
    {0.3, 0.0},
    {0.4, 0.0},
    {0.5, 0.0},

    {-0.1, 0.0},
    {-0.2, 0.0},
    {-0.3, 0.0},
    {-0.4, 0.0},
    {-0.5, 0.0},

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

  RCLCPP_INFO(
    rclcpp::get_logger(LOGGER_MAIN),
    "Starting Calulcations, please ensure robot is not able to move, press [n] to stop this node"
  );

  std::string stop;
  std::cin >> stop;
  if (stop.size() > 0) {
    if (stop == "n" || stop == "N") {
      rclcpp::shutdown();
    }
  }

  for (std::size_t i = 0u; i < linear_angular_tests.size(); ++i) {

    const double & linear = linear_angular_tests[i].first;
    const double & angular = linear_angular_tests[i].second;

    RCLCPP_INFO(
      rclcpp::get_logger(LOGGER_MAIN),
      "Starting test [%lu], with linear speed [%lf] and angular speed [%lf]",
      i,
      linear,
      angular
    );

    cal_pub->set_linear_and_angular(linear, angular);

    // set new publisher commands, wait a bit for robot to get to speed then wait until collection are sampled
    // let publisher do its thing at new speed for half a second
    rclcpp::sleep_for(500ms);

    cal_sub->reset_samples();
    while (!cal_sub->get_has_max_sample()) {
      // rclcpp::spin_some(cal_sub);
      rclcpp::sleep_for(10ms);
    }
    cal_calc.calculate_l(cal_sub->get_l_target_samples(), cal_sub->get_l_feedback_samples());
    cal_calc.calculate_r(cal_sub->get_r_target_samples(), cal_sub->get_r_feedback_samples());

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_MAIN), "received and calculated new sample");

    csv.add_line_l(
      linear, angular, 1.0, 1.0,
      cal_calc.get_l_diffference(), cal_calc.get_l_tf(), cal_calc.get_l_ft());

    csv.add_line_r(
      linear, angular, 1.0, 1.0,
      cal_calc.get_r_diffference(), cal_calc.get_r_tf(), cal_calc.get_r_ft());

    cal_calc.reset_constants();
  }


  csv.close_files();
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_MAIN), "Finished test, output data in csv files");

  rclcpp::shutdown();
}

#include "paxi_calibrate/calibrate_test.hpp"

CalibrateTest::CalibrateTest()
: Node("Calibrate_Test"),
  cal_sub{std::make_shared<CalibrateSubscriber>()},
  cal_pub{std::make_shared<TwistPub>()},
  cal_calc{},
  csv{LEFT_FILENAME, RIGHT_FILENAME},
  test_timer_{},
  linear_angular_tests_{}
{
  test_timer_ = create_wall_timer(100ms, std::bind(&CalibrateTest::run_test_callback, this));
  generate_tests();
}

void CalibrateTest::generate_tests()
{
  // pos
  add_linear(1.0);
  add_pause();
  add_angular(1.0);
  // neg
  add_pause();
  add_angular(-1.0);
  add_pause();
  add_linear(-1.0);
}

void CalibrateTest::add_linear(double sign)
{
  for (int j = START_RANGE; j < LINEAR_TEST_END_RANGE; ++j) {
    for (int i = 0; i < 10; ++i) {
      const double value = sign * ((j / GRANULARITY) + (STEP_COUNT * i));
      linear_angular_tests_.emplace_back(value, 0.0);
    }
  }
}

void CalibrateTest::add_angular(double sign)
{
  for (int j = START_RANGE; j < ANGULAR_TEST_END_RANGE; ++j) {
    for (int i = 0; i < 10; ++i) {
      const double value = sign * ( (j / GRANULARITY) + STEP_COUNT * i);
      linear_angular_tests_.emplace_back(0.0, value);
    }
  }
  // just add 1.0 or -1.0 as a test cast
  linear_angular_tests_.emplace_back(0.0, sign);
}

void CalibrateTest::add_pause()
{
  for (int i = 0; i < PAUSE_COUNT; ++i) {
    linear_angular_tests_.emplace_back(0.0, 0.0);
  }
}

void CalibrateTest::run_test_callback()
{


  for (std::size_t i = 0u; i < linear_angular_tests_.size(); ++i) {

    const double & linear = linear_angular_tests_[i].first;
    const double & angular = linear_angular_tests_[i].second;

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
    rclcpp::sleep_for(250ms);
    cal_sub->reset_samples();

    while (!cal_sub->get_has_max_sample()) {
      rclcpp::sleep_for(10ms);
    }

    const std::vector<double> & l_target_samples = cal_sub->get_l_target_samples();
    const std::vector<double> & r_target_samples = cal_sub->get_r_target_samples();
    const std::vector<double> & l_feedback_samples = cal_sub->get_l_feedback_samples();
    const std::vector<double> & r_feedback_samples = cal_sub->get_r_feedback_samples();

    cal_calc.calculate_l(l_target_samples, l_feedback_samples);
    cal_calc.calculate_r(r_target_samples, r_feedback_samples);

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_MAIN), "received and calculated new sample");

    csv.add_line_l(
      linear,
      angular,
      l_target_samples,
      l_feedback_samples,
      cal_calc.get_l_diffference(),
      cal_calc.get_l_tf(),
      cal_calc.get_l_ft()
    );

    csv.add_line_r(
      linear,
      angular,
      r_target_samples,
      r_feedback_samples,
      cal_calc.get_r_diffference(),
      cal_calc.get_r_tf(),
      cal_calc.get_r_ft()
    );

    cal_calc.reset_constants();
  }

  csv.close_files();
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_MAIN), "Finished test, output data in csv files");

  rclcpp::shutdown();
}

// // Copyright 2026 JustASimpleCoder
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

// #include "paxi_hardware/paxi_interface_node.hpp"

// #include <gtest/gtest.h>
// #include <gtest/gtest-death-test.h>

// #include "paxi_common/utils.hpp"

// namespace paxi_hardware
// {

// using paxi_common::utils::to_index;
// using paxi_common::utils::Wheel;
// using paxi_hardware::SerialFeedback;

// namespace topics = paxi_common::hardware_topics;

// class PaxiInterfaceNodeTest : public ::testing::Test
// {
// protected:
//   void SetUp() override {
//     paxi_node_ = std::make_unique<paxi_hardware::PaxiInterfaceNode>();
//   }

//   void TearDown() override {

//   };

//   std::unique_ptr<paxi_hardware::PaxiInterfaceNode> paxi_node_;
//   rclcpp::executors::MultiThreadedExecutor executor_;
// };


// class PaxiInterfaceNodeSubTest : public ::testing::Test
// {



// }

// struct TestRealTimeParams{
//   SerialFeedback feedback;
//   bool connected;
//   std::vector<double> state_positions;
// };

// class TestNullPubsAndSubs : public PaxiInterfaceNodeTest,
//   public ::testing::WithParamInterface<TestRealTimeParams>
// {

// };



// TEST_P(TestNullPubsAndSubs, PublishRealTime)
// {
//   TestRealTimeParams params = GetParam();
//   paxi_node_->publish_real_time(params.feedback, params.connected, params.state_positions); 
// }

// TEST_P(TestNullPubsAndSubs, PublishFeedback)
// {
//   TestRealTimeParams params = GetParam();
//   paxi_node_->publish_feedback(params.feedback); 
// }

// TEST_P(TestNullPubsAndSubs, PublishCmdToHover)
// {
//   TestRealTimeParams params = GetParam();

//   SerialCommand cmd;
//   paxi_node_->publish_cmd_to_hover(cmd); 
// }

// TEST_P(TestNullPubsAndSubs, PublishControllerCmd)
// {
//   TestRealTimeParams params = GetParam();

//   SerialCommand cmd;
//   paxi_node_->publish_controller_cmd(0.0, 0.0); 
// }

// TestRealTimeParams test_1 = {
//   SerialFeedback{},
//   false,
//   {1,2}
// };

// INSTANTIATE_TEST_SUITE_P(
//   PublishRealTime, 
//   TestNullPubsAndSubs, 
//   ::testing::Values(
//     TestRealTimeParams(),//SerialFeedback{1,2,3,4}, false, {12,13})
//     test_1
//   )
// );

// }  // namespace paxi_hardware

// int main(int argc, char ** argv)
// {
//   testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }

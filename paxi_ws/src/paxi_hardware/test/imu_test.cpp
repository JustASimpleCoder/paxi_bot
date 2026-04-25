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

// #include "paxi_hardware/imu.hpp"

// #include <gtest/gtest.h>

// #include "paxi_common/utils.hpp"

// namespace paxi_hardware
// {

// using paxi_common::utils::to_index;
// using paxi_common::utils::Wheel;
// class EncoderKinematicsTest : public ::testing::Test
// {
// protected:
//   void SetUp() override {encoder_kin = std::make_unique<paxi_hardware::ImuProcessing>();}

//   std::unique_ptr<paxi_hardware::ImuProcessing> imu_;
// };

// class EncoderKinematicsConstRPMTest : public EncoderKinematicsTest,
//   public ::testing::WithParamInterface<int>
// {
// };

// TEST_P(EncoderKinematicsConstRPMTest, UpdateEncoderConstRPM)
// {
  
// }

// INSTANTIATE_TEST_SUITE_P(
//   ForwardBackwardConstRPM, EncoderKinematicsConstRPMTest, ::testing::Range(INT16_MIN, INT16_MAX));





// }  // namespace paxi_hardware

// int main(int argc, char ** argv)
// {
//   testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }

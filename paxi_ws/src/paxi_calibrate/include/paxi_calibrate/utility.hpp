// Copyright 2026 JustASimpleCoder
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PAXI_CALIBRATE__UTILITY_HPP_
#define PAXI_CALIBRATE__UTILITY_HPP_

#include <cstdint>

// sample size of data collection
inline constexpr std::size_t SAMPLE_SIZE_RPM = 100;

// Calibration test can be done automatically with generated tests with twist publisher,
// or manual control
inline constexpr bool AUTOMATIC = true;

// paramter names
inline constexpr const char * GENERATE_TEST = "generate_test";

#endif  // PAXI_CALIBRATE__UTILITY_HPP_

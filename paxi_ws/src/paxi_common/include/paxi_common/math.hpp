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

#ifndef PAXI_COMMON__MATH_HPP_
#define PAXI_COMMON__MATH_HPP_

namespace paxi_common::math
{
// Constant for famous PI 3 constant
inline constexpr double PI = 3.14159265358979323846;

// converts RPM to rad/s
inline constexpr double RPM_TO_RAD_S = PI / 30.0;

// converts rad/s to rpm
inline constexpr double RAD_S_TO_RPM = 30.0 / PI;

// Degree to rad unit conversion
inline constexpr double DEG_TO_RAD = PI / 180.0;

// RAD to Degree unit conversion
inline constexpr double RAD_TO_DEG = 180.0 / PI;

}  // namespace paxi_common::math

#endif  // PAXI_COMMON__MATH_HPP_
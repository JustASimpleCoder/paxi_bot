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

#ifndef PAXI_HARDWARE__UTILITY_HPP_
#define PAXI_HARDWARE__UTILITY_HPP_

#include <array>
#include <cstdint>
#include "paxi_hardware/hoverboard_protocol_struct.hpp"

namespace paxi_hardware
{
// Debug sensor data, turn on to allow publishing of hardware information
inline constexpr bool DEBUG_SENSORS = true;

// Used in conjuction with calibrate package. Sets hardware to publish messages
// in order to calibrate RPM speeds
inline constexpr bool CALIBRATE_FIRMWARE = false;

// Enum to store wheel index and total wheel count
//      LEFT = 0,
//      RIGHT = 1,
//      COUNT = 2
enum class Wheel : std::size_t
{
  LEFT = 0,
  RIGHT = 1,
  COUNT = 2
};

// Helper function to conver WheelPostion enum to appropriate index
constexpr std::size_t to_index(Wheel pos) noexcept {return static_cast<std::size_t>(pos);}

// Used in templates to ensure arrays have at least two indices
// and useful for arrays storing wheel data
inline constexpr std::size_t WHEEL_COUNT = static_cast<std::size_t>(Wheel::COUNT);
}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__UTILITY_HPP_

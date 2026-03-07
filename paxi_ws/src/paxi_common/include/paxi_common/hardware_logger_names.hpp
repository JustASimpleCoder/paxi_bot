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

#ifndef PAXI_COMMON__HARDWARE_LOGGER_NAMES_HPP_
#define PAXI_COMMON__HARDWARE_LOGGER_NAMES_HPP_

namespace paxi_common::hardware_loggers
{

// Logger name for encoder
inline constexpr const char * LOGGER_ENCODER = "paxi_hardware_encoder";

// Logger name for encoder
inline constexpr const char * LOGGER_PROTOCOL = "paxi_hardware_protocol";

// Logger name for imu
inline constexpr const char * LOGGER_IMU = "paxi_hardware_imu";

// Logger for general paxi_hardware
inline constexpr const char * LOGGER_HARDWARE = "paxi_hardware";

// Logger name for paxi hardware worker
inline constexpr const char * LOGGER_PROTOCOL_WORKER = "paxi_hardware_protocol_worker";

// Logger name for serial port and protocol
inline constexpr const char * LOGGER_SERIAL = "paxi_hardware_serial";

}  // namespace paxi_common::hardware_loggers

#endif  // PAXI_COMMON__HARDWARE_LOGGER_NAMES_HPP_

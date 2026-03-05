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

}  // namespace paxi_hardware

#endif  // PAXI_COMMON__HARDWARE_TOPIC_NAMES_HPP_
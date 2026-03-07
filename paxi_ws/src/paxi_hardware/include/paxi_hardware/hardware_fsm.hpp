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


// TODO(jacob): Decide if this is overly complex, and we can add back in paxi_interface.cpp
// paxi_interface is already implemented as a FSM with callback returns

// Follows examples from https://www.cppstories.com/2023/finite-state-machines-variant-cpp/
// ANd from https://www.cppstories.com/2019/06/fsm-variant-game/

#ifndef PAXI_HARDWARE__HARDWARE_WORKER_HPP_
#define PAXI_HARDWARE__HARDWARE_WORKER_HPP_

#include <cstdint>
#include <variant>

#include "paxi_common/hardware_logger_names.hpp"

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/hardware_info.hpp"

namespace paxi_hardware
{

namespace state{

  struct Initializing{
    bool xacro_data_error{false};
    bool params_parse_error{false};
    bool states_interface_error{false};
  };

  struct Configuring{};

  struct Activating{
    bool state_interface_activated{true};
    bool serial_port_open{false};
    bool worker_started{false};
  };

  struct Running{
    bool worker_running{false};
  };

  struct Error{
    std::string reason{"unkown"};
  };

  struct Disconnected{
    std::size_t consecutive_error{0};
    rclcpp::Time first_error;
  };

  struct ReadTimeout{
    std::size_t consecutive_timeouts{0};
    rclcpp::Time first_timeout;
  };
 
  struct Deactivating{
    bool worker_stopped{false};
    bool serial_port_close{false};
  };

  struct CleaningUp{};
  //struct ShutingDown{};

  struct Deactivated{};
  struct Shutdown{};
  
}; // namespace state

namespace event{

  struct XacroFail{
    bool bad_read{false};
  };

  struct ParsingXacroFail{};
  struct InitFailure{std::string reason;};
  struct InitComplete{};

  struct ConfigFailure{std::string reason;};
  struct ConfigComplete{};

  struct ActivationFailed{std::string reason;};
  struct ActivationComplete{};

  struct ReadOk{};
  struct ReadNoData{rclcpp::Time now;};
  struct ReadError{rclcpp::Time now;};

  struct WriteError{rclcpp::Time now;};
  struct WriteOk{};

  struct UsbDisconnect{};
  struct RetryConnect{};

  struct FatalError{std::string reason;};

  struct DeactivatedRequest{};
  struct DeactivateComplete{};

  struct ShutdownRequested{};

  struct CleanupComplete{};

}; // namespace event

using HardwareStates = 
  std::variant<
    state::Configuring, state::Initializing, state::Activating, state::Running, state::Error,
    state::Disconnected, state::ReadTimeout, state::Deactivating, state::CleaningUp, 
    state::Deactivated, state::Shutdown>;

using HardwareEvents = 
  std::variant<event::XacroFail, event::ParsingXacroFail, event::InitFailure, event::InitComplete,
  event::ConfigFailure, event::ConfigComplete, event::ReadOk, event::ReadNoData, event::ReadError, event::WriteError,
  event::WriteOk, event::UsbDisconnect, event::RetryConnect, event::FatalError, event::DeactivatedRequest,
  event::DeactivateComplete, event::CleanupComplete, event::ShutdownRequested>;


// 'Overload helper template' helps to  bring all call cpature into a scope with a list of lambdas
template<typename ... Ts>
struct overload : Ts... { using Ts::operator()...; };
template<typename... Ts>
overload(Ts...) -> overload<Ts...>;

class HardwareFSM
{
public:
  HardwareFSM();
  ~HardwareFSM() = default;

  HardwareFSM(const HardwareFSM &) = delete;
  HardwareFSM & operator=(const HardwareFSM &) = delete;

  HardwareFSM(HardwareFSM &&) noexcept = delete;
  HardwareFSM & operator=(HardwareFSM &&) noexcept = delete;

  bool process (const HardwareEvents & event){

  HardwareStates next = std::visit(
    overload{
        // Initialized states
        [](const state::Initializing &, const event::InitComplete &) -> HardwareStates{
          return state::Configuring{};
        },
        [](const state::Initializing &, const event::FatalError & e) -> HardwareStates{
          return state::Error{e.reason};
        },

        // Configuring
        [](const state::Configuring &, const event::ConfigComplete &) -> HardwareStates{
          return state::Activating{};
        },
        [](const state::Configuring &, const event::FatalError & e) -> HardwareStates{
          return state::Error{e.reason};
        },

        // Activating
        [](const state::Activating &, const event::ActivationComplete &) -> HardwareStates{
          return state::Running{};
        },
        [](const state::Activating &, const event::ActivationFailed & e) -> HardwareStates{
          return state::Error{e.reason};
        },
        [](const state::Activating &, const event::FatalError & e) -> HardwareStates{
          return state::Error{e.reason};
        },

        //Running
        [](const state::Running &, const event::ReadNoData& e) -> HardwareStates{
          return state::ReadTimeout{1,e.now};
        },
        [](const state::Running &, const event::ReadError & e) -> HardwareStates{
          return state::Disconnected{1, e.now};
        },
        [](const state::Running &, const event::ReadOk &) -> HardwareStates{
          return state::Running{};
        },
        [](const state::Running &, const event::WriteError & e) -> HardwareStates{
          return state::Error{"write_failure"};
        },
        [](const state::Running &, const event::WriteOk) -> HardwareStates{
          return state::Running{};
        },
        [](const state::Running &, const event::DeactivatedRequest &) -> HardwareStates{
          return state::Deactivating{};
        },

        //Deactivating
        [](const state::Deactivating, const event::DeactivateComplete &) -> HardwareStates{
          return state::Deactivated{};
        },
        [](const state::Deactivating &, const event::FatalError & e) -> HardwareStates{
          return state::Error{e.reason};
        },

        // clean up 
        [](const state::CleaningUp &, const event::CleanupComplete &) -> HardwareStates{
          return state::Shutdown{};
        },
        [](const state::Running &, const event::ShutdownRequested & e) -> HardwareStates{
          return state::CleaningUp{};
        },

        [](const auto & s, const auto &) -> HardwareStates { return s; }
      }, 
      state_, event
    );

    const bool state_change = next.index() != state_.index();

    if (state_change){
      log_transition(next);
    }

    state_ = std::move(next);
    return state_change;
  }

private:

  HardwareStates state_;

  void log_transition(const HardwareStates & next_state) const 
  {
    static constexpr const char * LOGGER = "paxi_hardware_fsm";
    std::visit(overload{
      [](const state::Running &){ 
        RCLCPP_INFO(rclcpp::get_logger(LOGGER), "Current state [Running]");
      },

      [](const state::Disconnected &){ 
        RCLCPP_WARN(rclcpp::get_logger(LOGGER), "Current state Disconnected]");
      },

      [](const state::ReadTimeout &) { 
        RCLCPP_WARN(rclcpp::get_logger(LOGGER), "Current state ReadTimeout]");
      },

      [](const state::Error & e)     { 
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "Current state [Error] reason [%s]", e.reason.c_str());
      },

      [](const state::Deactivating &){ 
        RCLCPP_INFO(rclcpp::get_logger(LOGGER), "Current state [Deactivating]");
      },

      [](const auto &)               { 
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER), "Current state [Unwritten State change]");
      }
    }, next_state);
  }
};

}  // namespace paxi_hardware

#endif  // PAXI_HARDWARE__HARDWARE_WORKER_HPP_

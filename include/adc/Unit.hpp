// Copyright 2025 Pavel Suprunov
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

#pragma once

#include <adc/Channel.hpp>
#include <cstdint>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_continuous.h>
#include <expected>
#include <memory>
#include <vector>

namespace adc {

class Unit {
public:
  enum class Error : std::uint8_t {
    UNIT_CREATE_ERROR,
    CALIBRATION_INIT_ERROR,
    DEVICE_MAX_SIZE,
    DEVICE_CONFIGURATION_ERROR,
    UNIT_START_FAILED,
  };

public:
  using Number = uint8_t;
  using Pointer = std::unique_ptr<Unit>;

private:
  using Configuration = adc_digi_pattern_config_t;
  using Configurations = std::vector<Configuration>;

public:
  static auto create(Number unitNumber) -> std::expected<Pointer, Error>;

private:
  Unit(Number unitNumber, Channel::ContinuousHandle continuousHandle, Channel::CalibrationHandle calibrationHandle) noexcept;

public:
  ~Unit() noexcept;

public:
  [[nodiscard]] [[maybe_unused]] auto createChannel(Channel::Number channelNumber) noexcept -> std::expected<Channel::Pointer, Error>;

private:
  [[nodiscard]] auto reconfigure() -> bool;

private:
  Number const m_unitNumber;
  Channel::ContinuousHandle const m_continuousHandle;
  Channel::CalibrationHandle const m_calibrationHandle;

private:
  Configurations m_configurations{};
};

} // namespace adc

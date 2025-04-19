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
#include <esp_adc/adc_continuous.h>
#include <expected>
#include <memory>
#include <vector>

namespace adc {

class Unit {
public:
  using UnitNumber = uint8_t;
  using DriverHandle = adc_continuous_handle_t;
  using CalibrationHandle = adc_cali_handle_t;
  using ChannelConfiguration = adc_digi_pattern_config_t;

private:
  using ChannelConfigurations = std::vector<ChannelConfiguration>;

public:
  Unit(UnitNumber unitNumber);

public:
  [[nodiscard]] [[maybe_unused]] auto createChannel(Channel::ChannelNumber channelNumber) -> Channel;

private:
  auto reconfigure() -> bool;

private:
  UnitNumber const unitNumber;

private:
  DriverHandle driverHandle = nullptr;
  CalibrationHandle calibrationHandle = nullptr;

private:
  ChannelConfigurations channelConfigurations = {};
};

} // namespace adc

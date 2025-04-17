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
#include <vector>

namespace adc {

class ADC {
public:
  using Unit = std::uint8_t;
  using Handle = adc_continuous_handle_t;
  using ChannelNumber = Channel::Number;
  using Configurations = std::vector<adc_digi_pattern_config_t>;
  using CalibrationHandle = adc_cali_handle_t;

public:
  explicit ADC(ADC::Unit unit);
  ~ADC();

public:
  ChannelPtr createChannel(ADC::ChannelNumber channel);

private:
  void reconfigure();

private:
  ADC::Unit const m_unit;

private:
  ADC::Handle m_handle = nullptr;
  ADC::CalibrationHandle m_calibrationHandle = nullptr;

private:
  ADC::Configurations m_configurations = {};
};

} // namespace adc

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

#include <cstdint>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_continuous.h>

namespace adc {

auto const ADC_FRAME_SIZE = 256 * SOC_ADC_DIGI_RESULT_BYTES;

class Channel {
public:
  using Value = std::uint16_t;
  using Handle = adc_continuous_handle_t;
  using Number = std::uint8_t;
  using Voltage = std::uint16_t;
  using CalibrationHandle = adc_cali_handle_t;

public:
  Channel(Channel::Number channel, Channel::Handle &handle, Channel::CalibrationHandle &calibrationHandle);

public:
  [[nodiscard]] Channel::Value getRawValue() const;
  [[nodiscard]] Channel::Voltage getVoltage() const;

private:
  Channel::Number const m_channel;

private:
  Channel::Handle &m_handle;
  Channel::CalibrationHandle &m_calibrationHandle;
};

}// namespace adc

#include <memory>

using ChannelPtr = std::unique_ptr<adc::Channel>;

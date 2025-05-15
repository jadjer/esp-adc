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
#include <expected>
#include <memory>

namespace adc {

class Channel {
public:
  friend class Unit;

public:
  using Value = std::uint16_t;
  using Number = std::uint8_t;
  using Pointer = std::unique_ptr<Channel>;
  using Voltage = std::uint16_t;
  using FrameSize = std::uint32_t;
  using ContinuousHandle = adc_continuous_handle_t;
  using CalibrationHandle = adc_cali_handle_t;

private:
  using ContinuousEventData = adc_continuous_evt_data_t;

private:
  Channel(Number channelNumber, ContinuousHandle continuousHandle, CalibrationHandle calibrationHandle) noexcept;

public:
  [[nodiscard]] [[maybe_unused]] auto getRawValue() const -> Value;
  [[nodiscard]] [[maybe_unused]] auto getVoltage() const -> Voltage;
  [[nodiscard]] [[maybe_unused]] auto getFrameSize() const -> FrameSize;

private:
  Number const m_channelNumber;
  ContinuousHandle const m_continuousHandle;
  CalibrationHandle const m_calibrationHandle;
};

} // namespace adc

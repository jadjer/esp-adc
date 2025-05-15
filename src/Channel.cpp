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

#include "adc/Channel.hpp"

#include <array>
#include <cstdint>
#include <esp_err.h>
#include <esp_log.h>

#include "sdkconfig.h"

namespace adc {

namespace {

auto const TAG = "ADC Channel";
auto const ADC_FRAME_SIZE = 10 * SOC_ADC_DIGI_RESULT_BYTES;

using Byte = std::uint8_t;
using Frame = std::array<Byte, ADC_FRAME_SIZE>;
using Result = esp_err_t;
using Callbacks = adc_continuous_evt_cbs_t;

} // namespace

Channel::Channel(Channel::Number const channelNumber_, Channel::ContinuousHandle continuousHandle_, Channel::CalibrationHandle calibrationHandle_) noexcept
    : m_channelNumber(channelNumber_), m_continuousHandle(continuousHandle_), m_calibrationHandle(calibrationHandle_) {
  assert(m_continuousHandle != nullptr);
  assert(m_calibrationHandle != nullptr);
}

auto Channel::getRawValue() const -> Channel::Value {
  Frame frame{};

  std::uint32_t numberOfValuesInFrame = 0;

  Result const returnCode = adc_continuous_read(m_continuousHandle, frame.data(), ADC_FRAME_SIZE, &numberOfValuesInFrame, 0);
  if (returnCode != ESP_OK) {
    ESP_LOGE(TAG, "Read value error: %s", esp_err_to_name(returnCode));
    return 0;
  }

  std::uint32_t valueCount = 0;
  std::uint32_t sumOfRawDataPerFrame = 0;

  for (std::uint32_t i = 0; i < numberOfValuesInFrame; i += SOC_ADC_DIGI_RESULT_BYTES) {
    auto *const outputData = reinterpret_cast<adc_digi_output_data_t *>(&frame.at(i));

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
    std::uint8_t const currentChannelNumber = outputData->type1.channel;
    if (currentChannelNumber == m_channelNumber) {
      std::uint32_t const data = outputData->type1.data;
      valueCount += 1;
      sumOfRawDataPerFrame += data;
    }

#else
    std::uint8_t const currentChannelNumber = outputData->type2.channel;
    if (currentChannelNumber == channelNumber) {
      std::uint32_t const data = outputData->type2.data;
      valueCount += 1;
      sumOfRawDataPerFrame += data;
    }
#endif
  }

  if (valueCount == 0) {
    return 0;
  }

  Channel::Value const rawValue = sumOfRawDataPerFrame / valueCount;

  return rawValue;
}

auto Channel::getVoltage() const -> Channel::Voltage {
  auto const rawValue = getRawValue();

  int voltage = 0;

  Result const returnCode = adc_cali_raw_to_voltage(m_calibrationHandle, rawValue, &voltage);
  if (returnCode != ESP_OK) {
    ESP_LOGE(TAG, "Convert raw value to voltage error: %s", esp_err_to_name(returnCode));
  }

  return voltage;
}

auto Channel::getFrameSize() const -> Channel::FrameSize { return ADC_FRAME_SIZE; }

} // namespace adc

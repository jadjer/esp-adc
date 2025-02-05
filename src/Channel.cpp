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

#include <esp_log.h>

namespace adc {

auto const TAG = "ADC Channel";

Channel::Channel(Channel::Number const channel, Channel::Handle &handle, Channel::CalibrationHandle &calibrationHandle) : m_number(channel),
                                                                                                                          m_handle(handle),
                                                                                                                          m_calibrationHandle(calibrationHandle) {
}

Channel::Value Channel::getRawValue() const {
  std::uint8_t frame[ADC_FRAME_SIZE] = {};
  std::uint32_t numberOfValuesInFrame = 0;

  esp_err_t returnCode = adc_continuous_read(m_handle, frame, ADC_FRAME_SIZE, &numberOfValuesInFrame, 0);
  if (returnCode != ESP_OK) {
    return 0;
  }

  std::size_t valueCount = 0;
  std::uint32_t sumOfRawDataPerFrame = 0;

  for (auto i = 0; i < numberOfValuesInFrame; i += SOC_ADC_DIGI_RESULT_BYTES) {
    auto const outputData = reinterpret_cast<adc_digi_output_data_t *>(&frame[i]);

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
    std::uint32_t const channelNumber = outputData->type1.channel;
    if (channelNumber == m_number) {
      std::uint32_t const data = outputData->type1.data;
      valueCount += 1;
      sumOfRawDataPerFrame += data;
    }

#else
    std::uint32_t const channelNumber = outputData->type2.channel;
    if (channelNumber == m_number) {
      std::uint32_t const data = outputData->type2.data;
      valueCount += 1;
      sumOfRawDataPerFrame += data;
    }
#endif
  }

  if (valueCount == 0) {
    return 0;
  }

  auto const rawValue = sumOfRawDataPerFrame / valueCount;

  return rawValue;
}

Channel::Voltage Channel::getVoltage() const {
  auto const rawValue = getRawValue();

  int voltage = 0;

  ESP_ERROR_CHECK(adc_cali_raw_to_voltage(m_calibrationHandle, rawValue, &voltage));

  return voltage;
}

}// namespace adc

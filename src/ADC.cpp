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

#include "adc/ADC.hpp"

#include <esp_adc/adc_cali_scheme.h>
#include <esp_log.h>

namespace adc {

auto const TAG = "ADC";

auto const KILOHERTZ = 1000;
auto const ADC_BIT_WIDTH = ADC_BITWIDTH_12;
auto const ADC_ATTENUATION = ADC_ATTEN_DB_12;
auto const ADC_CONVERT_MODE = ADC_CONV_SINGLE_UNIT_1;
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
auto const ADC_OUTPUT_FORMAT = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
#else
auto const ADC_OUTPUT_FORMAT = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
#endif

ADC::ADC(ADC::Unit const unit) : m_unit(unit) {
  adc_continuous_handle_cfg_t adcHandleConfiguration = {
      .max_store_buf_size = 1024,
      .conv_frame_size = ADC_FRAME_SIZE,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adcHandleConfiguration, &m_handle));

  adc_cali_line_fitting_config_t calibrationConfiguration = {
      .unit_id = static_cast<adc_unit_t>(unit),
      .atten = ADC_ATTENUATION,
      .bitwidth = ADC_BIT_WIDTH,
  };
  ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&calibrationConfiguration, &m_calibrationHandle));
}

ADC::~ADC() {
  ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(m_calibrationHandle));
  ESP_ERROR_CHECK(adc_continuous_deinit(m_handle));
}

ChannelPtr ADC::createChannel(ADC::ChannelNumber channel) {
  auto const numberOfMaximumUnits = SOC_ADC_CHANNEL_NUM(m_unit);
  if ((m_configurations.size() + 1) >= numberOfMaximumUnits) {
    ESP_LOGE(TAG, "Reached maximum number of channels");
    return nullptr;
  }

  m_configurations.push_back({
      .atten = ADC_ATTENUATION,
      .channel = static_cast<std::uint8_t>(channel & 0x7),
      .unit = static_cast<std::uint8_t>(m_unit),
      .bit_width = ADC_BIT_WIDTH,
  });

  reconfigure();

  return std::make_unique<Channel>(channel, m_handle, m_calibrationHandle);
}

void ADC::reconfigure() {
  adc_continuous_config_t adcContinuousConfiguration = {
      .pattern_num = m_configurations.size(),
      .adc_pattern = m_configurations.data(),
      .sample_freq_hz = 100 * KILOHERTZ,
      .conv_mode = ADC_CONVERT_MODE,
      .format = ADC_OUTPUT_FORMAT,
  };

  ESP_ERROR_CHECK_WITHOUT_ABORT(adc_continuous_stop(m_handle));
  ESP_ERROR_CHECK(adc_continuous_config(m_handle, &adcContinuousConfiguration));
  ESP_ERROR_CHECK(adc_continuous_start(m_handle));
}

}// namespace adc

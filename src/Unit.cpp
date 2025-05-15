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

#include "adc/Unit.hpp"

#include <esp_adc/adc_cali_scheme.h>
#include <esp_log.h>

#include "sdkconfig.h"

namespace adc {

namespace {

auto const TAG = "ADC Unit";

auto const KILOHERTZ = 1000;
auto const FREQUENCY = 100 * KILOHERTZ;
auto const ADC_FRAME_SIZE = 256 * SOC_ADC_DIGI_RESULT_BYTES;
auto const ADC_BIT_WIDTH = ADC_BITWIDTH_12;
auto const ADC_ATTENUATION = ADC_ATTEN_DB_12;
auto const ADC_CONVERT_MODE = ADC_CONV_SINGLE_UNIT_1;

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
auto const ADC_OUTPUT_FORMAT = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
#else
auto const ADC_OUTPUT_FORMAT = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
#endif

using Result = esp_err_t;
using ContinousConfiguration = adc_continuous_config_t;
using DriverConfiguration = adc_continuous_handle_cfg_t;
using CalibrationConfiguration = adc_cali_line_fitting_config_t;

} // namespace

auto Unit::create(Unit::Number unitNumber) -> std::expected<Unit::Pointer, Unit::Error> {
  DriverConfiguration const driverConfiguration = {
      .max_store_buf_size = 1024,
      .conv_frame_size = ADC_FRAME_SIZE,
      .flags =
          {
              .flush_pool = true,
          },
  };

  Channel::ContinuousHandle continuousHandle{nullptr};

  Result const errorDriverHandle = adc_continuous_new_handle(&driverConfiguration, &continuousHandle);
  if (errorDriverHandle != ESP_OK) {
    return std::unexpected(Unit::Error::UNIT_CREATE_ERROR);
  }

  CalibrationConfiguration const calibrationConfiguration = {
      .unit_id = static_cast<adc_unit_t>(unitNumber),
      .atten = ADC_ATTENUATION,
      .bitwidth = ADC_BIT_WIDTH,
#if CONFIG_IDF_TARGET_ESP32
      .default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF,
#endif
  };

  Channel::CalibrationHandle calibrationHandle{nullptr};

  Result const errorCalibrationHandle = adc_cali_create_scheme_line_fitting(&calibrationConfiguration, &calibrationHandle);
  if (errorCalibrationHandle != ESP_OK) {
    return std::unexpected(Unit::Error::CALIBRATION_INIT_ERROR);
  }

  return Unit::Pointer(new Unit(unitNumber, continuousHandle, calibrationHandle));
}

Unit::Unit(Unit::Number const unitNumber, Channel::ContinuousHandle const continuousHandle, Channel::CalibrationHandle const calibrationHandle) noexcept
    : m_unitNumber(unitNumber), m_continuousHandle(continuousHandle), m_calibrationHandle(calibrationHandle) {}

Unit::~Unit() noexcept {
  adc_cali_delete_scheme_line_fitting(m_calibrationHandle);
  adc_continuous_deinit(m_continuousHandle);
}

auto Unit::createChannel(Channel::Number const channelNumber) noexcept -> std::expected<Channel::Pointer, Unit::Error> {
  std::uint8_t const numberOfMaximumUnits = SOC_ADC_CHANNEL_NUM(m_unitNumber);

  if (m_configurations.size() >= numberOfMaximumUnits) {
    ESP_LOGE(TAG, "Reached maximum number of channels");
    return std::unexpected(Unit::Error::DEVICE_MAX_SIZE);
  }

  Unit::Configuration const configuration = {
      .atten = ADC_ATTENUATION,
      .channel = channelNumber,
      .unit = m_unitNumber,
      .bit_width = ADC_BITWIDTH_12,
  };

  m_configurations.push_back(configuration);

  bool const isConfigured = reconfigure();
  if (not isConfigured) {
    return std::unexpected(Unit::Error::DEVICE_CONFIGURATION_ERROR);
  }

  return Channel::Pointer(new (std::nothrow) Channel(channelNumber, m_continuousHandle, m_calibrationHandle));
}

auto Unit::reconfigure() -> bool {
  ContinousConfiguration const configuration = {
      .pattern_num = m_configurations.size(),
      .adc_pattern = m_configurations.data(),
      .sample_freq_hz = FREQUENCY,
      .conv_mode = ADC_CONVERT_MODE,
      .format = ADC_OUTPUT_FORMAT,
  };

  adc_continuous_stop(m_continuousHandle);

  if (adc_continuous_config(m_continuousHandle, &configuration) != ESP_OK) {
    return false;
  }

  if (adc_continuous_start(m_continuousHandle) != ESP_OK) {
    return false;
  }

  return true;
}

} // namespace adc

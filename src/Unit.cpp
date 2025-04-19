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

auto const TAG = "ADC";

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

using Error = esp_err_t;
using DriverConfiguration = adc_continuous_handle_cfg_t;
using CalibrationConfiguration = adc_cali_line_fitting_config_t;

} // namespace

Unit::Unit(Unit::UnitNumber const unitNumber) : unitNumber(unitNumber) {
  DriverConfiguration const driverConfiguration = {
      .max_store_buf_size = 1024,
      .conv_frame_size = ADC_FRAME_SIZE,
      .flags =
          {
              .flush_pool = true,
          },
  };

  Error const errorDriverHandle = adc_continuous_new_handle(&driverConfiguration, &driverHandle);
  if (errorDriverHandle != ESP_OK) {
    throw std::system_error(std::error_code(errorDriverHandle, std::system_category()), "Failed to create ADC continuous driver handle");
  }

  CalibrationConfiguration const calibrationConfiguration = {
      .unit_id = static_cast<adc_unit_t>(unitNumber),
      .atten = ADC_ATTENUATION,
      .bitwidth = ADC_BIT_WIDTH,
#if CONFIG_IDF_TARGET_ESP32
      .default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF,
#endif
  };

  Error const errorCalibrationHandle = adc_cali_create_scheme_line_fitting(&calibrationConfiguration, &calibrationHandle);
  if (errorCalibrationHandle != ESP_OK) {
    throw std::system_error(std::error_code(errorCalibrationHandle, std::system_category()), "Failed to create ADC calibration scheme");
  }
}

auto Unit::createChannel(Channel::ChannelNumber const channelNumber) -> Channel {
  std::uint8_t const numberOfMaximumUnits = SOC_ADC_CHANNEL_NUM(unitNumber);
  if (numberOfMaximumUnits == channelConfigurations.size()) {
    ESP_LOGE(TAG, "Reached maximum number of channels");
    return std::unexpected(make_error_code(std::errc::too_many_files_open));
  }

  channelConfigurations.push_back(ChannelConfiguration{
      .atten = ADC_ATTENUATION,
      .channel = static_cast<std::uint8_t>(channelNumber & 0x7),
      .unit = static_cast<std::uint8_t>(unitNumber),
      .bit_width = ADC_BIT_WIDTH,
  });

  bool const isReconfigured = reconfigure();
  if (not isReconfigured) {
    return std::unexpected(make_error_code(std::errc::io_error));
  }

  return Channel(channelNumber, driverHandle, calibrationHandle);
}

auto Unit::reconfigure() -> bool {
  adc_continuous_config_t const adcContinuousConfiguration = {
      .pattern_num = channelConfigurations.size(),
      .adc_pattern = channelConfigurations.data(),
      .sample_freq_hz = FREQUENCY,
      .conv_mode = ADC_CONVERT_MODE,
      .format = ADC_OUTPUT_FORMAT,
  };

  if (adc_continuous_stop(driverHandle) != ESP_OK) {
    return false;
  }

  if (adc_continuous_config(driverHandle, &adcContinuousConfiguration) != ESP_OK) {
    return false;
  }

  if (adc_continuous_start(driverHandle) != ESP_OK) {
    return false;
  }

  return true;
}

} // namespace adc

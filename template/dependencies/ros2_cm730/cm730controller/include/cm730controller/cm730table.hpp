// Copyright 2019 Bold Hearts
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

#ifndef CM730CONTROLLER__CM730TABLE_HPP_
#define CM730CONTROLLER__CM730TABLE_HPP_

#include <cstdint>

namespace cm730controller
{

/** Identifies a value in EEPROM or RAM.
 * See page 4 in MX28 Technical Specifications PDF for more information.
 */
enum class CM730Table : uint8_t
{
  MODEL_NUMBER_L    = 0,   /// Lowest byte of model number
  MODEL_NUMBER_H    = 1,   /// Highest byte of model number
  VERSION           = 2,   /// Information on the version of firmware
  ID                = 3,   /// ID of CM730
  BAUD_RATE         = 4,   /// Baud Rate of CM730
  RETURN_DELAY_TIME = 5,   /// Return Delay Time
  RETURN_LEVEL      = 16,   /// Status Return Level
  EEPROM_LENGTH     = 17,
  DXL_POWER         = 24,   /// Dynamixel Power
  LED_PANEL         = 25,   /// LED of back panel
  LED_5_L           = 26,   /// Low byte of Head LED
  LED_5_H           = 27,   /// High byte of Head LED
  LED_6_L           = 28,   /// Low byte of Eye LED
  LED_6_H           = 29,   /// High byte of Eye LED
  BUTTON            = 30,   /// Button
  GYRO_Z_L          = 38,   /// Low byte of Gyro Z-axis
  GYRO_Z_H          = 39,   /// High byte of Gyro Z-axis
  GYRO_Y_L          = 40,   /// Low byte of Gyro Y-axis
  GYRO_Y_H          = 41,   /// High byte of Gyro Y-axis
  GYRO_X_L          = 42,   /// Low byte of Gyro X-axis
  GYRO_X_H          = 43,   /// High byte of Gyro X-axis
  ACCEL_X_L         = 44,   /// Low byte of Accelerometer X-axis
  ACCEL_X_H         = 45,   /// High byte of Accelerometer X-axis
  ACCEL_Y_L         = 46,   /// Low byte of Accelerometer Y-axis
  ACCEL_Y_H         = 47,   /// High byte of Accelerometer Y-axis
  ACCEL_Z_L         = 48,   /// Low byte of Accelerometer Z-axis
  ACCEL_Z_H         = 49,   /// High byte of Accelerometer Z-axis
  VOLTAGE           = 50,   /// Present Voltage
  LEFT_MIC_L        = 51,   /// Low byte of Left Mic. ADC value
  LEFT_MIC_H        = 52,   /// High byte of Left Mic. ADC value
  ADC2_L            = 53,   /// Low byte of ADC 2
  ADC2_H            = 54,   /// High byte of ADC 2
  ADC3_L            = 55,   /// Low byte of ADC 3
  ADC3_H            = 56,   /// High byte of ADC 3
  ADC4_L            = 57,   /// Low byte of ADC 4
  ADC4_H            = 58,   /// High byte of ADC 4
  ADC5_L            = 59,   /// Low byte of ADC 5
  ADC5_H            = 60,   /// High byte of ADC 5
  ADC6_L            = 61,   /// Low byte of ADC 6
  ADC6_H            = 62,   /// High byte of ADC 6
  ADC7_L            = 63,   /// Low byte of ADC 7
  ADC7_H            = 64,   /// High byte of ADC 7
  ADC8_L            = 65,   /// Low byte of ADC 8
  ADC8_H            = 66,   /// High byte of ADC 8
  RIGHT_MIC_L       = 67,   /// Low byte of Right Mic. ADC value
  RIGHT_MIC_H       = 68,   /// High byte of Right Mic. ADC value
  ADC10_L           = 69,   /// Low byte of ADC 9
  ADC10_H           = 70,   /// High byte of ADC 9
  ADC11_L           = 71,   /// Low byte of ADC 10
  ADC11_H           = 72,   /// High byte of ADC 10
  ADC12_L           = 73,   /// Low byte of ADC 11
  ADC12_H           = 74,   /// High byte of ADC 11
  ADC13_L           = 75,   /// Low byte of ADC 12
  ADC13_H           = 76,   /// High byte of ADC 12
  ADC14_L           = 77,   /// Low byte of ADC 13
  ADC14_H           = 78,   /// High byte of ADC 13
  ADC15_L           = 79,   /// Low byte of ADC 14
  ADC15_H           = 80,   /// High byte of ADC 14
  MAXNUM_ADDRESS
};

}  // namespace cm730controller

#endif  // CM730CONTROLLER__CM730TABLE_HPP_

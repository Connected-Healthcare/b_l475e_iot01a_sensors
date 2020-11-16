/**
 ******************************************************************************
 * @file    main.cpp
 * @author  CLab
 * @version V1.0.0
 * @date    5-September-2017
 * @brief   Simple Example application for using X_NUCLEO_IKS01A2
 *          MEMS Inertial & Environmental Sensor Nucleo expansion and
 *          B-L475E-IOT01A2 boards.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *notice, this list of conditions and the following disclaimer in the
 *documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes */
#include "mbed.h"

#include "HTS221Sensor.h"
#include "LPS22HBSensor.h"
#include "LSM6DSLSensor.h"

#include "lis3mdl_class.h"

#include "VL53L0X.h"

static DevI2C devI2c(PB_11, PB_10);
static LPS22HBSensor press_temp(&devI2c);
static HTS221Sensor hum_temp(&devI2c);
static LSM6DSLSensor acc_gyro(&devI2c, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, PD_11);
static LIS3MDL magnetometer(&devI2c);

static DigitalOut shutdown_pin(PC_6);
static VL53L0X range(&devI2c, &shutdown_pin, PC_7);

#include "spec_co.hpp"

static spec::CarbonMonoxide co(PA_0, PA_1);

int main() {
  hum_temp.init(NULL);
  press_temp.init(NULL);
  magnetometer.init(NULL);
  acc_gyro.init(NULL);
  range.init_sensor(VL53L0X_DEFAULT_ADDRESS);
  co.initialize();

  hum_temp.enable();
  press_temp.enable();
  acc_gyro.enable_x();
  acc_gyro.enable_g();

  printf("\r\n--- Starting new run ---\r\n\r\n");

  uint8_t id;
  hum_temp.read_id(&id);
  printf("HTS221  humidity & temperature    = 0x%X\r\n", id);

  press_temp.read_id(&id);
  printf("LPS22HB pressure & temperature    = 0x%X\r\n", id);

  magnetometer.read_id(&id);
  printf("LIS3MDL magnetometer              = 0x%X\r\n", id);

  acc_gyro.read_id(&id);
  printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);

  // range.read_id(&id);
  // printf("VL53L0X Time of Flight and Gesture detection sensor = 0x%X\r\n",
  // id);
  ThisThread::sleep_for(1000);

  printf("--- Reading sensor values ---\n\r");
  ;

  float hum_temp__temperature, hum_temp__humidity;
  float press_temp__temperature, press_temp__pressure;
  int32_t axes[3];
  while (1) {
    hum_temp.get_temperature(&hum_temp__temperature);
    hum_temp.get_humidity(&hum_temp__humidity);
    printf("HTS221: [temp] %.2f C, [hum] %.2f\r\n", hum_temp__temperature,
           hum_temp__humidity);

    press_temp.get_temperature(&press_temp__temperature);
    press_temp.get_pressure(&press_temp__pressure);
    printf("LPS22HB: [temp] %.2f C, [press] %.2f mbar\r\n",
           press_temp__temperature, press_temp__pressure);

    magnetometer.get_m_axes(axes);
    printf("LIS3MDL [mag/mgauss]:    %6ld, %6ld, %6ld\r\n", axes[0], axes[1],
           axes[2]);

    acc_gyro.get_x_axes(axes);
    printf("LSM6DSL [acc/mg]:        %6ld, %6ld, %6ld\r\n", axes[0], axes[1],
           axes[2]);

    acc_gyro.get_g_axes(axes);
    printf("LSM6DSL [gyro/mdps]:     %6ld, %6ld, %6ld\r\n", axes[0], axes[1],
           axes[2]);

    uint32_t distance;
    int status = range.get_distance(&distance);
    if (status == VL53L0X_ERROR_NONE) {
      printf("VL53L0X [mm]:            %6ld\r\n", distance);
    } else {
      printf("VL53L0X [mm]:                --\r\n");
    }

    printf("SPEC CO SENSOR: Gas Concentration (ppm) %lu Temperature (C) %d "
           "(Relative Humidity) %d\r\n",
           co.get_gas_concentration(), co.get_temperature(),
           co.get_relative_humidity());
    printf("-----\r\n");

    ThisThread::sleep_for(500);
  }
}

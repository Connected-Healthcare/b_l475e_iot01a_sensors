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

#include "internal_sensors.hpp"
#include "sgp30.hpp"
#include "spec_co.hpp"

static spec::CarbonMonoxide co(PA_0, PA_1);
static sensor::SGP30 sgp30(PB_9, PB_8);

int main() {
  internal_sensor::init_sensor();
  co.initialize();
  sgp30.start();

  printf("\r\n--- Starting new run ---\r\n\r\n");
  ThisThread::sleep_for(1000);

  while (1) {
    // Internal Sensor data
    internal_sensor::update_sensor_data();
    const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
    printf("HTS221: [temp] %.2f C, [hum] %.2f\r\n", data.hts221_temperature,
           data.hts221_humidity);
    printf("LPS22HB: [temp] %.2f C, [press] %.2f mbar\r\n",
           data.lps22hb_temperature, data.lps22hb_pressure);
    printf("LIS3MDL [mag/mgauss]:    %6ld, %6ld, %6ld\r\n",
           data.magnetometer_axes[0], data.magnetometer_axes[1],
           data.magnetometer_axes[2]);
    printf("LSM6DSL [acc/mg]:        %6ld, %6ld, %6ld\r\n",
           data.acceleration_axes[0], data.acceleration_axes[1],
           data.acceleration_axes[2]);
    printf("LSM6DSL [gyro/mdps]:     %6ld, %6ld, %6ld\r\n",
           data.gyroscope_axes[0], data.gyroscope_axes[1],
           data.gyroscope_axes[2]);
    printf("VL53L0X [mm]:            %6ld\r\n", data.distance);

    // CO Sensor
    printf("SPEC CO SENSOR: Gas Concentration (ppm) %lu Temperature (C) %d "
           "(Relative Humidity) %d\r\n",
           co.get_gas_concentration(), co.get_temperature(),
           co.get_relative_humidity());

    // SGP30 Sensor
    printf("SGP30: (co2) %d (voc) %d\r\n", sgp30.get_co2(), sgp30.get_voc());
    printf("-----\r\n");

    ThisThread::sleep_for(500);
  }
}

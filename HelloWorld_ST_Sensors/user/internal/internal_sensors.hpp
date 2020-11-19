#ifndef USER_INTERNAL_SENSORS_H_
#define USER_INTERNAL_SENSORS_H_

#include <cstdint>

namespace internal_sensor {

typedef struct _sensor_data {
  int32_t magnetometer_axes[3];
  int32_t acceleration_axes[3];
  int32_t gyroscope_axes[3];
  uint32_t distance;
  float hts221_temperature, hts221_humidity;
  float lps22hb_temperature, lps22hb_pressure;
} data_s;

void init_sensor(void);
void update_sensor_data(void);

const data_s &get_sensor_data(void);

} // namespace internal_sensor

#endif

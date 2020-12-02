#ifndef USER_INTERNET_H_
#define USER_INTERNET_H_

#include "sgp30.hpp"
#include "spec_co.hpp"

namespace internet {

typedef struct {
  spec::CarbonMonoxide *co;
  sensor::SGP30 *sgp30;
} sensors_t;

bool connect_as_tcp(void);

// TODO, UDP connection
// void connect_as_udp(void);

void send_sensor_data(sensors_t *sensors);

} // namespace internet

#endif

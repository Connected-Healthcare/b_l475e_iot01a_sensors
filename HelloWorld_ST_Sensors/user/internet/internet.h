#ifndef USER_INTERNET_H_
#define USER_INTERNET_H_

#include "gps.hpp"
#include "heartbeat.hpp"
#include "spec_co.hpp"

namespace internet {

typedef struct {
  spec::CarbonMonoxide *co;
  heartbeat::sparkfun_MAX32664 *hb;
  gps::adafruit_PA6H *gps;
} sensors_t;

bool connect_as_tcp(void);

// TODO, UDP connection
// void connect_as_udp(void);

void send_sensor_data(sensors_t *sensors);

} // namespace internet

#endif

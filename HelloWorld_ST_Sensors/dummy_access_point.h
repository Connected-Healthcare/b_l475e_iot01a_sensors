/**
 *
 * STEP 1: Update this WIFI Configuration file
 * STEP 2: Change the name to access_point.h
 *
 */
#ifndef ACCESS_POINT_H_
#define ACCESS_POINT_H_

#include "wifi.h"

// Make sure you use a 2.4GHZ WIFI Hotspot
// 5GHZ does not work
#define ACCESS_POINT_SSID "your_ssid"
#define ACCESS_POINT_PASSWORD "your_password"

// See WIFI_Ecn_t in wifi.h
#define ACCESS_POINT_ENCRYPTION WIFI_ECN_WPA2_PSK

// Remote address of your connection
// NOTE, Make sure this is only 4 numbers
#define REMOTE_ADDRESS_CONNECTION 192, 168, 1, 1
#define REMOTE_ADDRESS_PORT 8000

#endif

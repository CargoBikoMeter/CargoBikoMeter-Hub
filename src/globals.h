/*
 * Copyright (C) 2021 Cargobikometer Contributors
 * Contact: SAI-Lab Berlin (https://www.chemie.tu-berlin.de/sai_lab/sei_real_labor_sai_lab)
 * 
 * This file is part of the Cargobikometer firmware.
 *
 * The Cargobikometer firmware is free software: you can
 * redistribute it and/or modify it under the terms of the GNU
 * Lesser General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cargobikometer firmware is distributed in the hope that
 * it will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with the WZePaperDisplay firmware. If not,
 * see <http://www.gnu.org/licenses/>.
 */

// Based on https://github.com/openbikesensor/OpenBikeSensorFirmware/
// The OpenBikeSensor firmware is free software: you can
// redistribute it and/or modify it under the terms of the GNU
// Lesser General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option)
// any later version.

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "Arduino.h"
#include <ArduinoLog.h>
#include <HTTPClient.h>
#include <HTTPSServer.hpp>
#include <HTTPMultipartBodyParser.hpp>
#include <HTTPURLEncodedBodyParser.hpp>
#include <Preferences.h>
#include <SSLCert.hpp>
#include <Update.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>
#include <EEPROM.h>
#include "esp_sleep.h"
#include "HardwareSerial.h"
#include "TinyGPS++.h"

const char BUILD_DATE[] = __DATE__ " " __TIME__;

#define LOGLEVEL LOG_LEVEL_VERBOSE // SILENT|FATAL|ERROR|WARNING|INFO|NOTICE|TRACE|VERBOSE

// configuration structure object for global usage
// name length: max 15 character 
struct CBMConfig_s {
  char WIFI_SSID[33];        // wlan SSID
  char WIFI_PASSWORD[33];    // wlan password
  char AP_PASSWORD[13];      // password for soft access point
  char distance_day[7];      // distance daily, max.          999999
  char distance_tour[10];    // distance tour,  max.       999999999
  char distance_total[10];   // distance total in m, max.  999999999
  char wheel_size[8];        // in mm - e.g. Schwalbe Marathon Mondial: 47x622 - 2268 mm
  char dynamo_poles[3];      // defines the number of magnetic poles inside the hub dynamo  (Shimano:28, SON:26)
  char tx_interval[7];       // 
  char deep_sleep[7];        // 
  char project_url[51];      // url of project description 
  char Language[3];          // switch between: DE | EN | NL | FR | ES | SL
  char Timezone[40];         // choose your time zone from: https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv 
  char dnsServer[20];        // DNS server to use
  char ntpServer[40];        // choose from https://www.ntppool.org
  char gmtOffset[40];        // UK normal time is GMT, so GMT Offset is 0, for US (-5Hrs) is typically -18000, AU is typically (+8hrs) 28800 
  char daylightOffset[40];   // In the UK DST is +1hr or 3600-secs, other countries may use 2hrs 7200 or 30-mins 1800 or 5.5hrs 19800 Ahead of GMT use + offset behind - offset
  char hb_apiurl[51];        // server which receive heartbeat messages from the device - max length: 50
  char hb_authkey[71];        // API token to access the heartbeat server - max length: 70
};

#endif /* GLOBALS_H_ */
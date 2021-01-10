// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _cbm_main_H_
#define _cbm_main_H_
#include "Arduino.h"
//add your includes for the project here
#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>
#include <EEPROM.h>
#include "esp_sleep.h"
#include "HardwareSerial.h"
#include "TinyGPS++.h"
#include "VoltageMeter.h" // from OpenBikeSensor
#include "battery.h"      // from OpenBikeSensor

// include the device specific config which is linked
// in the filesystem to device_config.h
#include "device_config.h"


// defines if debug messages should be sent via USB serial line
// 0: no serial debug messages
// 1: normal user level debug messages
// 2: send also interrupt messages (should be avoided)
#define DUSB 1

// Serial Port speed
#define _BAUDRATE 115200 // works for debug messages to serial monitor

// define DEVELOP for messages during development
#define DEVELOP

//end of add your includes here

//add your function definitions
extern VoltageMeter* voltageMeter;


//Do not add code below this line
#endif /* _cbm_main_H_ */

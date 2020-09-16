// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _cbm_demoboard_H_
#define _cbm_demoboard_H_
#include "Arduino.h"
//add your includes for the project here
#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>
#include <EEPROM.h>
#include "esp_sleep.h"

#include "device_config.h"


// if OLED Display is connected to i2c
#define OLED 1                // Make define 1 on line if you have an OLED display connected



#include <U8x8lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//end of add your includes here


//add your function definitions for the project adlerkiez_iot_demo01 here



//Do not add code below this line
#endif /* _cbm_demoboard_H_ */

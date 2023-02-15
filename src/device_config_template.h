#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

// uncomment to use OTAA instead of ABP
//#define USE_OTAA

// UPDATE WITH YOUR DEVICE TTN SECRETS
// device: <name of the device in TTN console>
char device_name[11] = "TTN-DEVICE-NAME";

#ifdef USE_OTAA
static PROGMEM u1_t DEVEUI[8]  = {  }; // Device EUI, hex, lsb
static PROGMEM u1_t APPEUI[8]  = {  }; // Application EUI, hex, lsb
static PROGMEM u1_t APPKEY[16] = {  }; // App Key, hex, msb
#else
static PROGMEM u1_t NWKSKEY[16] = {  }; // LoRaWAN NwkSKey, network session key, hex, msb
static PROGMEM u1_t APPSKEY[16] = {  }; // LoRaWAN AppSKey, application session key, hex, msb
static PROGMEM u4_t DEVADDR = ; // LoRaWAN end-device address (DevAddr), hex, msb
#endif

// Pin definitions for Heltec ESP32 LoRaWAN based devices
// usable pins for input and output: 11, 13, 17, 21, 22, 23
// usable pins for input: 33, 36-39 (but not LoRa DIO pins 34, 35)
//
#define _PulseMeasurePin  2   // dynamo pulse (MUST use PIN 2 (PIN 25 not work with LoRa)
#define _BATTERY_PIN     36   // battery pin to use on device
#define _ADC_CORRECTION  1.6  // device specific correction factor 
#define _ButtonPin        0   // USER button: PRG button on device is GPIO0
#define _TouchPin        12   // touch button T1 on device is on PRG pin 0 
#define _GpsRxD          37   // read data from external GPS module
#define _GpsTxD          38   // send data to external GPS module

#define _MEASURE_INTERVAL   4 // how often we start the measure in seconds
#define _MOVEMENT_TIMEOUT 30 // seconds to wait for activating deep sleep (Default 120 seconds)

#define _TX_INTERVAL       900 // how often the data will be send via LoRaWAN (seconds)
#define _TIME_TO_SLEEP    3600 // deep sleep time (seconds)

// Serial Port speed for HardwareSerial
#define _HWS_BAUDRATE 9600   // speed for hardware serial interface 2

// if OLED Display is connected to i2c
#define OLED 1                // Make define 1 on line if you have an OLED display connected

#include <U8x8lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#if OLED==1
//
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
#endif

// define GPS stuff
// first enable or disable GPS_MODULE: 1 - GPS module attached,  0 - no GPS module attached
#define GPS_MODULE 1
// now define static GPS data, could be redefined later via LoRa command
// Alte Schule: _LAT 52.43892  _LON 13.54981
#define _LAT 52.43892
#define _LON 13.54981
#define _ALT 8

#endif

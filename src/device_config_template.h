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
#define _ResetPin        12   // reset DistanceTotal with HIGH-Signal on pin
#define _DebugPin        13   // set DEBUG level to one if pin is LOW
#define _GpsRxD          37   // read data from external GPS module
#define _GpsTxD          38   // send data to external GPS module

#define _MEASURE_INTERVAL   4 // how often we start the measure in seconds
#define _MOVEMENT_TIMEOUT 120 // seconds to wait for activating deep sleep

#define _TX_INTERVAL       60 // how often the data will be send via LoRaWAN
#define TIME_TO_SLEEP     600 // deep sleep time (in seconds)

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

// define GPS data
// example: CityLab Berlin:  _LAT 52.48389  _LON 13.38856
#define _LAT 52.48389
#define _LON 13.38856
#define _ALT 8

#endif

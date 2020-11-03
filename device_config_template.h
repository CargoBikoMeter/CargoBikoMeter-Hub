#pragma once

// uncomment to use OTAA instead of ABP
//#define USE_OTAA

// UPDATE WITH YOUR DEVICE TTN SECRETS
// device: <name of the device in TTN console>
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
const byte PulseMeasurePin = 2;    // dynamo pulse (MUST use PIN 2 (PIN 25 not work with LoRa)
const byte VoltageMeasurePin = 36; // dynamo pulse (MUST use PIN 2 (PIN 25 not work with LoRa)
const byte ResetPin = 12;          // reset DistanceTotal with HIGH-Signal on pin
const byte DebugPin = 13;          // set DEBUG level to one if HIGH
const byte VPin = 36;              // voltage measure pin

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

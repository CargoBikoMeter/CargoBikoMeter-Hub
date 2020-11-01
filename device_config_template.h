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

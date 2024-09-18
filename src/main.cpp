//
// Cargobikometer
//
// by: Roland Rutz
//

// This sketch is based on code at: https://www.weargenius.in/esp32-lorawan-node-using-arduino/
// It uses the MAC layer of LoRaWAN for Arduino/ESP32 is arduino-lmic by Matthijs Kooijman.
// https://github.com/matthijskooijman

// Do not remove the include below

#include "cbm_main.h"

extern void startConfigServer();
extern void readConfigurationData();
extern void writeConfigDataWheelSize();
extern void writeConfigDataDistanceTotal();
extern int  configServerUptimeMax;
extern void eraseConfigData();
extern CBMConfig_s CBMConfig;

// define the current development version
const char *CBMVersion = "1.1.1";

// define different debug level for the application
// this levels could be set directly on the device via HIGH level at specific pins
int debug = 1; // set debugging level, 0 - no messages, 1 - normal, 2 - extensive

// assign device specific pin definitions to global variables
const uint8_t buttonPin       = _ButtonPin;     // USER button button on device
const int     touchPin        = _TouchPin;      // touch button on device 
const int     LoRaSendLEDPin = _LoRaSendLEDPin; // LoRa send pin
const uint8_t PulseMeasurePin = _PulseMeasurePin;
const uint8_t BATTERY_PIN     = _BATTERY_PIN;
const uint8_t GpsRxD          = _GpsRxD;
const uint8_t GpsTxD          = _GpsTxD;

uint16_t WAIT_TIME_FOR_BUTTON = 10000;    // give user time to press USER button, in milliseconds
bool ConfigPinsState              = 0;    //
bool ConfigPinsPressed            = 0;
uint8_t CMD_COUNTER               = 0;  // count the seconds the USER (BOOT)_KEY is pressed

const uint8_t START_CONFIG_SERVER_COUNTER = 15;  // < 15s: start configuration server
const uint8_t CONFIG_RESET_COUNTER        = 20; //  > 20s: reset config data in NVS flash area
bool  ConfigError = false;

String cbmDeviceId, cbmWiFiId;

// sitch deep sleep on or off with external switch monitored with DeepSleepPin
// define deep sleep, 1 = enabled ; 0 = disabled 
int deep_sleep = 1;

// deep sleep definitions
#define uS_TO_S_FACTOR 1000000.0    /* Conversion factor for micro seconds to seconds */

// we count the boot cycles after powering up the device
RTC_DATA_ATTR int BootCount = 0;

// we count the TX cycles after powering up the device
RTC_DATA_ATTR int TxCount = 0;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations)
RTC_DATA_ATTR int32_t TX_INTERVAL = _TX_INTERVAL;

// deep sleep time (seconds)
RTC_DATA_ATTR int32_t TIME_TO_SLEEP = _TIME_TO_SLEEP;

// define variables for movement checking
// defines the timestamp for last detected movement
uint32_t LastMovementTime = 0;

// marks the TTN join state, will be set to true in
// function onEvent case EV_JOINED
bool JOINED = false;

// define, if display should be on or off
bool display = true;

// frequency definitions
RTC_DATA_ATTR int revolution_frequency = 0;      // 
float frequency = 0;                             // current measured frequency
uint32_t measuretime = 0;              // last time we measure the frequency

// Remember: pulse() returns an unsigned long value
// this is required for small frequency values below 10 Hz
unsigned long Htime;   // high time
unsigned long Ltime;   // low time
unsigned long Ttime;   // total time of a cycle
unsigned long period;
uint32_t PULSE_TIMEOUT = 2000000; // one second 1000000

unsigned long measurement_start = 0;    // time in millis
unsigned long measurement_interval = 0; // time in millis
uint32_t currentSeconds;

RTC_DATA_ATTR int wheel_size      = 0;  // will be defined by Config object during first boot

float distance_during_measurement = 0;  // counts the traveled distance during measurement interval in meter
//float distance_daily = 0;             // counts distance every day
// float distance_daily_km = 0;         //
RTC_DATA_ATTR int32_t distance_total = 0; // counts the overall distance and will be saved in RTC Memory
unsigned long distance_total_lora = 0;  // max distance: 42949672.95 km : 2^32 - 1s
float current_speed = 0;                // current speed
uint16_t voltage = 0;                   // will be finally sent to remote system
float ADC_CORRECTION = _ADC_CORRECTION; //
uint16_t BatteryAlarmLevel = 3100;      // battery alarm level in millivolt

// define the payload TX buffer for TTN
byte payload[12];
	// payload[0] = distance_total_lora;
	// payload[1] = distance_total_lora;
	// payload[2] = distance_total_lora;
	// payload[3] = distance_total_lora;
	// payload[4] = highByte(voltage);
	// payload[5] = lowByte(voltage);
	// payload[6] = ( LatitudeBinary >> 16 ) & 0xFF;
	// payload[7] = ( LatitudeBinary >> 8 ) & 0xFF;
	// payload[8] = LatitudeBinary & 0xFF;
	// payload[9] = ( LongitudeBinary >> 16 ) & 0xFF;
	// payload[10] = ( LongitudeBinary >> 8 ) & 0xFF;
	// payload[11] = LongitudeBinary & 0xFF;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
///RR void os_getArtEui (u1_t* buf) { }
///RR void os_getDevEui (u1_t* buf) { }
///RR void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 34, 35},
};

#if GPS_MODULE==1
// define GPS based stuff
TinyGPSPlus gps;
// because Hardware serial 1 on ESP LoRa points to
HardwareSerial SerialGPS(2); //
#endif

// define location variables for fixed or GPS module based location
bool GPSFix = false;
// save the coordinates in RTC memory, default: static coordinate from device_config.h
RTC_DATA_ATTR double lat = _LAT;
RTC_DATA_ATTR double lon = _LON;
RTC_DATA_ATTR int    alt = _ALT;

int GPSwarmupTime = 300;                   // GPS warm up time in seconds
uint32_t LatitudeBinary,  LongitudeBinary;
uint32_t measuretimeGPS = 0;              // last time we measure the frequency
#define _MEASURE_INTERVAL_GPS 10          // how often we start the measure


//
// ========== sub functions ===========

// Logging helper routines
void printTimestamp(Print* _logOutput, int logLevel) {
  static char c[12];
  sprintf(c, "%lu ", millis());
  _logOutput->print(c);
}

void printNewline(Print* _logOutput, int logLevel) {
  _logOutput->print('\n');
}

void setupLogging() {
  Log.begin(LOGLEVEL, &Serial);
  Log.setPrefix(printTimestamp);
  Log.setSuffix(printNewline);
  Log.verbose("Logging has started");
}

// switch Vext for GPS module ON or OFF
void VextON(void)
{
  if ( debug > 0 ) {
    Serial.println("DEBUG: switch on Vext");
  }
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
  if ( debug > 0 ) {
    Serial.println("INFO: switch off Vext");
  }
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

void flash_LED(void)
{
  // set the LoRaSendLEDPin to High
  digitalWrite(LoRaSendLEDPin, HIGH);
  delay(500);
  digitalWrite(LoRaSendLEDPin, LOW);
  delay(500);
  digitalWrite(LoRaSendLEDPin, HIGH);
  delay(500);
  digitalWrite(LoRaSendLEDPin, LOW);
}

void initEEPROM() {
	  Serial.println("Initialize EEPROM storage");
	  if (!EEPROM.begin(1000)) {
	    Serial.println("  Failed to initialize EEPROM");
	    Serial.println("  Restarting...");
	    delay(1000);
	    ESP.restart();
	  }

	  // write 1 m to EEPROM for the first time
	  // TODO: implement first time initialization for fresh ESP32 modules
//	  int address = 0;
//	  distance_total=EEPROM.readUInt(address);
//	  Serial.print("  first read distance_total from EEPROM in meter = ");
//	  Serial.println(distance_total);
//	  if ( distance_total == 0 ) {
//	    Serial.println("  write initial distance_total value=1 to EEPROM");
//	    EEPROM.writeUInt(address, 1);            // 2^32 - 1
//	    EEPROM.commit();
//	  }
}

void saveDistance() {
  // first convert integer into char array CBMConfig.distance_total
  sprintf (CBMConfig.distance_total, "%03i", distance_total);
  Log.verbose(F("save CBMConfig.distance_total : %s"), CBMConfig.distance_total);
  writeConfigDataDistanceTotal();
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void do_send(osjob_t* j){
	if ( debug > 0 ) {
	  Serial.println("DEBUG: do_send called");
	}

    // save distance_total each time LoRaWAN is sending data
    saveDistance();
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, payload, sizeof(payload), 0);
      Serial.println(F("Packet queued"));
	    //Increment TX counter number
	    ++TxCount;
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
			      JOINED= true; // CBM added line
            flash_LED(); // LED flashen
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            flash_LED();
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void getFrequency() {
  //float freq = 0;
  // use pulseIn function

  if ( debug > 0 ) {
    Serial.println("DEBUG: call pulseIn functions on PulseMeasurePin:" + String(PulseMeasurePin) );
  }

  // disable interrupt during measure
  noInterrupts();
  // measure HIGH and LOW
  Htime = pulseIn(PulseMeasurePin, HIGH, PULSE_TIMEOUT);   //read high time
  Ltime = pulseIn(PulseMeasurePin, LOW,  PULSE_TIMEOUT);    //read low time
  //enable interrupts again
  interrupts();

  if ( debug > 0 ) {
    Serial.println("DEBUG: after pulseIn functions" );
    Serial.println("DEBUG: Htime: " + String(Htime) );
    Serial.println("DEBUG: Ltime: " + String(Ltime) );
  }

  Ttime=0;
//  if ( (Htime > (Htime + Ltime)/3) and (Ltime > (Htime + Ltime)/3) ) {
//    Ttime = Htime + Ltime;
//  }

  Ttime = Htime + Ltime;

  if ( debug > 0 ) {
    Serial.println("DEBUG: Ttime: " + String(Ttime) );
  }

  // count only frequencies > 3 Hz eg.g Ttime < 333333
  if ( Ttime > 0 && Ttime < 333333 ) {
	// we MUST use double for high resolution
    frequency = 1000000 / (double)Ttime; //getting frequency with Ttime in micro seconds
  } else {
	frequency = 0; // set to zero if no further movement
  }

  // return frequency
  //return (freq);

}


void convertDistance() {
  // convert from meter into km
  //distance_total_km = distance_total / 1000.0 ;

  // convert distance_total_km into LoRa suitable value
  // max distance: 42949672.95 km : 2^32 - 1
  //   distance_total_lora = 4294967295;
  distance_total_lora = distance_total / 1000.0 * 100;

  // Prepare payload data
  payload[0] = ( distance_total_lora >> 24 ) & 0xFF;
  payload[1] = ( distance_total_lora >> 16 ) & 0xFF;
  payload[2] = ( distance_total_lora >>  8 ) & 0xFF;
  payload[3] =   distance_total_lora & 0xFF;

  if ( debug > 0 ) {
	Serial.println("");
	Serial.print("DEBUG: normalized distance_total_lora = ");
	Serial.println(distance_total_lora);
	Serial.println("DEBUG: Payload fields:");
	Serial.print("DEBUG: payload[0]: ");
	Serial.print(payload[0], BIN);
	Serial.println();
	Serial.print("DEBUG: payload[1]: ");
	Serial.print(payload[1], BIN);
	Serial.println();
	Serial.print("DEBUG: payload[2]: ");
	Serial.print(payload[2], BIN);
	Serial.println();
	Serial.print("DEBUG: payload[3]: ");
	Serial.print(payload[3], BIN);
	Serial.println();
  }
}

void getDistance() {
  current_speed = 0;
  distance_during_measurement = 0;

  if (Ttime > 0) {
	  // speed in m/s wheel_size in mm
    // 14 Hz = 1 revolution per second = 2,268 m/s * 3.6 = 8.16 km/h
	  current_speed = frequency / revolution_frequency  * wheel_size / 1000.0 ;

	  // distance in m/s
	  distance_during_measurement = current_speed * _MEASURE_INTERVAL;

	  // calculate the current speed in km/h
	  current_speed *= 3.6;

	  // count the total distance in m
	  distance_total += distance_during_measurement;

	  convertDistance();
  }
  
  if ( debug > 0 ) {
	Serial.println("");
	Serial.print("DEBUG: measurement_interval in seconds = ");
	Serial.println(_MEASURE_INTERVAL);
	Serial.print("DEBUG: current_speed in km/h = ");
	Serial.println(current_speed);
	Serial.print("DEBUG: distance_during_measurement in m = ");
	Serial.println(distance_during_measurement);
	Serial.print("DEBUG: distance_total in m = ");
	Serial.println(distance_total);
	Serial.println("");
  }
}

void resetDistanceTotal() {
  // reset the distance_total to 0 
  distance_total = 0;
}


#if OLED==1
// functions for OLED display

void showDisplayHeader() {
  if (display) {
	  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
	  u8x8.clear();
	  u8x8.inverse();
	  u8x8.print("CBM:");
	  u8x8.drawString(5,0,device_name);
	  u8x8.setFont(u8x8_font_chroma48medium8_r);
	  u8x8.noInverse();
    u8x8.setCursor(0, 1);
	  u8x8.print("V:");
	  u8x8.setCursor(2, 1);
	  u8x8.print(CBMVersion);  
  } else {
  	u8x8.clear();
  }
}

void showConfigErrorDisplay() {
  if (display) {
    showDisplayHeader();
    u8x8.setCursor(5, 3);
	  u8x8.print("ERROR");
	  u8x8.setCursor(1, 4);
	  u8x8.print("Configuration");
    u8x8.setCursor(3, 5);
	  u8x8.print("data empty");
    u8x8.setCursor(3, 6);
	  u8x8.print("touch CFG"); 
    u8x8.setCursor(1, 7);
	  u8x8.print("button for GUI");   
  } else {
  	u8x8.clear();
  }
}

void showConfigDisplay() {
  if (display) {
    showDisplayHeader();
	  u8x8.setCursor(1, 3);
	  u8x8.print("Configuration");
    u8x8.setCursor(1, 4);
	  u8x8.print("<15s: CFG-GUI");
    u8x8.setCursor(1, 5);
	  u8x8.print(">20s: RESET!");   
  } else {
  	u8x8.clear();
  }
}

void showConfigRunDisplay() {
  if (display) {
    showDisplayHeader();
	  u8x8.setCursor(1, 3);
	  u8x8.print("Configuration");
    u8x8.setCursor(1, 4);
	  u8x8.print(">5s: RESET km");
    u8x8.setCursor(1, 5);
	  u8x8.print("");   
  } else {
  	u8x8.clear();
  }
}

void showConfigDisplayAPPW() {
  if (display) {
    showDisplayHeader();
	  u8x8.setCursor(1, 3);
	  u8x8.print("Configuration");
    u8x8.setCursor(1, 4);
	  u8x8.print("<15s: CFG-GUI");
    u8x8.setCursor(1, 5);
	  u8x8.print(">20s: RESET!");   
  } else {
  	u8x8.clear();
  }
}

void showConfigServerDisplay() {
  if (display) {
    showDisplayHeader();
	  u8x8.setCursor(1, 3);
	  u8x8.print("Configuration");
    u8x8.setCursor(1, 4);
	  u8x8.print("   server");
    u8x8.setCursor(1, 5);
	  u8x8.print("  started");
    u8x8.setCursor(0, 6);
	  u8x8.print("SSID:");
    u8x8.setCursor(6, 6);
    u8x8.print(cbmWiFiId);    
  } else {
  	u8x8.clear();
  }
}

void showConfigServerDisplayAPPW() {
  if (display) {
    readConfigurationData(); // must read here to get the initial AP passwort
    showDisplayHeader();
	  u8x8.setCursor(1, 3);
	  u8x8.print("Configuration");
    u8x8.setCursor(1, 4);
	  u8x8.print("   server");
    u8x8.setCursor(1, 5);
	  u8x8.print("  started");
    u8x8.setCursor(0, 6);
	  u8x8.print("SSID:");
    u8x8.setCursor(6, 6);
	  u8x8.print(cbmWiFiId);        
    u8x8.setCursor(0, 7);
	  u8x8.print("PW:");
    u8x8.setCursor(4, 7);
	  u8x8.print(CBMConfig.AP_PASSWORD);
  } else {
  	u8x8.clear();
  }
}

void showDisplay() {
  if (display) {
	  showDisplayHeader();
	  u8x8.setCursor(8, 1);
	  u8x8.print("TX:");
	  u8x8.setCursor(11, 1);
	  u8x8.print(TX_INTERVAL/60);
	  u8x8.setCursor(0, 2);
	  u8x8.print(" Size: ");
	  u8x8.print(wheel_size, 1);
	  u8x8.clearLine(3);
	  u8x8.setCursor(0, 3);
	  u8x8.print(" Hz: ");
	  u8x8.setCursor(7, 3);
	  u8x8.print(frequency, 1);
	  u8x8.clearLine(4);
	  u8x8.setCursor(0, 4);
	  u8x8.print(" km/h:");
	  u8x8.setCursor(7, 4);
	  u8x8.print(current_speed, 1);
	  u8x8.clearLine(5);
	  u8x8.setCursor(0, 5);
	  u8x8.print(" km:");
	  u8x8.setCursor(7, 5);
	  u8x8.print(distance_total/1000.0);
	  u8x8.clearLine(6);
	  u8x8.setCursor(0, 6);
	  u8x8.print(" mV:");
	  u8x8.setCursor(7, 6);
	  u8x8.print(String(voltage));
	  u8x8.setCursor(0, 7);
	  u8x8.print("B");
	  u8x8.setCursor(1, 7);
	  u8x8.print(BootCount);
	  u8x8.setCursor(5, 7);
	  u8x8.print("T");
	  u8x8.setCursor(6, 7);
	  u8x8.print(TxCount);
	  u8x8.setCursor(10, 7);
	  u8x8.print("G");
	  u8x8.setCursor(11, 7);
	  u8x8.print(GPSFix);
    u8x8.setCursor(13, 7);
	  u8x8.print("D");
	  u8x8.setCursor(14, 7);
	  u8x8.print(deep_sleep);
  } else {
  	u8x8.clear();
  }
}


#endif

// check if user has pressed button or touchPin for configuration menue
void checkConfigPinsPressed() {
  Log.info(F("checkConfigPinsPressed  value"));

  CMD_COUNTER = 0;
  // check if button or touchPin are pressed
  // buttonPin pressed: HIGH -> LOW
  Log.verbose(F("  TouchRead value: %i"), touchRead(touchPin));  // for DEBUG
  if (  ! digitalRead(buttonPin) || (touchRead(touchPin) < 20) ) {
    ConfigPinsPressed = 1;
  } else {
    ConfigPinsPressed = 0;
  }

  Log.verbose(F("  ConfigPinsPressed state (pressed or touched = 1): %i"), ConfigPinsPressed);
  
  if ( ConfigPinsPressed == 1 && ! JOINED) { //check if button or touchPin was pressed before and being pressed now
    // show configuration screen messages
    showConfigDisplay();
    if (ConfigPinsState == 0)
    {
      ConfigPinsState = 1;
      Log.verbose(F("  Config button or touchPin pressed"));
      // now count the seconds the button ore touchPin is pressed
      while ( ConfigPinsState != 0 )
      {
        #if OLED==1
        u8x8.clearLine(7);
        u8x8.setCursor(2, 7);
	      u8x8.print("Seconds:");
        u8x8.setCursor(11, 8);
	      u8x8.print(CMD_COUNTER);
        #endif

        if ( ! digitalRead(buttonPin) || (touchRead(touchPin) < 20) ) {
          ConfigPinsState = 1;
        } else {
          ConfigPinsState = 0;
        }
        CMD_COUNTER++;
        Log.verbose(F(" CMD_COUNTER: %i"), CMD_COUNTER);

        if ( CMD_COUNTER  > CONFIG_RESET_COUNTER ) {
          #if OLED==1
          u8x8.clearLine(7);
          u8x8.setCursor(1, 7);
          u8x8.print("RESET CONFIG!");
          #endif
          // wait some seconds for user released button
          delay(5000);
          
          // erase configuration data
          Log.info(F("eraseConfigData"));
          eraseConfigData();
        }

        // increment counter every second
        delay(1000);

      }  // while loop

      // now process specific functions
      if ( CMD_COUNTER <= START_CONFIG_SERVER_COUNTER) {
        showConfigServerDisplay();
          
        startConfigServer();   
        CMD_COUNTER = 0;  // reset counter
        ConfigPinsState = 0;
      }

      CMD_COUNTER = 0;   // reset counter
      ConfigPinsState = 0;
    }
  }
  else {
    if (ConfigPinsState == 1) {
      ConfigPinsState = 0;
    }
  }

  if ( ConfigPinsPressed == 1 && JOINED && frequency == 0 ) { // RESET distance only, if no movement are detected
    Log.verbose(F("RESET distance  after 5 seconds ..."));
    showConfigRunDisplay();
    if (ConfigPinsState == 0)
    {
      ConfigPinsState = 1;
      Log.verbose(F("  Config button or touchPin pressed"));
      // now count the seconds the button ore touchPin is pressed
      while ( ConfigPinsState != 0 )
      {
        #if OLED==1
        u8x8.clearLine(7);
        u8x8.setCursor(2, 7);
	      u8x8.print("Seconds:");
        u8x8.setCursor(11, 8);
	      u8x8.print(CMD_COUNTER);
        #endif

        if ( ! digitalRead(buttonPin) || (touchRead(touchPin) < 20) ) {
          ConfigPinsState = 1;
        } else {
          ConfigPinsState = 0;
        }
        CMD_COUNTER++;
        Log.verbose(F(" CMD_COUNTER: %i"), CMD_COUNTER);

        // increment counter every second
        delay(1000);
      } // while loop

      if ( CMD_COUNTER  >= 5 ) {
        #if OLED==1
        u8x8.clearLine(7);
        u8x8.setCursor(1, 7);
        u8x8.print("RESET distance");
        #endif
        // wait some seconds for user released button
        delay(3000);
        Log.info(F("resetDistanceTotal"));
        resetDistanceTotal();
        saveDistance();     // save the distance 0 to EEPROM
        convertDistance(); // set the LoRa variable to 0
        showDisplay();
        delay(3000);
      }

      CMD_COUNTER = 0;   // reset counter
      ConfigPinsState = 0;
    }
  }
  else {
    if (ConfigPinsState == 1) {
      ConfigPinsState = 0;
    }
  }
  
  Log.verbose(F("checkConfigPinsPressed - end"));
}

void check_battery() {
	// voltage divider 1M/1M between 3.3V and ground
	// ADC range: 0 - 3.3V ; ADC resolution 4096 -> 3300 / 4096 = 0.8056
	// voltage divider requires: 0.8056 * 2 = 1,6113+
	uint8_t i=0; 
	for (i=1; i<=10; i++) {
	  voltage = voltage + ((float)analogRead(36)  * ADC_CORRECTION); 
    //Serial.println("DEBUG: counter i: " + String(i));
  }

	if ( debug > 0 ) {
	  Serial.println("DEBUG: Battery voltage after for loop: " + String(voltage) + " mV");
	}

  // calculate avarage value
  voltage = voltage/10;

	if ( debug > 0 ) {
	  Serial.println("DEBUG: Battery voltage: " + String(voltage) + " mV");
	}
	// set the TX payload variables
  payload[4] = highByte(voltage);
  payload[5] = lowByte(voltage);
}

void checkMovement() {
  uint32_t currentSeconds;

  // check, if there any movement based on Htime and Ltime variables
  // if not go into sleep mode when the last movement timestamp is far away
  if ( Htime + Ltime == 0) {
	  currentSeconds = (uint32_t) millis() / 1000;
	  if ( debug > 0 ) {
		Serial.print("DEBUG2: currentSeconds: ");
		Serial.println(currentSeconds);
		Serial.print("DEBUG2: LastMovementTime: ");
		Serial.println(LastMovementTime);
		Serial.print("DEBUG2: _MOVEMENT_TIMEOUT: ");
		Serial.println(_MOVEMENT_TIMEOUT);
	  }
	  if ( (currentSeconds - LastMovementTime) >= _MOVEMENT_TIMEOUT ) {
		  Serial.print("INFO: no movement since: ");
		  Serial.println(_MOVEMENT_TIMEOUT);

		  #if OLED==1
			  // show on OLED
			  u8x8.clear();
			  u8x8.setCursor(0, 3);
			  u8x8.print("No dynamo signal");
		  #endif
      delay(5000);

      // if deep sleep enabled go into deep sleep now
      if ( deep_sleep) {
        Serial.println("INFO: going into deep sleep mode in 5 seconds, by by ...");
		    Serial.println("INFO: wakeup with GPIO2 or after sleeptime: " + String(TIME_TO_SLEEP) + " seconds");
             #if OLED==1
			  u8x8.setCursor(0, 4);
			  u8x8.print("ENTER DEEP SLEEP");
			  u8x8.setCursor(0, 5);
			  u8x8.print("in 5 seconds");
			 #endif

		  // if battery level is below alarm level, write current distance into EEPROM
      if ( voltage < BatteryAlarmLevel) {
        Serial.println("WARN: Low battery, saving current distance into EEPROM ...");
        	saveDistance();
            #if OLED==1
      	     u8x8.setCursor(0, 7);
      	     u8x8.print("DISTANCE SAVED");
			      #endif
      }
      
      // switch Vext for GPS sensor OFF to save battery power
      VextOFF();

		  delay(5000);
	    // goto deep sleep now
      Serial.println("INFO: now calling esp_deep_sleep_start");
	    esp_deep_sleep_start();
      }
	  } 
  } else {
	  // set timestamp for last detected movement
	  LastMovementTime = (uint32_t) millis() / 1000;
  }
}

void measure() {
  uint32_t currentSeconds;

  currentSeconds = (uint32_t) millis() / 1000;
  if ( debug > 1 ) {
    Serial.print("DEBUG1: currentSeconds: ");
    Serial.println(currentSeconds);
    Serial.print("DEBUG1: measuretime: ");
    Serial.println(measuretime);
    Serial.print("DEBUG1: _MEASURE_INTERVAL: ");
    Serial.println(_MEASURE_INTERVAL);
  }

  if ((currentSeconds - measuretime) >= _MEASURE_INTERVAL) {  // Wake up every xx seconds

    // get the dynamo frequency
	  //frequency = getFreq();
    //frequency = getFrequency();
    getFrequency();
      if ( debug > 0 ) {
        Serial.print("DEBUG: Frequency: ");
        Serial.println(frequency);
      }


    // read the state of the user button or touch pin value now
    // this enables some functions during main loop
    checkConfigPinsPressed();

    // now calculate the distance moved during the measure interval
    getDistance();

    measuretime = currentSeconds;

    // check some environments
    // wait some seconds after startup before battery check
    //delay(3000)
    check_battery();

    checkMovement();

    // show the measured values on display
    #if OLED==1
        showDisplay();
      #endif
    }
}

void initLoRa() {
  //yield();
  Serial.println(F("Initialize LoRa modem ..."));
  SPI.begin(5, 19, 27);
  
  // LMIC initialization
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  //LMIC_setLinkCheckMode(0);
  LMIC_setAdrMode(false);     // Disable ADR
  //LMIC_setDrTxpow(DR_SF9,14);

  // Start job
  do_send(&sendjob);

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

/*
 * set the GPS data into LoRa payload array
 */
void setGPS() {

	  //char t
	  LatitudeBinary = ((lat + 90) / 180.0) * 16777215;
	  LongitudeBinary = ((lon + 180) / 360.0) * 16777215;

	  // fill the TX buffer
	  payload[6] = ( LatitudeBinary >> 16 ) & 0xFF;
	  payload[7] = ( LatitudeBinary >> 8 ) & 0xFF;
	  payload[8] = LatitudeBinary & 0xFF;
	  payload[9] = ( LongitudeBinary >> 16 ) & 0xFF;
	  payload[10] = ( LongitudeBinary >> 8 ) & 0xFF;
	  payload[11] = LongitudeBinary & 0xFF;

	  if ( debug > 0 ) {
		  Serial.println("DEBUG: GPSFix: " + String(GPSFix));
		  //sprintf(t, "Lat: %f", lat );
		  Serial.print("DEBUG: Lat: ");
		  Serial.println(lat,6);
		  Serial.print("DEBUG: LatitudeBinary:  ");
		  Serial.print(LatitudeBinary,BIN);
		  Serial.println();

		  //sprintf(t, "Lng: %f", lon );
		  Serial.print("DEBUG: Lon: ");
		  Serial.println(lon,6);
		  Serial.print("DEBUG: LongitudeBinary: ");
		  Serial.print(LongitudeBinary,BIN);
		  Serial.println();

		  Serial.print("DEBUG: payload[6]: ");
		  Serial.print(payload[6], BIN);
		  Serial.println();
		  Serial.print("DEBUG: payload[7]: ");
		  Serial.print(payload[7], BIN);
		  Serial.println();
		  Serial.print("DEBUG: payload[8]: ");
		  Serial.print(payload[8], BIN);
		  Serial.println();
		  Serial.print("DEBUG: payload[9]: ");
		  Serial.print(payload[9], BIN);
		  Serial.println();
		  Serial.print("DEBUG: payload[10]: ");
		  Serial.print(payload[10], BIN);
		  Serial.println();
		  Serial.print("DEBUG: payload[11]: ");
		  Serial.print(payload[11], BIN);
		  Serial.println();
	  }
}

/*
 * get the GPS data
 * if no GPS module is attached, then use static defined data
 */
void getGPS() {
	//Serial.println("DEBUG: getGPS function entered ...");
	currentSeconds = (uint32_t) millis() / 1000;
    if ((currentSeconds - measuretimeGPS) >= _MEASURE_INTERVAL_GPS) {

#if GPS_MODULE==1
		for (unsigned long start = millis(); millis() - start < 1000;) {
			while ( SerialGPS.available() > 0 ) {
			  gps.encode(SerialGPS.read());
			}
		}
		//Serial.println("DEBUG: measure entered ...");
		if (gps.location.isValid()) {
			GPSFix= true;
			lat=gps.location.lat();
			lon=gps.location.lng();
			Serial.println("INFO: got GPSFix");
			//Serial.print("ALT=");  Serial.println(gps.altitude.meters());
			Serial.print("INFO: lat: ");
			Serial.println(lat,6);
			Serial.print("INFO: lon: ");
			Serial.println(lon,6);
		} else {
			GPSFix = false;
			Serial.println("INFO: no GPSFix!");

		}
#endif
      setGPS();
	  measuretimeGPS = currentSeconds;
	}
}


//
// ##################   main program ##############################
//
void setup() {
	Serial.begin(_BAUDRATE);  //INTIALIZING THE SERIAL COMMUNICATION
	delay(1000); //Take some time to open up the Serial Monitor

  setupLogging();
  delay(100);

  #if OLED==1
	// initialize OLED display
	u8x8.begin();
  #endif

	// print startup message to console
	Serial.println("#####################################");
	Serial.println("    Cargobikometer is starting ...   ");
	Serial.print("    Device: ");
	Serial.println(device_name);
    
  Serial.println("SRC: PIO-Heltec-ESP32-LoRaV2--GPS");
  Serial.println("Build: " + String(__DATE__) + " " + String(__TIME__));
	Serial.println("#####################################");

  Log.info(F("Version: %s Build: %s"), CBMVersion, BUILD_DATE);

  // generate device specific id
  cbmDeviceId = String((uint16_t)(ESP.getEfuseMac() >> 32), HEX);
  cbmDeviceId.toUpperCase();
  cbmWiFiId = "CBM-" + cbmDeviceId;
  // Serial.print("wzWiFiId: ");
  // Serial.println(wzWiFiId);
  Log.info(F("cbmDeviceId: %s"), cbmDeviceId);

  // define pin mode for setting debug mode on
	pinMode(buttonPin, INPUT_PULLUP);

 	// define pin mode for reading dynamo pulse frequency
	pinMode(PulseMeasurePin, INPUT_PULLUP); // changed from INPUT to INPUT_PULLUP

	// define pin mode for voltage measure ADC
	pinMode(BATTERY_PIN, INPUT);

  // define pin mode for LoRaSendLEDPin
	pinMode(LoRaSendLEDPin, OUTPUT);
  flash_LED();
  
	//Increment boot number and print it every reboot
	++BootCount;
	Serial.println("INFO-Setup: Boot number: " + String(BootCount));

	//Print the wakeup reason for ESP32
	print_wakeup_reason();

	//Configure GPIO39 as ext0 wake up source for HIGH logic level
	esp_sleep_enable_ext0_wakeup(GPIO_NUM_39,1);

	//Configure GPIO2 (frequency input pin) as ext0 wake up source for LOW logic level
	// because with no external signal pin2 bound to photo-coupler is high
	esp_sleep_enable_ext0_wakeup(GPIO_NUM_2,0);

	/*
	First we configure the wake up source
	We set our ESP32 to wake up after some seconds
	*/
	Serial.println("INFO-Setup: Prepare timer wakeup to sleep for " + String(TIME_TO_SLEEP) +
	" Seconds");
	esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // switch Vext pin ON for GPS sensor
  VextON();
  delay(1000);

  // important configuration data will be checked
  if ( BootCount == 1  ) {

 	  // read configuration data from Config object 
    readConfigurationData();

    // read the state of the user button value now
    // config server only starts after pressing the user button for 1 second
    checkConfigPinsPressed();  
    
    // check important configuration data
    if ( strlen(CBMConfig.Language) == 0 ) {
      Log.error(F("ERROR: boot=1 - Language empty"));
      #if OLED==1
      showDisplayHeader();
      u8x8.setCursor(1, 3);
      u8x8.print("Config ERROR");
      u8x8.setCursor(0, 4);
      u8x8.print("Language empty");
      #endif
      ConfigError = true;
      delay(5000);
    }

    if ( strlen(CBMConfig.WIFI_SSID) == 0 ) {
      Log.error(F("ERROR: boot=1 - WIFI SSID empty"));
      #if OLED==1
      showDisplayHeader();
      u8x8.setCursor(1, 3);
      u8x8.print("Config ERROR");
      u8x8.setCursor(0, 4);
      u8x8.print("WIFI SSID empty");
      #endif
      ConfigError = true;
      delay(5000);
    }

    if ( strlen(CBMConfig.wheel_size) == 0 ) {
      Log.error(F("ERROR-Setup: boot=1 - CBMConfig.wheel_size not defined, check config"));
      #if OLED==1
      showDisplayHeader();
      u8x8.setCursor(1, 3);
      u8x8.print("Config ERROR");
      u8x8.setCursor(0, 4);
      u8x8.print("wheel_size empty");
      #endif
      ConfigError = true;
      delay(5000);
    } else {
      // set the wheel_size
      wheel_size= atoi(CBMConfig.wheel_size);
      Log.info(F("INFO-Setup: boot=1 - RTC wheel_size based on CBMConfig.wheel_size set to: %i"), wheel_size);
    }

    if ( strlen(CBMConfig.dynamo_poles) == 0 ) {
      Log.error(F("ERROR: boot=1 - CBMConfig.dynamo_poles not defined"));
      #if OLED==1
      showDisplayHeader();
      u8x8.setCursor(1, 3);
      u8x8.print("Config ERROR");
      u8x8.setCursor(1, 4);
      u8x8.print("dynamo_poles");
      u8x8.setCursor(1, 5);
      u8x8.print("   empty");
      ConfigError = true;
      #endif
      delay(5000);
    } else {
      // set the revolution based on defined dynamo poles
      revolution_frequency= atoi(CBMConfig.dynamo_poles)/2;
      Log.info(F("INFO-Setup: boot=1 - RTC revolution_frequency based on CBMConfig.dynamo_poles set to: %i"), revolution_frequency);
    }

    if ( strlen(CBMConfig.distance_total) >= 0 ) {
      // set the distance_total only if defined manually or by function distance_total_autosave
      distance_total = atof(CBMConfig.distance_total);
      Serial.println("INFO-Setup: boot=1 - RTC distance_total read from CBMConfig object: " + String(distance_total) );
    }

    if ( strlen(CBMConfig.tx_interval) == 0 ) {
      Log.error(F("ERROR: boot=1 - CBMConfig.tx_interval not defined"));
      #if OLED==1
      showDisplayHeader();
      u8x8.setCursor(1, 3);
      u8x8.print("Config ERROR");
      u8x8.setCursor(1, 4);
      u8x8.print("tx_interval");
      u8x8.setCursor(1, 5);
      u8x8.print("   empty");
      ConfigError = true;
      #endif
      delay(5000);
    } else {
      // set the revolution based on defined dynamo poles
      TX_INTERVAL= atoi(CBMConfig.tx_interval);
      Log.info(F("INFO-Setup: boot=1 - RTC TX_INTERVAL based on CBMConfig.tx_interval set to: %i"), TX_INTERVAL);
    }

    if ( strlen(CBMConfig.deep_sleep) == 0 ) {
      Log.error(F("ERROR: boot=1 - CBMConfig.deep_sleep not defined"));
      #if OLED==1
      showDisplayHeader();
      u8x8.setCursor(1, 3);
      u8x8.print("Config ERROR");
      u8x8.setCursor(1, 4);
      u8x8.print("deep_sleep");
      u8x8.setCursor(1, 5);
      u8x8.print("   empty");
      ConfigError = true;
      #endif
      delay(5000);
    } else {
      // set the revolution based on defined dynamo poles
      TIME_TO_SLEEP= atoi(CBMConfig.deep_sleep);
      Log.info(F("INFO-Setup: boot=1 - RTC TIME_TO_SLEEP based on CBMConfig.deep_sleep set to: %i"), TIME_TO_SLEEP);
    }

    if (ConfigError) {
      Log.error(F("ERROR: boot=1 - Wrong configuration values in CBMConfig object, reboot!"));
      showConfigErrorDisplay();
      delay(10000);
      ESP.restart(); // restart device to give user a chance to edit configuration data
    }

    // set the voltage TX payload variable to 5 Volt to indicate system was booted for the first time
    voltage=5000;
    Log.info(F("INFO-Setup: boot=1 - setting voltage to indicate boot up to: %d mV"), voltage);
    payload[4] = highByte(voltage);
    payload[5] = lowByte(voltage);
  } else {

    // log RTC memeory data
    Log.info(F("INFO-Setup: RTC wheel_size based on CBMConfig.wheel_size set to: %i"), wheel_size);
    Log.info(F("INFO-Setup: RTC revolution_frequency based on CBMConfig.dynamo_poles set to: %i"), revolution_frequency);
    Serial.println("INFO-Setup: RTC distance_total read from CBMConfig object: " + String(distance_total) );

  } // BootCount == 1
  
	// get some parameters before LoRa initializes and will be sent this parameters

	// convert distance into LoRaWAN payload values
	convertDistance();
	Serial.println("INFO-Setup: distance_total_lora: " + String(distance_total_lora) );

  #if OLED==1
	showDisplay();
  #endif

  #if GPS_MODULE==1
    // get GPS position
    // initialize GPS
    SerialGPS.begin(_HWS_BAUDRATE, SERIAL_8N1, GpsRxD, GpsTxD);
    // now get GPS data from real sensor
    getGPS();
  #endif
    Serial.println("INFO-Setup: get GPS position from RTC memory ...");
    // print current GPS data from RTC memory
    Serial.print("INFO: Lat:: ");
    Serial.println(lat,6);
    Serial.print("INFO: Lon: ");
    Serial.println(lon,6);

    // set initial GPS data into LoRa payload, if no GPS is available use static coordinates
    setGPS();

      Serial.println("INFO-Setup: initialize LoRaWAN ...");
      // initialize LoRaWAN OTA 
      initLoRa();

}

void loop() {
	// start LoRa
	os_runloop_once();
 
  if ( JOINED ) {
    

	  // get the current distance
	  measure();

	  // get GPS position if GPS module is attached
	  // with no attached GPS module use static defined GPS data
	  getGPS();
	}
}

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

// define the current development timestamp
char version[9] = "20210117";

// define different debug level for the application
// this levels could be set directly on the device via HIGH level at specific pins
int debug = 0; // set debugging level, 0 - no messages, 1 - normal, 2 - extensive

// deep sleep definitions
#define uS_TO_S_FACTOR 1000000.0    /* Conversion factor for micro seconds to seconds */

// we count the boot cycles after powering up the device
RTC_DATA_ATTR int BootCount = 0;

// we count the TX cycles after powering up the device
RTC_DATA_ATTR int TxCount = 0;

// define variables for movement checking
// defines the timestamp for last detected movement
uint32_t LastMovementTime = 0;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations)
uint32_t TX_INTERVAL = _TX_INTERVAL;

// assign device specific pin definitions to global variables
const uint8_t PulseMeasurePin = _PulseMeasurePin;
const uint8_t BATTERY_PIN     = _BATTERY_PIN;
const uint8_t ResetPin        = _ResetPin;
const uint8_t DebugPin        = _DebugPin;
const uint8_t GpsRxD          = _GpsRxD;
const uint8_t GpsTxD          = _GpsTxD;

// define, if display should be on or off
bool display = true;

// frequency definitions
int freq_fixed = 14;
float frequency = 0;

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

uint16_t wheel_size = 2268;             // in mm - Tandem Schwalbe Marathon Mondial: 47x622 - 2268 mm
uint8_t dynamo_pulse = 14;              // defines the hub dynamo pulse per revolution (Shimano 14)
                                        // 14 Hz = 1 revolution per second = 2,268 m/s * 3.6 = 8.16 km/h

float distance_during_measurement = 0;  // counts the traveled distance during measurement interval in meter
//float distance_daily = 0;               // counts distance every day
// float distance_daily_km = 0;            //
RTC_DATA_ATTR float distance_total = 0; // counts the overall distance and will be saved in RTC Memory
//float distance_total_km = 0;            // will be finally sent to remote system
unsigned long distance_total_lora = 0;  // max distance: 42949672.95 km : 2^32 - 1s
//float travelling_time_of_day = 0;       // time of travelling for the day
//float travelling_time_total = 0;        // total travelling time
float current_speed = 0;                // current speed
uint16_t voltage = 0;                   // will be finally sent to remote system
VoltageMeter* voltageMeter;             // we use this object for getting battery status
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
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

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
//
void do_send(osjob_t* j) {
  if ( debug > 0) {
    Serial.println(F("DEBUG: do_send function entry ..."));
  }
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {


    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    //LMIC_setTxData2(1, distance_total_lora, sizeof(mydata) - 1, 0);
    Serial.println(F("INFO: do_send: Packet queued"));
    digitalWrite(BUILTIN_LED, HIGH);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  //u8x8.clearLine(6);
  //u8x8.clearLine(7);
  //u8x8.setCursor(0, 6);
  //u8x8.printf("TIME %lu", os_getTime());
  Serial.print(": ");
  switch (ev) {
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
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      digitalWrite(BUILTIN_LED, LOW);
      Serial.print("LMIC.dataLen(EV_TXCOMPLETE): ");
      Serial.println(LMIC.dataLen);
      // increment TxCount for OLED display and logging
      ++TxCount;
      Serial.println("INFO: TxCount= " + String(TxCount));

      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
    	 // data received in rx slot after tx
    	 Serial.print("Data Received: ");
    	 Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
    	 Serial.println();
    	 while(1) {
    	 }
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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
    default:
      Serial.println(F("Unknown event"));
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
  if (Ttime > 0) {
	  // speed in m/s wheel_size in mm
	  current_speed = ( frequency / dynamo_pulse ) * wheel_size / 1000.0 ;

	  // distance in m/s
	  distance_during_measurement = current_speed * _MEASURE_INTERVAL;

	  // calculate the current speed in km/h
	  current_speed *= 3.6;

	  // count the total distance in m
	  distance_total += distance_during_measurement;

	  convertDistance();

	  if ( debug > 0 )
	  {
		Serial.println("");
		Serial.print("DEBUG: measurement_interval in seconds = ");
		Serial.println(_MEASURE_INTERVAL);
		Serial.print("DEBUG: current_speed in km/h = ");
		Serial.println(current_speed);
		Serial.print("DEBUG: distance_during_measurement in m = ");
		Serial.println(distance_during_measurement);
		Serial.print("DEBUG: distance_total in m = ");
		Serial.println(distance_total);
	  }
  } else {
	current_speed = 0;
  }
}

#if OLED==1
// show on OLED display
void showDisplay() {
  if (display) {
	  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
	  u8x8.clear();
	  u8x8.inverse();
	  u8x8.print("CBM:");
	  u8x8.drawString(5,0,device_name);
	  u8x8.setFont(u8x8_font_chroma48medium8_r);
	  u8x8.noInverse();
	  u8x8.setCursor(0, 1);
	  u8x8.drawString(1,1,version);
	  u8x8.setCursor(10, 1);
	  u8x8.print("TX:");
	  u8x8.setCursor(13, 1);
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
	  u8x8.print("B:");
	  u8x8.setCursor(2, 7);
	  u8x8.print(BootCount);
	  u8x8.setCursor(6, 7);
	  u8x8.print("T:");
	  u8x8.setCursor(8, 7);
	  u8x8.print(TxCount);
	  u8x8.setCursor(12, 7);
	  u8x8.print("G:");
	  u8x8.setCursor(14, 7);
	  u8x8.print(GPSFix);
  } else {
  	u8x8.clear();
  }
}
#endif

void check_battery() {
	voltageMeter = new VoltageMeter; // takes a moment, so do it here
	//voltage = (float)analogRead(36) / 4096 * 3.3;
	// next line use a voltage divider 100k/100k
	//voltage = (int)analogRead(VoltageMeasurePin)*2;
	voltage = (int)(voltageMeter->read() * 1000);

	if ( debug > 0 ) {
	  Serial.println("DEBUG: Battery voltage: " + String(voltage) + " mV");
	}
	// set the TX payload variables
    payload[4] = highByte(voltage);
    payload[5] = lowByte(voltage);
}

void checkResetPin() {
	Serial.println("INFO: checking ResetPin ...");
	if (!digitalRead(ResetPin)){
	  Serial.println("INFO: setting distance_total to zero ");
	  distance_total = 0;
	  //distance_total_km = 0;
	  // save the current distance to EEPROM
	  // reset the address pointer to zero
	  int address = 0;
	  EEPROM.writeUInt(address, distance_total);            // 2^32 - 1
	  EEPROM.commit();
	  // clear the line on display
	  u8x8.clearLine(5);
	}
}

void checkDebugPins() {
	Serial.println("INFO: checking DebugPin ...");
	if (!digitalRead(DebugPin)){
	  Serial.println("INFO: setting DEBUG level to: 1");
      debug = 1;
	}
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
	  // save the current distance to EEPROM if battery level is very low
	  // reset the address pointer to zero
	  initEEPROM();
	  int address = 0;
	  EEPROM.writeUInt(address, distance_total);            // 2^32 - 1
	  EEPROM.commit();
}

void readEEPROM() {
	  // read the saved distance from EEPROM
	  int address = 0;
	  distance_total=EEPROM.readUInt(address);
	  Serial.print("INFO: read distance_total from EEPROM in meter = ");
	  Serial.println(distance_total);
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
		  Serial.println("INFO: going into deep sleep mode in 5 seconds, by by ...");
		  Serial.println("INFO: wakeup with GPIO2 or after sleeptime: " + String(TIME_TO_SLEEP) + " seconds");
		  if (display) {
			  // show on OLED
			  u8x8.clear();
			  u8x8.setCursor(0, 3);
			  u8x8.print("No dynamo signal");
			  u8x8.setCursor(0, 4);
			  u8x8.print("ENTER DEEP SLEEP");
			  u8x8.setCursor(0, 5);
			  u8x8.print("in 5 seconds");
		  }

		  // if battery level is below alarm level, write current distance into EEPROM
          if ( voltage < BatteryAlarmLevel) {
        	Serial.println("WARN: Low battery, saving current distance into EEPROM ...");
        	saveDistance();
        	if (display) {
      	     u8x8.setCursor(0, 7);
      	     u8x8.print("DISTANCE SAVED");
        	}
          }

		  delay(5000);
	      // goto deep sleep now
	      esp_deep_sleep_start();
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

    // now calculate the distance moved during the measure interval
    getDistance();

    measuretime = currentSeconds;

	// check some environments
    // wait some seconds after startup before battery check
    delay(3000);
	check_battery();

	checkMovement();

    // show the measured values on display
    showDisplay();

  }
}


void initLoRa() {
  yield();
  Serial.println(F("Initialize LoRa modem ..."));
  SPI.begin(5, 19, 27);
  // LMIC initialization
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

// RR: support for non EU regions removed
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI); // g2-band

  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);
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

	// print startup message to console
	Serial.println("#####################################");
	Serial.println("    Cargobikometer is starting ...   ");
	Serial.print("    Device: ");
	Serial.println(device_name);
	Serial.println("#####################################");

	// define pin mode for reading dynamo pulse frequency
	pinMode(PulseMeasurePin, INPUT_PULLUP); // changed from INPUT to INPUT_PULLUP

	// define pin mode for voltage measure ADC
	pinMode(BATTERY_PIN, INPUT);

	// define pin mode for setting debug mode on
	pinMode(DebugPin, INPUT_PULLUP);

	// define pin mode for resetting distance_total to 1
	pinMode(ResetPin, INPUT_PULLUP);

	//Increment boot number and print it every reboot
	++BootCount;
	Serial.println("INFO-Setup: Boot number: " + String(BootCount));

	//Print the wakeup reason for ESP32
	print_wakeup_reason();

	//Configure GPIO39 as ext0 wake up source for HIGH logic level
	//esp_sleep_enable_ext0_wakeup(GPIO_NUM_39,1);

	//Configure GPIO2 (frequency input pin) as ext0 wake up source for LOW logic level
	// because with no external signal pin2 bound to photo-coupler is high
	esp_sleep_enable_ext0_wakeup(GPIO_NUM_2,0);

	/*
	First we configure the wake up source
	We set our ESP32 to wake up after some seconds
	*/
	esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
	Serial.println("INFO-Setup: Prepare ESP32 to sleep for " + String(TIME_TO_SLEEP) +
	" Seconds");

	// check DEBUG level
	checkDebugPins();

	// check, if EEPROM distance should be set to zero
	checkResetPin();

	check_battery();
	Serial.println("INFO-Setup: current battery voltage: " + String(voltage) + " mV");

	// get some parameters before LoRa initializes and will be sent this parameters

	// set distance_total to EEPROM value if RTC memory variable distance_total is empty
	if ( distance_total == 0 ) {
	  // initialize EEPROM (read distance_total and convert to distance_total_lora before LoRa init sends value)
	  Serial.println("INFO-Setup: reading distance_total in meter from EEPROM: ");
	  initEEPROM();
      readEEPROM();
	}
	Serial.println("INFO-Setup: distance_total in meter: " + String(distance_total) );

	// convert distance into LoRaWAN payload values
	convertDistance();
	Serial.println("INFO-Setup: distance_total_lora: " + String(distance_total_lora) );

#if OLED==1
	// initialize OLED display
	u8x8.begin();
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

	// initialize LoRa
	initLoRa();
}

void loop() {
	// start LoRa
	os_runloop_once();

	// get the current distance
	measure();

	// get GPS position if GPS module is attached
	// with no attached GPS module use static defined GPS data
	getGPS();

}


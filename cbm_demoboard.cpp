//
// Cargobikometer Demonstrator
//
// by: Roland Rutz
//

// This sketch is based on code at: https://www.weargenius.in/esp32-lorawan-node-using-arduino/
// It uses the MAC layer of LoRaWAN for Arduino/ESP32 is arduino-lmic by Matthijs Kooijman.
// https://github.com/matthijskooijman

// Do not remove the include below
#include "cbm_demoboard.h"

#define _MEASURE_INTERVAL 5          // how often we start the measure
#define _MOVEMENT_TIMEOUT 60        // seconds

#if OLED==1
//
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
#endif

char version[9] = "20201101";

bool debug = false; // set DEBUGGING ON or OFF
bool debug2 = false; // set DEBUGGING Level 2 ON or OFF
bool debug3 = false; // return fix frequency value for simulation

// deep sleep definitions
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  600        /* sleep time (in seconds) */

RTC_DATA_ATTR int BootCount = 0;

// define variables for movement checking
uint32_t noMovementTime = 0;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 180;
//const unsigned TX_INTERVAL = 60;
//const unsigned TX_INTERVAL = 3600;
RTC_DATA_ATTR int TxCount = 0;

// Pin definitions
const byte PulseMeasurePin = 2; // dynamo pulse (MUST use PIN 2 (PIN 25 not work with LoRa)
const byte VoltageMeasurePin = 36; // dynamo pulse (MUST use PIN 2 (PIN 25 not work with LoRa)
const byte ResetPin = 12; // reset DistanceTotal with HIGH-Signal on pin
const byte DebugPin = 13; // set DEBUG level to one if HIGH

// define, if OLED display should be on or off
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
const uint32_t PULSE_TIMEOUT = 1000000; // one second 1000000

unsigned long measurement_start = 0; // // time in millis
unsigned long measurement_interval = 0; // // time in millis

const uint16_t wheel_size = 2268; // in mm - Tandem Schwalbe Marathon Mondial: 47x622 - 2268 mm
const uint8_t dynamo_pulse = 14; // defines the hub dynamo pulse per revolution (Shimano 14)
// 14 Hz = 1 revolution per second = 2,268 m/s * 3.6 = 8.16 km/h

float distance_during_measurement = 0; // counts the traveled distance during measurement interval in meter
float distance_daily = 0; // counts distance every day
float distance_daily_km = 0;
float distance_total = 0; // counts the overall distance
float distance_total_km = 0;
uint16_t distance_total_lora = 0; // fixed value for test only
float travelling_time_of_day = 0; // time of travelling for the day
float travelling_time_total = 0; // total travelling time
float current_speed = 0; // current speed
float average_speed = 0; //

const byte VPin = 36; // voltage measure pin
uint16_t voltage = 0;

// EEPROM address for saving distance
int address = 0;

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

//
// ========== sub functions ===========
//
void do_send(osjob_t* j) {
  if ( debug ) {
    Serial.println(F("DEBUG: do_send function entry ..."));
  }
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    byte payload[4];
    payload[0] = highByte(distance_total_lora);
    payload[1] = lowByte(distance_total_lora);
    payload[2] = highByte(voltage);
    payload[3] = lowByte(voltage);
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

int getFrequency() {
  float freq = 0;
  // use pulseIn function

  if ( debug ) {
    Serial.println("DEBUG: call pulseIn functions on PulseMeasurePin:" + String(PulseMeasurePin) );
  }
  // disable interrupt during measure
  noInterrupts();
  // measure HIGH and LOW
  Htime = pulseIn(PulseMeasurePin, HIGH, PULSE_TIMEOUT);   //read high time
  Ltime = pulseIn(PulseMeasurePin, LOW,  PULSE_TIMEOUT);    //read low time
  //enable interrupts again
  interrupts();

  if ( debug ) {
    Serial.println("DEBUG: after pulseIn functions" );
    Serial.println("DEBUG: Htime: " + String(Htime) );
    Serial.println("DEBUG: Ltime: " + String(Ltime) );
  }

  Ttime = Htime + Ltime;

  if ( Ttime > 0 ) {
    if ( debug ) {
      Serial.println("DEBUG: Ttime = " + String(Ttime) );
    }
    freq = 1000000 / Ttime; //getting frequency with Ttime is in micro seconds
  }
  // display frequency

  if ( debug3 ) {
    return (freq_fixed);
  } else {
    return (freq);
  }
}

void convertDistance() {
	  // convert from meter into km
	  distance_total_km = distance_total / 1000.0 ;

	  // convert distance_total_km into integer with
	  distance_total_lora = (int)distance_total_km;
}

void getDistance() {
  if (Ttime > 0) {
	  // speed in m/s wheel_size in mm
	  current_speed = ( frequency / dynamo_pulse ) * wheel_size / 1000.0 ;

	  // distance in m/s
	  distance_during_measurement = current_speed * _MEASURE_INTERVAL;

	  // calculate the current speed in km/h
	  current_speed *= 3.6;

	  // count the total distance in m of the current day
	  distance_daily += distance_during_measurement;
	  distance_daily_km = distance_daily / 1000.0 ;

	  // count the total distance in m
	  distance_total += distance_during_measurement;
	  // save the current distance to EEPROM
	  EEPROM.writeUInt(address, distance_total);            // 2^32 - 1
	  EEPROM.commit();

	  convertDistance();

	  if ( debug )
	  {
		Serial.print("DEBUG: measurement_interval in seconds = ");
		Serial.println(_MEASURE_INTERVAL);
		Serial.print("DEBUG: current_speed in km/h = ");
		Serial.println(current_speed);
		Serial.print("DEBUG: distance_during_measurement in m = ");
		Serial.println(distance_during_measurement);
		Serial.print("DEBUG: distance_daily in m = ");
		Serial.println(distance_daily);
		Serial.print("DEBUG: distance_total in m = ");
		Serial.println(distance_total);
		Serial.print("DEBUG: distance_total_lora in km = ");
		Serial.println(distance_total_lora);
		Serial.println("");
	  }
  } else {
	current_speed = 0;
  }
}

void showDisplay() {
  if (display) {
	  // show on OLED
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
	  u8x8.print(distance_total_km);
	  u8x8.clearLine(6);
	  u8x8.setCursor(0, 6);
	  u8x8.print(" mV:");
	  u8x8.setCursor(7, 6);
	  u8x8.print(voltage);
	  u8x8.setCursor(0, 7);
	  u8x8.print(" B:");
	  u8x8.setCursor(3, 7);
	  u8x8.print(BootCount);
	  u8x8.setCursor(7, 7);
	  u8x8.print("T:");
	  u8x8.setCursor(9, 8);
	  u8x8.print(TxCount);
  } else {
  	u8x8.clear();
  }
}

void measure() {
  uint32_t currentSeconds;

  currentSeconds = (uint32_t) millis() / 1000;
  if ( debug2 ) {
    Serial.print("DEBUG2: currentSeconds: ");
    Serial.println(currentSeconds);
    Serial.print("DEBUG2: measuretime: ");
    Serial.println(measuretime);
    Serial.print("DEBUG2: _MEASURE_INTERVAL: ");
    Serial.println(_MEASURE_INTERVAL);
  }

  if ((currentSeconds - measuretime) >= _MEASURE_INTERVAL) {  // Wake up every xx seconds

    // get the dynamo frequency
	//frequency = getFreq();
    frequency = getFrequency();
    if ( debug2 ) {
      Serial.print("DEBUG2: Frequency: ");
      Serial.println(frequency);
    }

    // now calculate the distance moved during the measure interval
    getDistance();

    measuretime = currentSeconds;

    // show the measured values on display
    showDisplay();
  }
}

void initDisplay(void) {
  if (display) {
	  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
	  u8x8.clear();
	  u8x8.inverse();
	  u8x8.print(" Cargobikometer");
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
	  u8x8.print(distance_total_km);
	  u8x8.clearLine(6);
	  u8x8.setCursor(0, 6);
	  u8x8.print(" mV:");
	  u8x8.setCursor(7, 6);
	  u8x8.print(voltage);
	  u8x8.setCursor(0, 7);
	  u8x8.print(" B:");
	  u8x8.setCursor(3, 7);
	  u8x8.print(BootCount);
	  u8x8.setCursor(7, 7);
	  u8x8.print("T:");
	  u8x8.setCursor(9, 8);
	  u8x8.print(TxCount);
  } else {
  	u8x8.clear();
  }
}

void initEEPROM() {
	Serial.println("Init EEPROM storage");
	  if (!EEPROM.begin(1000)) {
	    Serial.println("  Failed to initialize EEPROM");
	    Serial.println("  Restarting...");
	    delay(1000);
	    ESP.restart();
	  }

	  // write 1 m to EEPROM for the first time
	  // TODO: implement first time initialization for fresh ESP32 modules
	  distance_total=EEPROM.readUInt(address);
	  Serial.print("  first read distance_total from EEPROM in meter = ");
	  Serial.println(distance_total);
	  if ( distance_total == 0 ) {
	    Serial.println("  write initial distance_total value=1 to EEPROM");
	    EEPROM.writeUInt(address, 1);            // 2^32 - 1
	    EEPROM.commit();
	  }

	  // read the saved distance from EEPROM
	  distance_total=EEPROM.readUInt(address);
	  Serial.print("  second read distance_total from EEPROM in meter = ");
	  Serial.println(distance_total);
}

void initLoRa() {
  yield();
  Serial.println(F("Init LoRa modem ..."));
  SPI.begin(5, 19, 27);
  // LMIC init
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

void check_battery() {
	//voltage = (float)analogRead(36) / 4096 * 3.3;
	// next line use a voltage divider 100k/100k
	voltage = (int)analogRead(36)*2;
	if ( debug2 ) {
	  Serial.println("DEBUG2: Battery Voltage: " + String(voltage) + " V");
	}
}

void checkResetPin() {
	Serial.println("INFO: checking ResetPin ...");
	if (digitalRead(ResetPin)){
	  Serial.println("INFO: setting distance_daily to zero ");
	  Serial.println("INFO: setting distance_total to zero ");
	  distance_daily = 0;
	  distance_total = 0;
	  distance_total_km = 0;
	  // save the current distance to EEPROM
	  EEPROM.writeUInt(address, distance_total);            // 2^32 - 1
	  EEPROM.commit();
	  // clear the line on display
	  u8x8.clearLine(5);
	}
}

void checkDebugPins() {
	Serial.println("INFO: checking DebugPin ...");
	if (digitalRead(DebugPin)){
	  Serial.println("INFO: DEBUG level 1 enabled");
      debug = true;
	} else {
    	Serial.println("INFO: DEBUG level 1 disabled");
    	debug = false;
	}
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

void checkMovement() {
  uint32_t currentSeconds;

  // check, if there any movement based on frequency variable
  // if not go into sleep mode after some minutes
  if ( frequency == 0 ) {
	  currentSeconds = (uint32_t) millis() / 1000;
	  if ( debug2 ) {
		Serial.print("DEBUG2: currentSeconds: ");
		Serial.println(currentSeconds);
		Serial.print("DEBUG2: noMovementTime: ");
		Serial.println(noMovementTime);
		Serial.print("DEBUG2: _MOVEMENT_TIMEOUT: ");
		Serial.println(_MOVEMENT_TIMEOUT);
	  }
	  if ((currentSeconds - noMovementTime) >= _MOVEMENT_TIMEOUT) {
		  Serial.print("INFO: no movement since: ");
		  Serial.println(_MOVEMENT_TIMEOUT);
		  Serial.println("INFO: going into deep sleep mode in 5 seconds, by by ...");
		  Serial.println("INFO: wakeup with GPIO2 or after sleeptime: " + String(TIME_TO_SLEEP) + " seconds");
#if OLED==1
		  // show on OLED
		  u8x8.clear();
		  u8x8.setCursor(0, 3);
		  u8x8.print("No dynamo signal");
		  u8x8.setCursor(0, 4);
		  u8x8.print("ENTER DEEP SLEEP");
		  u8x8.setCursor(0, 5);
		  u8x8.print("in 5 seconds");
#endif
		  delay(5000);

		noMovementTime = currentSeconds;
	    // goto deep sleep now
	    esp_deep_sleep_start();
	  }
  }
}

//
// ##################   main program ##############################
//
void setup() {
	Serial.begin(115200);  //INTIALIZING THE SERIAL COMMUNICATION
	delay(1000); //Take some time to open up the Serial Monitor

	// print startup message to console
	Serial.println("#####################################");
	Serial.println("    Cargobikometer is starting ...   ");
	Serial.println("#####################################");

	// define pin mode for reading dynamo pulse frequency
	pinMode(PulseMeasurePin, INPUT); // changed from INPUT to INPUT_PULLUP

	// define pin mode for voltage measure ADC
	pinMode(VoltageMeasurePin, INPUT);

	// define pin mode for resetting distance_total to 1
	pinMode(ResetPin, INPUT);

	// check DEBUG level
	checkDebugPins();

	// check, if EEPROM distance should be set to zero
	checkResetPin();

	//Increment boot number and print it every reboot
	++BootCount;
	Serial.println("Boot number: " + String(BootCount));

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
	Serial.println("INFO: Setup ESP32 to sleep for " + String(TIME_TO_SLEEP) +
	" Seconds");

	// get values before LoRa initializes and sent these values

	// initialize EEPROM (read distance_total and convert to distance_tota_lora before LoRa init sends value)
	initEEPROM();
	convertDistance();
	Serial.println("INFO: distance_total_lora: " + String(distance_total_lora) + " km" );

	check_battery();
	Serial.println("INFO: current battery voltage: " + String(voltage) + " mV");

	// initialize OLED display
	u8x8.begin();
	initDisplay();
	showDisplay();

	// initialize LoRa
	initLoRa();

}


void loop() {
	os_runloop_once();
	measure();
	check_battery();
	checkMovement();
}


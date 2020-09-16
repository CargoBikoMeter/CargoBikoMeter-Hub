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

#define _MEASURE_INTERVAL 2          // how often we read the frequency in seconds

#if OLED==1
//
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
#endif

int debug = 1; // set DEBUGGING ON or OFF
int debug2 = 0; // return fix frequency value for simulation

// Pin definitions
int buttonPin = 13;         //
const byte measurePin = 25; // dynamo pulse

int freq_fixed = 14;
float frequency = 0;

uint32_t measuretime = 0;              // last time we measure the frequency

// Remember: pulse() returns an unsigned long value
// this is required for small frequency values below 10 Hz
unsigned long Htime;   // high time
unsigned long Ltime;   // low time
unsigned long Ttime;   // total time of a cycle
unsigned long period;
const uint32_t PULSE_TIMEOUT = 1000000; // one second

unsigned long measurement_start = 0; // // time in millis
unsigned long measurement_interval = 0; // // time in millis

const uint16_t wheel_size = 2268; // in mm Tandem Schwalbe Marathon Mondial: 47x622 - 2268 mm
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

// EEPROM address for saving distance
int address = 0;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 180;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 34, 35},
};


void do_send(osjob_t* j) {
  Serial.println(F("DEBUG: do_send function entry ..."));

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    byte payload[2];
    payload[0] = highByte(distance_total_lora);
    payload[1] = lowByte(distance_total_lora);
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    //LMIC_setTxData2(1, distance_total_lora, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
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
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
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
  // disable interrupt during measure
  noInterrupts();
  // measure HIGH and LOW
  if ( debug == 1 ) {
    Serial.println("DEBUG: call pulseIn functions on measurePin:" + String(measurePin) );
  }
  Htime = pulseIn(measurePin, HIGH, PULSE_TIMEOUT);   //read high time
  Ltime = pulseIn(measurePin, LOW,  PULSE_TIMEOUT);    //read low time
  if ( debug == 1 ) {
    Serial.println("DEBUG: after pulseIn functions" );
    Serial.println("DEBUG: Htime: " + String(Htime) );
    Serial.println("DEBUG: Ltime: " + String(Ltime) );
  }
  //enable interrupts again
  interrupts();

  Ttime = Htime + Ltime;
  if ( Ttime > 0 ) {
    if ( debug == 1 ) {
      Serial.println("DEBUG: Ttime = " + String(Ttime) );
    }
    freq = 1000000 / Ttime; //getting frequency with Ttime is in micro seconds
  }
  // display frequency

  if ( debug2 == 1 ) {
    return (freq_fixed);
  } else {
    return (freq);
  }
}


void getDistance() {
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

  // covert from meter into km
  distance_total_km = distance_total / 1000.0 ;

  // convert distance_total_km into integer with
  distance_total_lora = (int)distance_total_km;

  if ( debug == 1 )
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
}

void showDistance() {
  // show on OLED
  //u8x8.clearLine(3);
  //u8x8.clearLine(4);
  //u8x8.clearLine(5);
  u8x8.setCursor(0, 3);
  u8x8.print(" Hz: ");
  u8x8.setCursor(7, 3);
  u8x8.print(frequency, 1);
  u8x8.setCursor(0, 4);
  u8x8.print(" km/h:");
  u8x8.setCursor(7, 4);
  u8x8.print(current_speed, 1);
  u8x8.setCursor(0, 5);
  u8x8.print(" km:");
  u8x8.setCursor(7, 5);
  u8x8.print(distance_total_km);
}


void initDisplay(void)
{
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  u8x8.clear();
  u8x8.inverse();
  u8x8.print(" Cargobikometer");
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.noInverse();
  u8x8.setCursor(0, 1);
  u8x8.print(" 2020-02-18 180s");
  u8x8.setCursor(0, 3);
  u8x8.print(" Hz:");
  u8x8.setCursor(0, 4);
  u8x8.print(" km/h:");
  u8x8.setCursor(0, 5);
  u8x8.print(" km: ");
}

void initEEPROM() {
	Serial.println("\nInit EEPROM storage\n");
	  if (!EEPROM.begin(1000)) {
	    Serial.println("Failed to initialize EEPROM");
	    Serial.println("Restarting...");
	    delay(1000);
	    ESP.restart();
	  }

	  // write 1 m to EEPROM for the first time
	  distance_total=EEPROM.readUInt(address);
	  Serial.print("DEBUG: first read distance_total from EEPROM in meter = ");
	  Serial.println(distance_total);
	  if ( distance_total == 0 ) {
	    Serial.println("DEBUG: write 1 to EEPROM for test ");
	    EEPROM.writeUInt(address, 1);            // 2^32 - 1
	    EEPROM.commit();
	  }

	  // read the saved distance from EEPROM
	  distance_total=EEPROM.readUInt(address);
	  Serial.print("DEBUG: second read distance_total from EEPROM in meter = ");
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

void measure() {
  uint32_t currentSeconds;

  currentSeconds = (uint32_t) millis() / 1000;
  if ( debug == 2 ) {
    Serial.print("DEBUG: currentSeconds: ");
    Serial.println(currentSeconds);
    Serial.print("DEBUG: measuretime: ");
    Serial.println(measuretime);
    Serial.print("DEBUG: _MEASURE_INTERVAL: ");
    Serial.println(_MEASURE_INTERVAL);
  }

  if ((currentSeconds - measuretime) >= _MEASURE_INTERVAL) {  // Wake up every xx seconds

    // get the dynamo frequency
    frequency = getFrequency();
    if ( debug == 1 ) {
      Serial.print("DEBUG: Frequency: ");
      Serial.println(frequency);
    }

    getDistance();
    showDistance();
    measuretime = currentSeconds;
  }
}

// ##################   main program ##############################

void setup()
{
  Serial.begin(115200);  //INTIALISING THE SERIAL COMMUNICATION
  pinMode(measurePin, INPUT);

  // initialize OLED display
  u8x8.begin();
  initDisplay();

  // init EEPROM
  initEEPROM();

  // initialize LoRa
  initLoRa();

}


void loop() {
  os_runloop_once();
  measure();

}

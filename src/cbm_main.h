// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _cbm_main_H_
#define _cbm_main_H_

#include "globals.h"

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

//Do not add code below this line
#endif /* _cbm_main_H_ */
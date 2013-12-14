// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef lwip_test_H_
#define lwip_test_H_
#include "Arduino.h"
//add your includes for the project lwip_test here
#include "SamNonDuePin.h"
#include "lwip/src/include/lwip/netif.h"
#include <stdio.h>
#include "ethernet_sam.h"
#include "timer_mgt_sam.h"
#include <stdarg.h>
#undef printf

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

//add your function definitions for the project lwip_test here
void printf(char *fmt, ... );

//Do not add code below this line
#endif /* lwip_test_H_ */

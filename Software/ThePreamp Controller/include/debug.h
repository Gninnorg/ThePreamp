#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

#ifdef DEBUG_USE_WEBSERIAL
#include <WebSerial.h>
#endif

// To enable debug define DEBUG 1
// To disable debug define DEBUG 0
#ifndef DEBUG
#define DEBUG 1
#endif

#if DEBUG == 1
#ifdef DEBUG_USE_WEBSERIAL
#define debug(x) Serial.print(x); WebSerial.print(x);
#define debugln(x) Serial.println(x); WebSerial.println(x);
#else
#define debug(x) Serial.print(x);
#define debugln(x) Serial.println(x);
#endif
#else
#define debug(x)
#define debugln(x)
#endif

#endif

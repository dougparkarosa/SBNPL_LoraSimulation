#ifndef DEBUG_H
#define DEBUG_H
#include "LoraMeshConfiguration.h"

#if USE_ARDUINO
#include <HardwareSerial.h>
#else
#include <iostream>
#endif

#define str(x) #x
#define debugLog(x) Debug::printVar(str(x), x)
#define debugMsg(s) Debug::print(s)

namespace Debug {
#if USE_ARDUINO
extern HardwareSerial Serial;
#endif

void begin();
void end();

bool &on();

void enable();
void disable();

template <typename T> void printVar(const char *vName, const T &v) {
  if (on()) {
#if USE_ARDUINO
    if (Serial.availableForWrite()) {
      Serial.print(vName);
      Serial.print(": ");
      Serial.println(v);
      Serial.flush();
#else
    std::cout << vName << ": " << v << std::endl;
#endif // USE_ARDUINO
    }
  }

  template <typename T> void print(const T &v) {
    if (on()) {
#if USE_ARDUINO
      if (Serial.availableForWrite()) {
        Serial.println(v);
      }
#else
    std::cout << v << std::endl;
#endif // USE_ARDUINO
    }
  }

} // namespace Debug

#endif // DEBUG_H
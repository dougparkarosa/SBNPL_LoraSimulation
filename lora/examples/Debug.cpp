#ifndef DEBUG_H
#define DEBUG_H

#include "Debug.h"

#if USE_ARDUINO
#include <HardwareSerial.h>
#endif // USE_ARDUINO

namespace Debug {
#if USE_ARDUINO
HardwareSerial Serial(0);
void begin() { Serial.begin(115200); }
void end() { Serial.end(); }
#else
void begin() {}
void end() {}
#endif

bool &on() {
  static bool onVal = true;
  return onVal;
}

void enable() {
  // begin();
  on() = true;
}
void disable() {
  on() = false;
  // end();
}

} // namespace Debug

#endif // DEBUG_H
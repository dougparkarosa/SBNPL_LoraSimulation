#include "System.h"
#include "Debug.h"

#if USE_ARDUINO
#include <esp_system.h>
#endif

namespace System {
IDType getSystemID() {
  // Base System ID on Mac address
  IDType baseMacAddr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
#if USE_ARDUINO
  if (ESP_OK != esp_efuse_mac_get_default(baseMacAddr.data())) {
    // Log an error
    debugMsg("esp_efuse_mac_get_default failed in System::getSystemID()!");
  }
#else
  debugMsg("getSystemID not implemented!");
#endif
  return baseMacAddr;
}

} // namespace System
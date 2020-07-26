#ifndef SYSTEM_ID_H
#define SYSTEM_ID_H

#include "LoraMeshConfiguration.h"

#include <stdint.h>

#include "Array.h"

namespace System {
using IDType = Array<uint8_t, 6>;
} // namespace System

#endif // SYSTEM_ID_H

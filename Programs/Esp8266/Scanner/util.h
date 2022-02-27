#pragma once

#include <cstdint>

using uint   = unsigned int;
using uint8  = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

// TODO: Move this into more extensive logging header
#include <SoftwareSerial.h>
#define Warn(msg, fmt...) Serial.printf("WARN - " msg "\n", fmt)
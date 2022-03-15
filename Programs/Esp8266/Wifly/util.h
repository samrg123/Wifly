#pragma once

#include <cstdint>

using uint   = unsigned int;
using uint8  = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

#define LIKELY(x)   __builtin_expect(x, 1)
#define UNLIKELY(x) __builtin_expect(x, 0) 

// TODO: Move this into more extensive logging header
#include <SoftwareSerial.h>
#define Log(msg, fmt...) Serial.printf("MSG - " msg "\n", fmt)
#define Warn(msg, fmt...) Serial.printf("WARN - " msg "\n", fmt)


template<typename T, size_t kN>
inline constexpr size_t ArrayCount(const T (&)[kN]) { return kN; }


template<typename T, typename TBytes>
inline constexpr const void* ByteOffset(const T* ptr, TBytes bytes) {
    return reinterpret_cast<const char*>(ptr) + bytes;
}

template<typename T, typename TBytes>
inline constexpr void* ByteOffset(T* ptr, TBytes bytes) {
    return reinterpret_cast<char*>(ptr) + bytes;
}

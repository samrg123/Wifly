#pragma once

#include <cstdint>

using uint   = unsigned int;
using uint8  = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

#define LIKELY(x)   __builtin_expect(x, 1)
#define UNLIKELY(x) __builtin_expect(x, 0) 


#ifdef __INTELLISENSE__
    #define INTELISENSE_CHOOSE(a, b...) a 
#else
    #define INTELISENSE_CHOOSE(a, b...) b 
#endif

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

template<typename T1, typename T2>
constexpr auto Max(T1 a, T2 b) {
    return a >= b ? a : b;
}

template<typename T1, typename T2>
constexpr auto Min(T1 a, T2 b) {
    return a <= b ? a : b;
}

// TODO: Move this out to stringUtil.h
#include <WString.h>

// Hack to prevent intelisense failing to infer templates in WString.h
#ifdef __INTELLISENSE__
    template<typename T>
    inline String operator+(const String& str, T arg) {
        String result = String(str).concat(arg);
        return result;
    }
#endif

#define STRINGIZE(x) #x 
#define DEFER_STRINGIZE(x) STRINGIZE(x)

// Looks like IRAM_ATTR breaks the assembler so we just define our own IRAM_FUNC macro that does what its supposed to do
#define IRAM_FUNC __attribute__((section( ".iram.text." __FILE__ "." DEFER_STRINGIZE(__LINE__) "." DEFER_STRINGIZE(__COUNTER__) )))
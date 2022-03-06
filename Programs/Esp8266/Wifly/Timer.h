#pragma once

#include "util.h"

class Timer {
  private:

    uint64 timestampUs;

  public:

    inline constexpr Timer(uint64 timestampUs = 0): timestampUs(timestampUs) {}

    inline uint64 LapUs() {
      uint64 currentUs = micros64();
      uint64 deltaUs = currentUs - timestampUs;

      timestampUs = currentUs;
      
      return deltaUs;
    }

    inline float LapMs() {
      uint64 deltaUs = LapUs();
      return deltaUs * .001f;
    }

    inline float LapS() {
      uint64 deltaUs = LapUs();
      return deltaUs * .000001f;
    }    

};

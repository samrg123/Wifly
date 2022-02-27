#pragma once

class Timer {
  private:

    unsigned long timestampUs;

  public:

    inline Timer(bool start = true) {
      if(start) timestampUs = micros();
    }

    inline unsigned long LapUs() {
      unsigned long currentUs = micros();
      unsigned long deltaUs = currentUs - timestampUs;

      timestampUs = currentUs;
      
      return deltaUs;
    }

    inline float LapMs() {
      unsigned long deltaUs = LapUs();
      return deltaUs * .001f;
    }

    inline float LapS() {
      unsigned long deltaUs = LapUs();
      return deltaUs * .000001f;
    }    

};

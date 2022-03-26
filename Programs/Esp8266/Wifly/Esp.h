#pragma once

#include <Esp.h>
#include <SoftwareSerial.h>

class Esp: public EspClass {
    public:

        enum AdcMode {
            ADC_3V3 = 33,
            ADC_VDD = 255, //Note: A0 is disabled in this mode and must be floating
        };

        static inline constexpr AdcMode kAdcMode = ADC_3V3;

        static const char* FlashModeString(FlashMode_t mode) {
            switch(mode) {
                case FM_QIO:      return "QUAD Input/Output";
                case FM_QOUT:     return "QUAD Output";
                case FM_DIO:      return "Dual Input/Output";
                case FM_DOUT:     return "Dual Output";
                case FM_UNKNOWN:  return "Unknown";
                default:          return "INVALID";
            }
        }

        // Returns the voltage on VDD pin of ESP if kAdcMode == ADC_VDD,
        // Otherwise the voltage on A0 is returned.  
        static inline float getVcc() {

            if constexpr(kAdcMode == ADC_VDD) {

                // Note: correction for d1 mini having additional voltage divider attached to A0 (TOUT).
                constexpr float kVddScale = 3.295 / 2980.; 
                return system_get_vdd33() * kVddScale;

            } else {

                // Note: External voltage divider on ADC scales 0-5V to 
                //       0-1V ADC range.
                constexpr float kAdcScale = 5./1024.;
                constexpr float kAdcCalibration = 4.06/4.1845703125; 
                return system_adc_read() * (kAdcScale * kAdcCalibration);
            }
        }

        static void PrintDetails() {

            Serial.printf(
                "ESP Status: {\n"
                "\tChip ID: %d\n"  
                "\tVcc: %f V\n"
                "\tFree Heap: %d bytes\n"
                "\tFlash ID: %d\n"   
                "\tFlash Size: %d bytes\n"
                "\tFlash Real Size: %d bytes\n"
                "\tFlash Speed: %d Hz\n"
                "\tFlash Mode: %s\n"
                "}\n",

                Esp::getChipId(),
                Esp::getVcc(),
                Esp::getFreeHeap(),
                Esp::getFlashChipId(),
                Esp::getFlashChipSize(),
                Esp::getFlashChipRealSize(),
                Esp::getFlashChipSpeed(),
                FlashModeString(Esp::getFlashChipMode())
            );
        }
};

// Define function used to configure ESP at startup
ADC_MODE(Esp::kAdcMode);

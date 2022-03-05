#pragma once

#include <Esp.h>
#include <SoftwareSerial.h>

//Note: Required for ESP.getVCC()
ADC_MODE(ADC_VCC);

class Esp: public EspClass {
    public:

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

        static void PrintDetails() {

            Serial.printf(
                "ESP Status: {\n"
                "\tChip ID: %d\n"  
                "\tVCC: %d mV\n"
                "\tFree Heap: %d bytes\n"
                "\tFlash ID: %d\n"   
                "\tFlash Size: %d bytes\n"
                "\tFlash Real Size: %d bytes\n"
                "\tFlash Speed: %d Hz\n"
                "\tFlash Mode: %s\n"
                "}\n",

                ESP.getChipId(),
                ESP.getVcc(),
                ESP.getFreeHeap(),
                ESP.getFlashChipId(),
                ESP.getFlashChipSize(),
                ESP.getFlashChipRealSize(),
                ESP.getFlashChipSpeed(),
                FlashModeString(ESP.getFlashChipMode())
            );
        }

    

};

#include <SPI.h>
#include <Adafruit_SSD1331.h>
#include <Adafruit_GFX.h>

class Display: public Adafruit_SSD1331 {

  static inline constexpr uint8_t kSCL  = D5;
  static inline constexpr uint8_t kSDA  = D7; 
  static inline constexpr uint8_t kRES  = D3;
  static inline constexpr uint8_t kDC   = D0;
  static inline constexpr uint8_t kCS   = D8;

  static constexpr bool ValidDigitalOutPin(uint8_t pin) {
  
     //TODO: make sure there are all valid
    return pin == D0 ||
           pin == D8 ||
           pin == D1 ||
           pin == D2 ||
           pin == D3 ||
           pin == D4;
  }

  public:  
  
    enum Color: uint16_t {
        kColorBlack        = 0x0000,
        kColorBlue         = 0x001F,
        kColorRed          = 0xF800,
        kColorGreen        = 0x07E0,
        kColorCyan         = 0x07FF,
        kColorMagenta      = 0xF81F,
        kColorYellow       = 0xFFE0,
        kColorWhite        = 0xFFFF,
    };

    GFXcanvas16 backBuffer = GFXcanvas16(TFTWIDTH, TFTHEIGHT);
  
    Display(): Adafruit_SSD1331(&SPI, kCS, kDC, kRES) {
  
        static_assert(kSCL == D5, "SCL must be connected to D5 for fast SPI");
        static_assert(kSDA == D7, "SDA must be connected to D5 for fast SPI");
        
        static_assert(ValidDigitalOutPin(kRES), "kRES is not a valid digital output");
        static_assert(ValidDigitalOutPin(kDC),  "kDC is not a valid digital output");
        static_assert(ValidDigitalOutPin(kCS),  "kCS is not a valid digital output");
    }

    inline void Init() {
      
        // initialise the SSD1331 with 80MHz spi bus fastested it goes.
        //This gives us a max refresh speed of ~85Hz, but isn't as stable
        // begin(80000000);

        // initialise the SSD1331 with 80MHz spi bus fastested it goes.
        begin(40000000);        

        //Note: we need to wait 100ms after on command is sent to the display 
        // at the end of begin() before we can send the display commands. Otherwise
        // we can get undefined behavior
        delay(100);

        //clear out garbage screen content
        fillScreen(kColorBlack);
        enableDisplay(true);
    }

    inline void Clear() {
      fillScreen(kColorBlack);
      setCursor(0, 0);
    }
    
    inline void Draw() {
      drawRGBBitmap(0, 0, backBuffer.getBuffer(), TFTWIDTH, TFTHEIGHT);
    }
    
};

static Display display;

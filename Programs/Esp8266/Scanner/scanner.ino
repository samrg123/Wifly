
#include "util.h"

#include "Wifi.h"
#include "WifiServer.h"
#include "Display.h"
#include "Imu.h"
#include "Timer.h"

constexpr uint kBaudRate = 115200;

constexpr const char* const kSSID = "Shay";
constexpr const char* const kWifiPassword = "Shay2012";

// constexpr const char* const kSSID = "WiflyHub";
// constexpr const char* const kWifiPassword = "thewifly";

constexpr uint16 kServerPort = 1234;


// #include <unordered_map>
// using Command = void(*)(Connection& connection, const char* command, uint8 numArgs, const char** args);
// using CommandMap = unordered_map<const char*, Command>; 

//Note: Required for ESP.getVCC()
ADC_MODE(ADC_VCC);

const char* FlashModeString(FlashMode_t mode) {
  switch(mode) {
    case FM_QIO:      return "QUAD Input/Output";
    case FM_QOUT:     return "QUAD Output";
    case FM_DIO:      return "Dual Input/Output";
    case FM_DOUT:     return "Dual Output";
    case FM_UNKNOWN:  return "Unknown";
    default:          return "INVALID";
  }
}

void PrintEspDetails() {
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

void setup() {

  pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, LOW); // Turn the LED on (Note that LOW is the voltage level but actually the LED is on; this is because it is active low on the ESP-01
  
  display.Init();
  delay(10);
  
  display.println("Connecting UART\n");

  Serial.begin(kBaudRate);
  while(!Serial) {}
  Serial.println(); // print a new line for first to sperate serial garbage from first output line

  PrintEspDetails();

  display.println("Connecting IMU\n");
  imu.Init();

  display.printf("Connecting WIFI:\n'%s'\n\n", kSSID);
  wifi.Connect(kSSID, kWifiPassword);

  //TODO: Implement scrolling text instead of manually having to clear the screen
  display.Clear();
  
  display.printf("Starting Server:\n"
                 "%s\n"
                 "Port:%d\n", 

                 wifi.localIP().toString().c_str(), 
                 kServerPort);
  
  wifiServer.Init(kServerPort, [](WifiServer::Connection& connection){

    connection.client.printf("Hello Client: %s:%d | Connected to %s:%d\n", 
                             connection.client.remoteIP().toString().c_str(), 
                             connection.client.remotePort(),
                             connection.client.localIP().toString().c_str(), 
                             connection.client.localPort()
                             );

    connection.onReadCallback = [](WifiServer::Connection& connection, size_t numBytes) {

      Serial.printf(
        "Client %s:%d says (%d): '%.*s'\n", 
        connection.client.remoteIP().toString().c_str(), 
        connection.client.remotePort(),                              
        numBytes,
        numBytes, &connection.buffer.front()
      );
      Serial.flush();

      connection.client.printf(
        "Client %s:%d says (%d): '%.*s'\n", 
        connection.client.remoteIP().toString().c_str(), 
        connection.client.remotePort(),                              
        numBytes,
        numBytes, &connection.buffer.front()
      );

      connection.buffer.clear();
    };

  });

  delay(10000);

  display.println("Done!");
}

void loop() {

  static Timer loopTimer;

  //capture Vcc
  uint16_t vccMv = ESP.getVcc();

  //capture ellapsed time
  unsigned long deltaUs = loopTimer.LapUs();

  //capture RSS
  int rss = wifi.RSSI();
  
  //capture IMU data
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);
  
  // //Print sensor vals to console so we can use them with plotter  
  // Serial.printf(
  //   "RSS:%d, "
  //   "Acceleration.x:%f, Acceleration.y:%f, Acceleration.z:%f, "
  //   "Rotation.x:%f, Rotation.y:%f, Rotation.z:%f, "
  //   "Temperature(C):%f\n",
  //   rss,
  //   a.acceleration.x, a.acceleration.y, a.acceleration.z,
  //   g.gyro.x, g.gyro.y, g.gyro.z,
  //   temp.temperature
  // );


  wifiServer.Update();


  //Print sensor vals to the display
  display.backBuffer.fillScreen(display.kColorBlack);
  display.backBuffer.setCursor(0,0);

  display.backBuffer.setTextColor(display.kColorWhite);
  display.backBuffer.printf("Vcc: %4.2f\n", .001f * vccMv);

  display.backBuffer.setTextColor(display.kColorRed);
  display.backBuffer.printf("FPS: %5.2f\n", 1000000.f / deltaUs);

  display.backBuffer.setTextColor(display.kColorCyan);
  display.backBuffer.printf("RSSI: %d\n", rss);

  display.backBuffer.setTextColor(display.kColorYellow);
  display.backBuffer.printf("G: % 04.2f % 04.2f \n"
                            "      % 04.2f\n", 
                            g.gyro.x, g.gyro.y, g.gyro.z);

  display.backBuffer.setTextColor(display.kColorGreen);
  display.backBuffer.printf("A: % 04.2f % 04.2f \n"
                            "      % 04.2f\n", 
                            a.acceleration.x, a.acceleration.y, a.acceleration.z);
  
  display.backBuffer.setTextColor(display.kColorMagenta);
  display.backBuffer.printf("T: %5.2f\n", temp.temperature);

  display.Draw();

//  delay(100);
}

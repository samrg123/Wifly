#pragma once

#include "util.h"
#include "Timer.h"
#include "WiflyCommands.h"

#include "Esp.h"
#include "Imu.h"
#include "Wifi.h"
#include "Display.h"

class Wifly {
    public:

        static inline constexpr uint kBaudRate = 115200;

        static inline constexpr const char* const kSSID = "Shay";
        static inline constexpr const char* const kWifiPassword = "Shay2012";

        // static inline constexpr const char* const kSSID = "WiflyHub";
        // static inline constexpr const char* const kWifiPassword = "thewifly";

        static inline constexpr uint16 kServerPort = 1234;

        protected:
        
            Esp esp;
            Imu imu;
            Wifi wifi;
            Display display;

            WifiCommandServer wifiCommandServer = WifiCommandServer(WiflyCommands::kCommandMap.values, WiflyCommands::kCommandMap.values + ArrayCount(WiflyCommands::kCommandMap.values));

    public:

        void Init() {
            pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
            digitalWrite(LED_BUILTIN, LOW); // Turn the LED on (Note that LOW is the voltage level but actually the LED is on; this is because it is active low on the ESP-01
            
            display.Init();
            delay(10);
            
            display.println("Connecting UART\n");

            Serial.begin(kBaudRate);
            while(!Serial) {}
            Serial.println(); // print a new line for first to sperate serial garbage from first output line

            esp.PrintDetails();

            display.println("Connecting IMU\n");
            imu.Init();

            display.printf("Connecting WIFI:\n'%s'\n\n", kSSID);
            wifi.Connect(kSSID, kWifiPassword);

            //TODO: Implement scrolling text instead of manually having to clear the screen
            display.Clear();
            
            display.printf(
                "Starting Server:\n"
                "%s\n"
                "Port:%d\n", 

                wifi.localIP().toString().c_str(), 
                kServerPort
            );
            
            wifiCommandServer.Init(kServerPort, [](WifiServer& server, WifiServer::Connection& connection) {

                Serial.printf("Connected Client: %s:%d\n", 
                            connection.client.remoteIP().toString().c_str(), 
                            connection.client.remotePort());

                connection.onRead.Append([](WifiServer::Connection::OnReadArgs& args) {

                WifiServer::Connection& connection = args.connection;
                size_t newBytes = args.bytesRead;
                uint8* newData = connection.buffer.data() + args.bytesRead;
                
                Serial.printf(
                    "Client %s:%d says (%d): '%.*s'\n", 
                    connection.client.remoteIP().toString().c_str(), 
                    connection.client.remotePort(),                              
                    newBytes,
                    newBytes, newData
                );

                });

            });

            // Wait 10s so port number is available on screen
            delay(10000);
        }

        void Update() {

            static Timer loopTimer;

            //capture Vcc
            uint16 vccMv = esp.getVcc();

            //capture elapsed time
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


            wifiCommandServer.Update();


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

};

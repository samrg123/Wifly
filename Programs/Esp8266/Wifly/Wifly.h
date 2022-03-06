#pragma once

#include "util.h"
#include "Timer.h"
#include "WiflyCommandServer.h"

#include "Esp.h"
#include "Imu.h"
#include "Wifi.h"
#include "Display.h"

class Wifly {
    public:

        static inline constexpr uint kBaudRate = 115200;

        // static inline constexpr const char* const kSSID = "Shay";
        // static inline constexpr const char* const kWifiPassword = "Shay2012";

        static inline constexpr const char* const kSSID = "WiflyHub";
        static inline constexpr const char* const kWifiPassword = "thewifly";

        static inline IPAddress kDefaultIp         = IPAddress(192, 168, 43, 140);
        static inline IPAddress kDefaultGateway    = IPAddress(192, 168, 43, 100);
        static inline IPAddress kDefaultSubnetMask = IPAddress(255, 255, 255, 0);


        static inline constexpr uint16 kServerPort = 1234;

        struct SensorData {
            uint64 timestampUs;

            sensors_vec_t linearAcceleration;
            sensors_vec_t angularAcceleration;
            float temperature;
            
            uint16 vccMv;
            int8 rssi;
        };

    protected:
    
        Esp esp;
        Imu imu;
        Wifi wifi;
        Display display;

        WiflyCommandServer commandServer;
        SensorData currentSensorData = {};

    public:

        using DataStreamCallback = void(*)(Wifly& wifly, const SensorData& sensorData, uint64 deltaUs);
        ChainedCallback<DataStreamCallback> updateDataStreams;

        //Print sensor vals to console so we can use them with plotter  
        static inline void UpdateSerialDataStream(Wifly& wifly, const SensorData& sensorData, uint64 deltaUs) {
            Serial.printf(
                "deltaUs:%llu, "
                "Acceleration.x:%f, Acceleration.y:%f, Acceleration.z:%f, "
                "Rotation.x:%f, Rotation.y:%f, Rotation.z:%f, "
                "Temperature(C):%f\n"
                "Vcc(mV):%d, "
                "RSS:%d ",

                deltaUs,
                sensorData.linearAcceleration.x, sensorData.linearAcceleration.y, sensorData.linearAcceleration.z,
                sensorData.angularAcceleration.x, sensorData.angularAcceleration.y, sensorData.angularAcceleration.z,
                sensorData.temperature,
                sensorData.vccMv,
                sensorData.rssi
            );
        }

        //Print Sensor vals to screen
        static inline void UpdateDisplayDataStream(Wifly& wifly, const SensorData& sensorData, uint64 deltaUs) {
            
            Display& display = wifly.display;
           
            display.backBuffer.fillScreen(display.kColorBlack);
            display.backBuffer.setCursor(0,0);

            display.backBuffer.setTextColor(display.kColorWhite);
            display.backBuffer.printf("Vcc: %4.2f\n", .001f * sensorData.vccMv);

            display.backBuffer.setTextColor(display.kColorRed);
            display.backBuffer.printf("FPS: %5.2f\n", 1000000.f / deltaUs);

            display.backBuffer.setTextColor(display.kColorCyan);
            display.backBuffer.printf("RSSI: %d\n", sensorData.rssi);

            display.backBuffer.setTextColor(display.kColorYellow);
            display.backBuffer.printf(
                "G: % 04.2f % 04.2f \n"
                "      % 04.2f\n", 
                sensorData.angularAcceleration.x, sensorData.angularAcceleration.y, sensorData.angularAcceleration.z
            );

            display.backBuffer.setTextColor(display.kColorGreen);
            display.backBuffer.printf(
                "A: % 04.2f % 04.2f \n"
                "      % 04.2f\n", 
                sensorData.linearAcceleration.x, sensorData.linearAcceleration.y, sensorData.linearAcceleration.z
            );
            
            display.backBuffer.setTextColor(display.kColorMagenta);
            display.backBuffer.printf("T: %5.2f\n", sensorData.temperature);

            display.Draw();
        }


    public:

        inline SensorData ReadSensors() {

            uint64 timestampUs = micros64();

            sensors_event_t a, g, temp;
            imu.getEvent(&a, &g, &temp);

            return SensorData {
                .timestampUs = timestampUs,
                .linearAcceleration = a.acceleration,
                .angularAcceleration = g.gyro,
                .temperature = temp.temperature,
                .vccMv = esp.getVcc(),
                .rssi = wifi.RSSI() 
            };
        }

        void Init() {

            pinMode(LED_BUILTIN, OUTPUT); // Initialize the LED_BUILTIN pin as an output
            digitalWrite(LED_BUILTIN, LOW); // Turn the LED on. Note: LED is active low
            
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

            if (!wifi.config(kDefaultIp, kDefaultGateway, kDefaultSubnetMask)) {
                Serial.printf(
                    "Failed to configure Wifi for: {\n"
                    "\tIp: %s\n"
                    "\tgateway: %s\n"
                    "\tSubnet Mask: %s\n"
                    "}\n",
                    kDefaultIp.toString().c_str(),
                    kDefaultGateway.toString().c_str(),
                    kDefaultSubnetMask.toString().c_str()
                );
            }
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
            
            commandServer.Init(kServerPort, [](WifiServer& server, WifiServer::Connection& connection) {
                
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

            // Note: Serial data stream is off by default
            updateDataStreams.Append(UpdateDisplayDataStream);
        }

        void Update() {

            commandServer.Update();

            SensorData sensorData = ReadSensors();

            updateDataStreams(*this, sensorData, sensorData.timestampUs - currentSensorData.timestampUs);

            currentSensorData = sensorData;
        }

};

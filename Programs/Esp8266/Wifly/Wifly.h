#pragma once

#include "util.h"
#include "Timer.h"
#include "ExtendedWifiCommandServer.h"

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

            template<typename ValueT>
            static inline String ValueString(const void* valuePtr_) {

                if(!valuePtr_) return String("NULL");

                const ValueT* valuePtr = reinterpret_cast<const ValueT*>(valuePtr_);

                if constexpr(std::is_same<ValueT, sensors_vec_t>::value) {
                    
                    return String("{ ") + valuePtr->x + ", " + valuePtr->y + ", " + valuePtr->z + " }";
                    
                } else {

                    return String(*valuePtr);
                }
            }
            
            String SensorValueString(String sensorName) const {

                struct SensorNameLutEntry {
                    const char* name;
                    size_t valueByteOffset;
                    String (*valueString)(const void* sensorValue);
                };

                static constexpr SensorNameLutEntry kSensorNameLut[] = {
                    {"time",  offsetof(SensorData, timestampUs),         ValueString<decltype(timestampUs)>},
                    {"accel", offsetof(SensorData, linearAcceleration),  ValueString<decltype(linearAcceleration)>},
                    {"gyro",  offsetof(SensorData, angularAcceleration), ValueString<decltype(angularAcceleration)>},
                    {"temp",  offsetof(SensorData, temperature),         ValueString<decltype(temperature)>},
                    {"rssi",  offsetof(SensorData, rssi),                ValueString<decltype(rssi)>},
                    {"vcc",   offsetof(SensorData, vccMv),               ValueString<decltype(vccMv)>}
                };

                for(const SensorNameLutEntry& lutEntry : kSensorNameLut) {
                    
                    if(sensorName == lutEntry.name) {
                        const void* sensorValuePtr = ByteOffset(this, lutEntry.valueByteOffset);
                        return lutEntry.valueString(sensorValuePtr);
                    }
                }

                return "NULL";
            }
        };

    protected:
    
        Esp esp;
        Imu imu;
        Wifi wifi;
        Display display;
        SensorData currentSensorData = {};

        struct WiflyCommandServer: public ExtendedWifiCommandServer {
            Wifly& wifly;

            static inline constexpr WiflyCommandServer& Server(const Command& command) {
                return static_cast<WiflyCommandServer&>(command.server);
            }

            static inline constexpr ExtendedCommand kExtendedCommands[] = {
         
                ExtendedCommand {
                    
                    .name = "hello",
                    .description = "Says Hello to the client",
                    .callback = [](Command command) { 

                        Connection& connection = command.connection;
                        connection.client.printf("Hello %s:%d\n", connection.client.remoteIP().toString().c_str(), connection.client.remotePort()); 
                    }
                },

                // TODO: Add help Command and stream 'type' 'on/off' command 
                // TODO: add Wifi::OnDisconnect(server& connection&) callback. Needed to free resources when streaming to wifi client

                ExtendedCommand {
                    .name = "read",
                    .description = "Returns the last read value of each sensor specified in the arguments.",
                    .detailedHelp = "Supported sensors: {\n"
                                    "\ttime  - Returns the timestamp of the last read sensor value in microseconds.\n"
                                    "\taccel - Returns the linear acceleration in g's.\n"
                                    "\tgyro  - Returns the angular acceleration in r/s^2.\n"
                                    "\ttemp  - Returns the temperature of the IMU in degrees C.\n"
                                    "\trssi  - Returns the RSSI value of the Wifi module.\n"
                                    "\tvcc   - Returns the VCC voltage of the ESP8266 in mV.\n"
                                    "}",

                    .args = ExtendedCommand::Args((ExtendedCommand::Arg[]){
                        { .name = "SensorName..." }
                    }),

                    .callback = [](Command command) {          
                    
                        WiflyCommandServer& server = Server(command);
                        Wifly& wifly = server.wifly;

                        String result;
                        for(int i = 0; i < command.numArgs; ++i) {
                            
                            String sensorName = command.args[i];
                            result+= sensorName + ": " + wifly.currentSensorData.SensorValueString(sensorName);
                            result+= '\n';
                        }
                        
                        command.connection.client.print(result);
                    }
                },
            };        

            WiflyCommandServer(Wifly& wifly)
                : ExtendedWifiCommandServer(ExtendedCommandMap(kExtendedCommands)), 
                  wifly(wifly) {

                onConnect.Append([](WifiServer& server, Connection& connection) {
                
                    Log("Connected Client: %s:%d\n", 
                        connection.client.remoteIP().toString().c_str(), connection.client.remotePort());
                    
                    connection.client.printf(
                        "Greetings %s:%d | Connected to: %s:%d",
                        connection.client.remoteIP().toString().c_str(), connection.client.remotePort(),
                        connection.client.localIP().toString().c_str(), connection.client.localPort()
                    );
                });
            }
        };

        WiflyCommandServer commandServer;

    public:

        Wifly(): commandServer(*this) {}

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
            
            commandServer.Init(kServerPort);

            // Wait 5s so port number is available on screen
            delay(5000);

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

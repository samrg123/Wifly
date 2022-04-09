#pragma once

#include "util.h"
#include "Timer.h"
#include "Banner.h"
#include "ExtendedWifiCommandServer.h"

#include "Esp.h"
#include "Imu.h"
#include "Wifi.h"
#include "Display.h"

#include "woof.h"

#include <StreamString.h>

class Wifly {
    public:

        static inline constexpr uint kBaudRate = 115200;

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
            
            float vcc;
            int8 rssi;

            template<typename ValueT>
            static inline String ValueString(const void* valuePtr_) {

                if(!valuePtr_) return String("NULL");

                const ValueT* valuePtr = reinterpret_cast<const ValueT*>(valuePtr_);

                if constexpr(std::is_same<ValueT, float>::value) {

                    String result;
                    
                    S2Stream stream(result);

                    float value = *valuePtr;
                    stream.printf("%.7g", value);
                    
                    return result;

                } else if constexpr(std::is_same<ValueT, sensors_vec_t>::value) {
                    
                    String result;
                    
                    S2Stream stream(result);
                    stream.printf("{ %.7g, %.7g, %.7g }", 
                                  valuePtr->x, valuePtr->y, valuePtr->z);
                    
                    return result;
                    
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
                    {"vcc",   offsetof(SensorData, vcc),                 ValueString<decltype(vcc)>}
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

        bool streamData = false;

        using SensorStreamCallback = void(*)(Wifly& wifly, const SensorData& sensorData, uint64 deltaUs);                
        struct SensorStream: ChainedCallback<SensorStreamCallback> {

            //Print sensor vals to console so we can use them with plotter  
            static inline void UpdateSerial(Wifly& wifly, const SensorData& sensorData, uint64 deltaUs) {
                Serial.printf(
                    "deltaUs:%llu, "
                    "Acceleration.x:%f, Acceleration.y:%f, Acceleration.z:%f, "
                    "Rotation.x:%f, Rotation.y:%f, Rotation.z:%f, "
                    "Temperature(C):%f\n"
                    "Vcc(V):%f, "
                    "RSS:%d ",

                    deltaUs,
                    sensorData.linearAcceleration.x, sensorData.linearAcceleration.y, sensorData.linearAcceleration.z,
                    sensorData.angularAcceleration.x, sensorData.angularAcceleration.y, sensorData.angularAcceleration.z,
                    sensorData.temperature,
                    sensorData.vcc,
                    sensorData.rssi
                );
            }

            //Print Sensor vals to screen
            static inline void UpdateDisplay(Wifly& wifly, const SensorData& sensorData, uint64 deltaUs) {
                
                Display& display = wifly.display;

                display.backBuffer.fillScreen(display.kColorBlack);
                display.backBuffer.setCursor(0,0);

                display.backBuffer.setTextColor(display.kColorWhite);
                display.backBuffer.printf("Vcc: %4.2f\n", sensorData.vcc);

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

            struct CallbackLutEntry {
                const char* name;
                const SensorStreamCallback callback;
            };

            static inline constexpr CallbackLutEntry kCallbackLut[] = {
                {"display", UpdateDisplay},
                {"serial",  UpdateSerial},
            };

            SensorStreamCallback GetCallback(const char* name) {
                for(const CallbackLutEntry& entry : kCallbackLut) {
                    if(!strcmp(name, entry.name)) return entry.callback;
                }
                return nullptr;
            }

            SensorStream(): ChainedCallback({UpdateDisplay}) {}
        };
        SensorStream sensorStream;


        struct WiflyCommandServer: public ExtendedWifiCommandServer {
            Wifly& wifly;

            static inline constexpr WiflyCommandServer& Server(const Command& command) {
                return static_cast<WiflyCommandServer&>(command.server);
            }

            // Note: HelpCallback uses kExtendedCommands array for info so we forward declare it
            // TODO: Forward declare all Callbacks and implement them in a cpp file. Will cleanup
            //       header and allow us to print extended help when a command is specified with invalid args
            static inline void HelpCallback(Command command);

            // Note: For whatever reason Intelisense doesn't reliably evalute this expression as constexpr 
            //       so we just define it as const when Intelisense is compiling project so we don't get false errors
            // TODO: This leads to other issues such as constexpr auto a = kExtendedCommands no longer being valid in 
            //       intelisense, figure out why intelisense is reporting this incorrectly and remove INTELISENSE_CHOOSE macro
            // TODO: Add help Command and stream 'type' 'on/off' command 
            // TODO: add Wifi::OnDisconnect(server& connection&) callback. Needed to free resources when streaming to wifi client   
            static inline INTELISENSE_CHOOSE(const, constexpr) ExtendedCommand kExtendedCommands[] = {

                ExtendedCommand {
                    
                    .name = "hello",
                    .summary = "Says hello to the client.",
                    .detailedHelp = "Sends the client 'Hello ip:port' where 'ip:port' corresponds to the client's ip and port numbers.",
                    .callback = [](Command command) { 

                        Connection& connection = command.connection;
                        connection.client.printf("Hello %s:%d\n", connection.client.remoteIP().toString().c_str(), connection.client.remotePort()); 
                    }
                },   

                ExtendedCommand {
                    
                    .name = "help",
                    .summary = "Prints the help menu. If arguments are provided detailed help is printed for each specified command.",                 

                    .args = (ExtendedCommand::Arg[]) {
                        { .name = "Command...", .type = ExtendedCommand::Arg::TYPE_OPTIONAL }
                    },

                    .callback = HelpCallback
                },

                ExtendedCommand {
                    .name = "read",
                    .summary = "Returns the last read value of each sensor specified in the arguments.",
                    .detailedHelp = "Supported sensors: {\n"
                                    "\ttime  - Returns the timestamp of the last read sensor value in microseconds.\n"
                                    "\taccel - Returns the linear acceleration in g's.\n"
                                    "\tgyro  - Returns the angular acceleration in r/s^2.\n"
                                    "\ttemp  - Returns the temperature of the IMU in degrees C.\n"
                                    "\trssi  - Returns the RSSI value of the Wifi module.\n"
                                    "\tvcc   - Returns the VCC voltage of the ESP8266 in mV.\n"
                                    "}",

                    .args = (ExtendedCommand::Arg[]) {
                        { .name = "SensorName..." } //TODO: make this optional. If no args are provided we should retrurn all sensor values
                    },

                    .callback = [](Command command) {          
                    
                        const Wifly& wifly = Server(command).wifly;

                        String result;
                        for(int i = 0; i < command.numArgs; ++i) {
                            
                            String sensorName = command.args[i];
                            result = result + sensorName + ": " + wifly.currentSensorData.SensorValueString(sensorName) + '\n';
                        }

                        command.connection.client.println(result);
                    }
                },

                ExtendedCommand {
                    .name = "stream",
                    .summary = "Starts or stops the provided sensor stream.",
                    .detailedHelp = "Supported Streams: {\n"
                                    "\tdisplay - Controls updates to the screen\n"
                                    "\tserial  - Controls updates to the serial port\n"
                                    "\twifi    - Controls updates to connected wifi clients\n"
                                    "}",
                    
                    .args = (ExtendedCommand::Arg[]) {
                        { .name = "stream" },
                        { .name = "on|off"}
                    },
                    
                    .callback = [](Command command) {
                        
                        // TODO: add arg parser
                        
                        Client& client = command.connection.client; 
                        if(command.numArgs != 2) {
                            client.println("Invalid args");
                            return;
                        }

                        Wifly& wifly = Server(command).wifly;
                        
                        const char* streamName = command.args[0];

                        enum State { OFF, ON, INVALID };

                        // TODO: cleanup - Hack to turn on wifi broadcast ... should really only be for connected client
                        if(String("wifi") == streamName) {
                            const char* stateStr = command.args[1];
                            State state = !strcmp(stateStr, "on") ? ON : !strcmp(stateStr, "off") ? OFF : INVALID;

                            if(state == INVALID) {
                                client.printf("Invalid state: '%s'\n", stateStr);
                                return;
                            }

                            if(wifly.streamData == state) {
                                client.printf("Sensor stream is already %s\n", stateStr);
                                return;
                            }                                         
                            
                            wifly.streamData = state;
                            return;
                        }


                        SensorStreamCallback callback = wifly.sensorStream.GetCallback(streamName);
                        if(!callback) {
                            client.printf("Invalid stream: '%s'\n", streamName);
                            return;
                        }



                        const char* stateStr = command.args[1];
                        State state = !strcmp(stateStr, "on") ? ON : !strcmp(stateStr, "off") ? OFF : INVALID;

                        if(state == INVALID) {
                            client.printf("Invalid state: '%s'\n", stateStr);
                            return;
                        }

                        auto callbackIt = wifly.sensorStream.Find(callback);
                        bool isStreamOn = callbackIt != wifly.sensorStream.callbacks.end();
                        if(isStreamOn == state) {
                            client.printf("Sensor stream is already %s\n", stateStr);
                            return;
                        }

                        if(state == ON) {
                            wifly.sensorStream.Append(callback);
                        } else {
                            wifly.sensorStream.Remove(callbackIt);
                        }

                        client.printf("Turned sensor stream '%s' %s\n", streamName, stateStr);
                    }
                },

                ExtendedCommand {
                    .name = "woof",
                    .summary = "A royal how do you do.",
                    .detailedHelp = "In loving memory of Shay 2008-2021.",
                    .callback = [](Command command) {
                        
                        Wifly& wifly = Server(command).wifly;
                        wifly.sensorStream.Remove(SensorStream::UpdateDisplay);

                        Display& display = wifly.display;

                        display.backBuffer.drawRGBBitmap(0, 0, Woof::kBuffer, Woof::kWidth, Woof::kHeight);
                        display.Draw();

                        command.connection.client.println("The Queen says Woof");
                    }
                },
            };        

            WiflyCommandServer(Wifly& wifly)
                : ExtendedWifiCommandServer(ExtendedCommandMap(kExtendedCommands)), 
                  wifly(wifly) {

                onConnect.Append([](WifiServer& server, Connection& connection) {
                
                    Log("Connected Client: %s:%d | connectedClientCount: %d\n", 
                        connection.client.remoteIP().toString().c_str(), connection.client.remotePort(),
                        server.ConnectedClientCount());
                    
                    connection.client.printf(
                        "Greetings %s:%d | Connected to: %s:%d\n",
                        connection.client.remoteIP().toString().c_str(), connection.client.remotePort(),
                        connection.client.localIP().toString().c_str(), connection.client.localPort()
                    );
                });

                onDisconnect.Append([](WifiServer& server, Connection& connection) {
                    Log("Disconnected Client: %s:%d\n", 
                        connection.client.remoteIP().toString().c_str(), connection.client.remotePort());
                });
            }
        };

        WiflyCommandServer commandServer;

    public:

        Wifly(): commandServer(*this) {}

        inline SensorData ReadSensors() {

            uint64 timestampUs = micros64();

            sensors_event_t a, g, temp;
            imu.getEvent(&a, &g, &temp);

            return SensorData {
                .timestampUs = timestampUs,
                .linearAcceleration = a.acceleration,
                .angularAcceleration = g.gyro,
                .temperature = temp.temperature,
                .vcc = esp.getVcc(),
                .rssi = wifi.RSSI() 
            };
        }

        // TODO: Once we configure IMU to trigger interrupt when data is ready
        //       switch from pulling SensorData in Update loop to capturing it here.
        static IRAM_FUNC void ImuInterrupt(void* wifly_) {
            Wifly* wifly = static_cast<Wifly*>(wifly_);
            Serial.println("IMU INTERRUPT CALLED");
        }

        void Init() {

            // Turn on power LED
            // Note: LED is active low
            pinMode(LED_BUILTIN, OUTPUT);
            digitalWrite(LED_BUILTIN, LOW);

            //Init Display.    
            display.Init();
            
            // Init Serial
            display.println("Connecting UART\n");
            Serial.begin(kBaudRate);
            while(!Serial) {}
            Serial.println(); // print a new line for first to sperate serial garbage from first output line

            // Print ESP details to serial
            esp.PrintDetails();

            // Init IMU
            display.println("Connecting IMU\n");
            // imu.Init(MPU6050_RANGE_2_G, MPU6050_RANGE_250_DEG, MPU6050_BAND_260_HZ);
            imu.Init(MPU6050_RANGE_4_G, MPU6050_RANGE_250_DEG, MPU6050_BAND_260_HZ);
            
            // Connect to wifi
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

            // Install Imu interrupt
            // Warn: pinmode for D6 has to be set after display.Init()
            // Note: The display is connected to HSPI and only need to receives data.
            //       Therefore it only connects to HSPI pins D5, D7, D8 and leaves D6 (MISO) unused
            //       Rather than wasting the pin we override the pinmode set by display to be a GPIO input.
            pinMode(D6, INPUT_PULLUP);
            imu.setInterruptPinPolarity(0); //active high
            attachInterruptArg(digitalPinToInterrupt(D6), ImuInterrupt, this, FALLING);

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
        }

        void Update() {

            commandServer.Update();

            SensorData sensorData = ReadSensors();

            // TODO: clean this up. Hack to stream data back to laptop
            if(streamData) {

                constexpr char* values[] = {
                    "time",
                    "accel",
                    "gyro",
                    "rssi",          
                };

                String sensorValueString;
                for(const char* value : values) {
                    sensorValueString = sensorValueString + value + ": " + sensorData.SensorValueString(value) + '\n';
                }
                sensorValueString+= '\n';

                commandServer.Broadcast(sensorValueString.c_str(), sensorValueString.length());
            }

            sensorStream(*this, sensorData, sensorData.timestampUs - currentSensorData.timestampUs);

            currentSensorData = sensorData;
        }
};

void Wifly::WiflyCommandServer::HelpCallback(WiflyCommandServer::Command command) {

    // TODO: pull these funtions out into a text formatter that will automatically
    //       adjust indenting and wrap text to banner width

    auto indentNewlines = [](const char* str) {
        
        // TODO: preallocate the size of result to at least fit str 
        String result;
        if(!str) return result;

        for(char c; (c = *str); ++str) {
            result+= c;
            if(c == '\n') result+= '\t';
        }
        return result;
    };

    auto argsString = [](const ExtendedCommand& extendedCommand) {
        
        const ExtendedCommand::Args& args = extendedCommand.args;
        
        int numArgs = args.numArgs; 
        if(!numArgs) return String("None"); 
        
        String result;
        int i = 0;
        while(true) {

            const ExtendedCommand::Arg& arg = args.arg[i];

            if(arg.type == arg.TYPE_OPTIONAL) {
                result = result + '[' + arg.name + ']';
            } else {
                result = result + '<' + arg.name + '>';
            }
            
            if(++i < numArgs) {
                result+= ' ';
            } else {
                return result;
            }
        }
    };

    auto helpSummaryString = [&](const ExtendedCommand& extendedCommand) {

        String result(extendedCommand.name);
        
        constexpr int kLeftColumnWidth = 8;
        for(int i = result.length(); i < kLeftColumnWidth; ++i) {
            result+= ' ';
        }
        
        return result + ": " + extendedCommand.summary;
    };

    auto detailedHelpString = [&](const ExtendedCommand& extendedCommand) {
        
        String result;
        result = result + extendedCommand.name + " {\n"
                 "\tArgs:    " + argsString(extendedCommand) + "\n"
                 "\tSummary: " + extendedCommand.summary + "\n"
                 "\tDetails: " + (extendedCommand.detailedHelp ? indentNewlines(extendedCommand.detailedHelp) : String("None")) + "\n"
                 "}";
        
        return result;
    };

    String result;
    int numArgs = command.numArgs; 
    if(numArgs) {
    
        // print extended Help
        String helpString;
        for(int i = 0; i < numArgs; ++i) {
            const char* arg = command.args[i];
            
            // TODO: switch to binary search once we can enforce ordering on kExtendedCommands 
            for(const ExtendedCommand& extendedCommand : kExtendedCommands) {
                if(!strcmp(arg, extendedCommand.name)) {
                    helpString = detailedHelpString(extendedCommand);
                    break;
                }
            }
            
            if(helpString.isEmpty()) {
                result = result + "\nInvalid Command: '" + arg + '\'';  
            } else {
                result = result + '\n' + helpString;
                helpString.clear();                         
            }
        }

    } else {

        constexpr auto kBanner = Banner<90>(" Help Menu ", '-');
        result = result + kBanner.buffer + '\n';

        // print Help summary
        for(const ExtendedCommand& extendedCommand : kExtendedCommands) {
            result = result + helpSummaryString(extendedCommand) + '\n';
        }
    }

    Connection& connection = command.connection;
    connection.client.print(result); 
}

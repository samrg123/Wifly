#pragma once

#include "util.h"
#include "WifiCommandServer.h"

struct WiflyCommand {

    struct Arg {
        enum Type { TYPE_REQUIRED, TYPE_OPTIONAL };

        const char* name = nullptr;
        Type type = TYPE_REQUIRED;
    };

    struct Args {
        const Arg* args;
        size_t numArgs;

        constexpr Args(): args(nullptr), numArgs(0) {}

        template<size_t kN>
        constexpr Args(const Arg (&args_)[kN]): args(args_), numArgs(kN) {}
    };

    const char* name         = nullptr;
    const char* description  = nullptr;
    const char* detailedHelp = nullptr;

    Args args;
    
    WifiCommandServer::CommandCallback callback = nullptr;
};

template<size_t kN>
struct WiflyCommandMap {

    std::pair<const char*, WifiCommandServer::CommandCallback> values[kN];

    constexpr WiflyCommandMap(const WiflyCommand (&commands)[kN]): values{} {
        for(int i = 0; i < kN; ++i) {
            values[i].first = commands[i].name;
            values[i].second = commands[i].callback;
        }
    }
};

class WiflyCommandServer: public WifiCommandServer {
    public:

        static inline constexpr WiflyCommand kCommands[] = {

            WiflyCommand {

                .name = "hello",
                .description = "Says Hello to the client",
                .callback = [](WifiCommandServer::Command command) { 
                    
                    WifiServer::Connection& connection = command.connection;
                    connection.client.printf("Hello %s:%d\n", connection.client.remoteIP().toString().c_str(), connection.client.remotePort()); 
                }
            }
        };

        static inline constexpr auto kCommandMap = WiflyCommandMap(kCommands);

        WiflyCommandServer(): 
            WifiCommandServer(kCommandMap.values, 
                              kCommandMap.values + ArrayCount(kCommandMap.values)) {

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




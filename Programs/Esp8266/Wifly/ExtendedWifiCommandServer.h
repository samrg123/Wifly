#pragma once

#include "util.h"
#include "WifiCommandServer.h"

class ExtendedWifiCommandServer: public WifiCommandServer {
    public:

        struct ExtendedCommand {

            struct Arg {
                enum Type { TYPE_REQUIRED, TYPE_OPTIONAL };

                const char* name = nullptr;
                Type type = TYPE_REQUIRED;
            };

            struct Args {
                const Arg* arg;
                size_t numArgs;

                constexpr Args(): arg(nullptr), numArgs(0) {}

                template<size_t kN>
                constexpr Args(const Arg (&args_)[kN]): arg(args_), numArgs(kN) {}
            };

            const char* name         = nullptr;
            const char* summary      = nullptr;
            const char* detailedHelp = nullptr;

            Args args;
            
            CommandCallback callback = nullptr;
        };

    protected:

        template<size_t kN>
        struct ExtendedCommandMap {

            std::pair<const char*, WifiCommandServer::CommandCallback> pairs[kN];

            constexpr ExtendedCommandMap(const ExtendedCommand (&commands)[kN]): pairs{} {
                for(int i = 0; i < kN; ++i) {
                    pairs[i].first = commands[i].name;
                    pairs[i].second = commands[i].callback;
                }
            }
        };

        template<size_t kN>
        ExtendedWifiCommandServer(const ExtendedCommandMap<kN> extendedCommandMap): 
            WifiCommandServer(extendedCommandMap.pairs, 
                              extendedCommandMap.pairs + ArrayCount(extendedCommandMap.pairs)) {}
};




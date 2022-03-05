#pragma once

#include "WifiCommandServer.h"

struct WiflyCommand {

  struct Arg {
    enum Type { TYPE_REQUIRED, TYPE_OPTIONAL };

    const char* name;
    Type type = TYPE_REQUIRED;
  };

  struct Args {
    const Arg* args;
    size_t numArgs;

    constexpr Args(): args(nullptr), numArgs(0) {}

    template<size_t kN>
    constexpr Args(const Arg (&args_)[kN]): args(args_), numArgs(kN) {}
  };

  const char* name;
  const char* description = nullptr;
  const char* detailedHelp = nullptr;

  Args args;
  
  WifiCommandServer::CommandCallback callback;
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

struct WiflyCommands {

  static inline constexpr WiflyCommand kCommands[] = {

    WiflyCommand {

      .name = "hello",
      .description = "Says Hello to the client",
      .callback = [](WifiCommandServer::Command command) { 
        
        WifiServer::Connection& connection = command.connection;
        connection.client.printf("Hello %s:%d\n", connection.client.remoteIP().toString().c_str(), connection.client.remotePort()); 
      },
    }
  };

  static inline constexpr auto kCommandMap = WiflyCommandMap(kCommands);
};

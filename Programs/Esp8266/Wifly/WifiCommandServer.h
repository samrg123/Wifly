#pragma once

#include <string>
#include <unordered_map>

#include "WifiServer.h"

class WifiCommandServer: public WifiServer {
    public:
        
        struct Command {
            WifiCommandServer& server;
            Connection& connection;
            
            std::string name;
            char** args;
            size_t numArgs;
        };
        
        using CommandCallback = void(*)(Command command);
        using CommandMap = std::unordered_map<std::string, CommandCallback>;

        CommandMap commandMap;

    protected:

        static inline bool IsWordSeperator(char c) {
            switch(c) {
                case ' ':
                case '\t': return true;
            }
            return false;
        }

        //Note: this inserts null terminators in after each word in cstr
        static std::vector<char*> ParseWords(char* cstr) {

            //skip leading whitespace
            while(IsWordSeperator(*cstr)) {
                ++cstr;
            }

            //parse words
            char* wordStart = cstr;
            std::vector<char*> words;
            for(char c; (c = *cstr); ++cstr) {
                
                if(IsWordSeperator(c)) {
                    
                    //add word
                    *cstr = '\0';
                    words.push_back(wordStart);

                    //fast forward to next word
                    ++cstr;
                    while(IsWordSeperator(*cstr)) {
                        ++cstr;
                    }
                    wordStart = cstr;
                }
            }

            //add last world.
            if(*wordStart) {
                words.push_back(wordStart);
            }

            return words;
        }

        inline bool ExecuteCommand(Command command) {

            auto it = commandMap.find(command.name);
            if(it == commandMap.end()) return false;

            //invoke callback
            it->second(command);
            return true;
        }

        static void ProcessCommand(Connection::OnReadArgs& args) {

            WifiCommandServer& instance = static_cast<WifiCommandServer&>(args.server);

            Connection& connection = args.connection;
            char* buffer = reinterpret_cast<char*>(connection.buffer.data());
            size_t bufferSize = connection.buffer.size(); 
 
            size_t writeOffset = 0;
            for(size_t readIndex = bufferSize - args.bytesRead; 
                readIndex < bufferSize; 
                ++readIndex) {

                switch(buffer[readIndex]) {
                    
                    //handle backspace
                    case '\b': {
                        if(writeOffset < readIndex) ++writeOffset;
                    } break;

                    //execute the command in buffer
                    case '\n': {
                    
                        buffer[readIndex - writeOffset] = '\0';
                        std::vector<char*> commandWords = ParseWords(buffer);
                        
                        bool commandExecuted = instance.ExecuteCommand(Command {
                            .server = instance,
                            .connection = connection,
                            .name = commandWords.front(),
                            .args = commandWords.data() + 1,
                            .numArgs = commandWords.size() - 1
                        });
                        
                        
                        if(!commandExecuted) {
                            connection.client.printf("Invalid Command: '%s'\n", commandWords.front());
                        }
                        
                        //start new command
                        writeOffset = readIndex+1;
                        continue;
                    
                    } break;
                }

                // fixup buffer
                buffer[readIndex - writeOffset] = buffer[readIndex];
            }

            //shrink buffer to fit unprocessed data
            //Note: we consumed all new bytes up to and including last newline 
            size_t remainingBytes = bufferSize - writeOffset; 
            args.connection.buffer.resize(remainingBytes);
            args.bytesRead = remainingBytes; 
        }

        inline void AppendOnConnectCallback() {
            onConnect.Append([](WifiServer& server, Connection& connection) {
                connection.onRead.Append(ProcessCommand);
            });            
        }

    public:

        WifiCommandServer() {
            AppendOnConnectCallback();
        }

        WifiCommandServer(const CommandMap& commandMap): commandMap(commandMap) {
            AppendOnConnectCallback();
        }

        template<typename InputItT>
        WifiCommandServer(InputItT begin, InputItT end): commandMap(begin, end) {
            AppendOnConnectCallback();
        }        
};
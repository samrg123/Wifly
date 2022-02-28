#pragma once

#include "util.h"
#include "ChainedCallback.h"

#include <WiFiServer.h>
#include <vector>

class WifiServer: public WiFiServer {

    public:

        static inline constexpr uint8 kMaxBacklog     = 5; //number of connections queued up
        static inline constexpr uint8 kMaxConnections = 5; //number of connections managed by server

        struct Connection {

            using OnReadCallback = void(*)(Connection& connection, size_t bytesRead);
    
            WiFiClient client;
            ChainedCallback<OnReadCallback> onRead;  
            std::vector<uint8> buffer;
        };

        using OnConnectCallback = void(*)(Connection& connection);
        ChainedCallback<OnConnectCallback> onConnect; 

    protected:
        
        Connection connections[kMaxConnections];
        uint8 connectedClientCount = 0;

    public:

        WifiServer(): WiFiServer(0) {}

        uint8 ConnectedClientCount() const { return connectedClientCount; }

        static const char* TcpStateString(wl_tcp_state state) {
            switch(state) {
                case CLOSED:      return "CLOSED";
                case LISTEN:      return "LISTEN";
                case SYN_SENT:    return "SYN_SENT";
                case SYN_RCVD:    return "SYN_RCVD";
                case ESTABLISHED: return "ESTABLISHED";
                case FIN_WAIT_1:  return "FIN_WAIT_1";
                case FIN_WAIT_2:  return "FIN_WAIT_2";
                case CLOSE_WAIT:  return "CLOSE_WAIT";
                case CLOSING:     return "CLOSING";
                case LAST_ACK:    return "LAST_ACK";
                case TIME_WAIT:   return "TIME_WAIT";
                default:          return "INVALID";
            }
        }

        inline wl_tcp_state TcpState() {
            return wl_tcp_state(status());
        }

        // TODO: replace all PrintStatus functions with more generic 'ToString' 
        void PrintStatus() {
            Serial.printf(
                "WifiServer Status {\n"
                "\tIP: %s\n"
                "\tTCP Port: %d\n"
                "\tTCP State: %s\n"
                "\tConnections: %d/%d\n"
                "\tHas Client: %s\n"
                "\tMax Backlog: %d\n"
                "}\n",

                _addr.toString().c_str(),
                _port,
                TcpStateString(TcpState()),
                connectedClientCount,kMaxConnections,
                hasClient() ? "True" : "False",
                kMaxBacklog
            );
        }

        void Init(uint16 port, OnConnectCallback onConnectCallback = nullptr) {

            onConnect.Append(onConnectCallback);

            begin(port, kMaxBacklog);

            PrintStatus();
        }

        void Update() {
            
            connectedClientCount = 0; 
            for(Connection& connection : connections) {

                if(!connection.client) {
                
                    //Try to get new client
                    connection.client = available();
                    connection.buffer.clear();
                    
                    //Invoke onConnect callback
                    // Note: we don't call client.connected just in case connection is still in the 
                    //       initial stages where connected() would return false. Instead we call
                    //       onConnect as soon as connection is initiated
                    if(connection.client.status() != CLOSED) {
                        ++connectedClientCount;
                        onConnect(connection);
                    }
                
                } else if(connection.client.connected()) {

                    ++connectedClientCount;
                }

                //check for data
                size_t availableBytes = connection.client.peekAvailable();
                if(!availableBytes) {
                    continue;
                }

                //allocate the new buffer
                size_t oldBufferSize = connection.buffer.size();
                size_t newBufferBytes = oldBufferSize + availableBytes;
                
                uint32 freeHeapBytes = ESP.getFreeHeap();
                if(newBufferBytes > freeHeapBytes) {

                    Warn("Skipping buffer reallocation of size: %u for connection: %s. Free Heap Bytes: %u", 
                         newBufferBytes,
                         connection.client.remoteIP().toString().c_str(),
                         freeHeapBytes
                    );

                    continue;
                }
                
                // TODO: Even with sanity check above I still don't like using new when exceptions are turned off. 
                //       Think of way to get stdlib to use placement new or no-throw new
                connection.buffer.resize(newBufferBytes);
                
                //read in the buffer
                size_t readBytes = connection.client.read(&connection.buffer[oldBufferSize], availableBytes);
                connection.buffer.resize(oldBufferSize + readBytes);

                //invoke onRead callback
                if(readBytes) connection.onRead(connection, readBytes);
            }
        }
};

WifiServer wifiServer;

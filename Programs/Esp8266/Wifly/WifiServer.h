#pragma once

#include "util.h"
#include "ChainedCallback.h"

#include <WiFiServer.h>
#include <WiFiClient.h>

#include <vector>

class WifiServer: public WiFiServer {

    public:

        static inline constexpr uint8 kMaxBacklog     = 5; //number of connections queued up
        static inline constexpr uint8 kMaxConnections = 5; //number of connections managed by server

        struct Connection {

            struct OnReadArgs;
            using OnReadCallback = void(*)(OnReadArgs& args);
    
            WiFiClient client;
            ChainedCallback<OnReadCallback> onRead;  
            std::vector<uint8> buffer;
        };

        struct Connection::OnReadArgs {
            WifiServer& server;
            Connection& connection;
            size_t bytesRead;
        };


        using ConnectionCallback = void(*)(WifiServer& server, Connection& connection);
        ChainedCallback<ConnectionCallback> onConnect; 
        ChainedCallback<ConnectionCallback> onDisconnect; 

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

        void Init(uint16 port, 
                  ConnectionCallback onConnectCallback = nullptr,
                  ConnectionCallback onDisconnectCallback = nullptr) {

            if(onConnectCallback) onConnect.Append(onConnectCallback);
            if(onDisconnectCallback) onConnect.Append(onDisconnectCallback);

            begin(port, kMaxBacklog);

            PrintStatus();
        }

        void Broadcast(const char* data, size_t bytes) {
            for(Connection& connection : connections) {
                if(connection.client) {
                    connection.client.write(data, bytes);
                }
            }
        }

        void Update() {
            
            for(Connection& connection : connections) {

                if(!connection.client) {
            
                    // Log("ClientStatus: %d | remoteIp: %d:%d | localIp: %d:%d",
                    //     connection.client.status(),
                    //     connection.client.remoteIP(), connection.client.remotePort(),                            
                    //     connection.client.localIP(), connection.client.localPort()                            
                    // );                      

                    // TODO: THIS DOESN'T WORK!!! available always returns a connection 
                    if(connection.client.remoteIP().isSet()) {
                        --connectedClientCount;
                        onDisconnect(*this, connection);
                    }

                    //Try to establish a new connection
                    connection = Connection {
                        .client = available()
                    };
                    
                    //Invoke onConnect callback
                    // Note: we don't call client.connected just in case connection is still in the 
                    //       initial stages where connected() would return false. Instead we call
                    //       onConnect as soon as connection is initiated
                    if(connection.client.status() != CLOSED) {
                        ++connectedClientCount;
                        onConnect(*this, connection);
                    
                    } else {

                        // No available client
                        continue;
                    } 
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
                size_t bytesRead = connection.client.read(&connection.buffer[oldBufferSize], availableBytes);
                connection.buffer.resize(oldBufferSize + bytesRead);

                //invoke onRead callback
                if(bytesRead) {

                    Connection::OnReadArgs args = {
                        .server = *this,
                        .connection = connection,
                        .bytesRead = bytesRead
                    };

                    connection.onRead(args);
                }
            }
        }
};

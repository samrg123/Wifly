#pragma once

#include <ESP8266WiFi.h>

//const char* const kSSID = "Shay";
//const char* const kWifiPassword = "Shay2012";

const char* const kSSID = "WiflyHub";
const char* const kWifiPassword = "thewifly";

const char* WiFiStatusString(wl_status_t status) {
  switch(status) {
    case WL_IDLE_STATUS:      return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL:    return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:   return "WL_SCAN_COMPLETED";
    case WL_CONNECTED:        return "WL_CONNECTED";
    case WL_CONNECT_FAILED:   return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:  return "WL_CONNECTION_LOST";
    case WL_WRONG_PASSWORD:   return "WL_WRONG_PASSWORD";
    case WL_DISCONNECTED:     return "WL_DISCONNECTED";
    case WL_NO_SHIELD:        return "WL_NO_SHIELD";
    default:                  return "INVALID";
  }
}

void PrintWiFiDetails() {
  Serial.printf(
    "WiFi Status: {\n"
    "\tMAC: %s\n"
    "\tStatus: %s\n"
    "\tSSID: %s\n"
    "\tChannel: %d\n"
    "\tRSSI: %d\n"
    "\tGateway: %s\n"
    "\tSubnetMask: %s\n"    
    "\tIP: %s\n"
    "\tDNS: %s\n"    
    "}\n",

    WiFi.macAddress().c_str(),
    WiFiStatusString(WiFi.status()),    
    WiFi.SSID().c_str(),
    WiFi.channel(),
    WiFi.RSSI(),
    WiFi.gatewayIP().toString().c_str(),
    WiFi.subnetMask().toString().c_str(),
    WiFi.localIP().toString().c_str(),    
    WiFi.dnsIP().toString().c_str()
  );
}

void ConnectToWiFi(const char* ssid, const char* password) {

  Serial.printf("Connecting to SSID '%s': \n", ssid);

  WiFi.begin(ssid, password);

  wl_status_t status;
  while((status = WiFi.status()),
         status != WL_CONNECTED &&
         status != WL_NO_SHIELD &&
         status != WL_CONNECT_FAILED &&
         status != WL_WRONG_PASSWORD) {

    Serial.printf("\nFailed to connect. status = %s | Retrying\n", WiFiStatusString(status));          
    delay(500);
  }
  
  Serial.write('\n');

  PrintWiFiDetails();
}

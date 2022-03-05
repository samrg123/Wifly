#pragma once

#include <ESP8266WiFi.h>

class Wifi: public ESP8266WiFiClass {

  public:

    static const char* WiFiStatusString(wl_status_t status) {
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

    void PrintWiFiStatus() {

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

        macAddress().c_str(),
        WiFiStatusString(status()),
        SSID().c_str(),
        channel(),
        RSSI(),
        gatewayIP().toString().c_str(),
        subnetMask().toString().c_str(),
        localIP().toString().c_str(),
        dnsIP().toString().c_str()
      );
    }

    void Connect(const char* ssid, const char* password) {

      Serial.printf("Connecting to SSID '%s': \n", ssid);

      begin(ssid, password);
      wl_status_t wifiStatus;
      while((wifiStatus = status()),
            wifiStatus != WL_CONNECTED &&
            wifiStatus != WL_NO_SHIELD &&
            wifiStatus != WL_CONNECT_FAILED &&
            wifiStatus != WL_WRONG_PASSWORD) {

        Serial.printf("\nFailed to connect. status = %s | Retrying\n", WiFiStatusString(wifiStatus));
        delay(500);
      }

      Serial.write('\n');

      PrintWiFiStatus();
    }

};

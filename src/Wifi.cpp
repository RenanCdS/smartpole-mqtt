#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>

// MESH NETWORK
#include "painlessMesh.h"
#include "../header/Wifi.h"

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient); 
const char *SSID = "";
const char *PWD = "";

Wifi::Wifi(){}

WiFiClient setupWifi() {
    Serial.printf("Connecting...");
    WiFi.begin(SSID, PWD);
    Serial.printf(SSID);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.printf(".");
        delay(500);
    }
    Serial.printf("Connected.");

    return wifiClient;
}
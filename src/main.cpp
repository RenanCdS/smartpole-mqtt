#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient); 
const char *SSID = "<SSID>";
const char *PWD = "<PASSWORD>";
char *mqttServer = "<IP_ADDR>";
int mqttPort = 1883;

void connectToWiFi() {
  Serial.printf("Connecting...");
  WiFi.begin(SSID, PWD);
  Serial.printf(SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(500);
  }
  Serial.printf("Connected.");
}

void setupMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  Serial.print("MQTT configured...");
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  connectToWiFi();
  setupMQTT();
}

int temperatureValue = 10;

void loop() {
  
  if (!mqttClient.connected()) {
    reconnect();
  }

  temperatureValue = temperatureValue + 1;
  if (temperatureValue == 30) {
    temperatureValue = 10;
  }
  char tempString[100];
  sprintf(tempString, "{\"temperature\": %d}", temperatureValue);
  
  Serial.printf("Publishing message...");
  mqttClient.publish("smartpole/temperature", tempString);
  delay(2000);
}
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>

// MESH NETWORK
#include "painlessMesh.h"

#define   MESH_PREFIX     "SMART_POLES_NETWORK"
#define   MESH_PASSWORD   "PASSWORD"
#define   MESH_PORT       5555
#define   GATEWAY_NODE_ID 2224947593
#define   IS_GATEWAY      true

const char *SSID = "";
const char *PWD = "";
char *mqttServer = "192.168.0.172";
int mqttPort = 1883;

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient); 


void sendMessage();

// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

void setupMeshNetwork() {
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
}

void sendMessageToGateway(String message) {
  mesh.sendSingle(GATEWAY_NODE_ID, message);
}

// End MESH NETWORK

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
  // setupMeshNetwork();
}

void sendMessageToBroker(String topic, char *message) {
  if (!mqttClient.connected()) {
    reconnect();
  }
  
  Serial.printf("Publishing message...");
  mqttClient.publish("smartpole/temperature", message);
  delay(2000);
}

int temperatureValue = 10;

void loop() {

  // FAKE DATA!!! Send Real data by sensors
  temperatureValue = temperatureValue + 1;
  if (temperatureValue == 30) {
    temperatureValue = 10;
  }
  char tempString[100];
  sprintf(tempString, "{\"temperature\": %d}", temperatureValue);
  // END

  if (IS_GATEWAY) {
    sendMessageToBroker("smartpole/temperature", tempString);
  } else {

  }

  // mesh.update();
}
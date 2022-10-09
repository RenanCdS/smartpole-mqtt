#include <Arduino.h>
#include <painlessMesh.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <cstring>


#include "painlessMesh.h"

#define IS_ROOT           true
#define ROOT_HOSTNAME     "SmartPoleRoot"
#define NODE_HOSTNAME     "SmartPoleNode1"

#define   MESH_PREFIX     "SMART_POLES_NETWORK"
#define   MESH_PASSWORD   "PASSWORD"
#define   MESH_PORT       5555

#define STATION_SSID      "Ygor 2.4G"
#define STATION_PASSWORD  "Y87644378"

IPAddress getlocalIP();
void mqttCallback(char* topic, byte* payload, unsigned int length);

IPAddress myIP(0,0,0,0);
IPAddress mqttBroker(192, 168, 0, 172);

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;
WiFiClient wifiClient;
PubSubClient mqttClient(mqttBroker, 1883, mqttCallback, wifiClient);

void sendMessage();

void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  if (IS_ROOT) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("ESP32Client")) {
      Serial.println("Sending message from node1...");
      mqttClient.publish("smartpole/temperature", msg.c_str());
      delay(2000);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
    }
  }
  delay(2000);
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
  char* cleanPayload = (char*)malloc(length+1);
  payload[length] = '\0';
  memcpy(cleanPayload, payload, length+1);
  String msg = String(cleanPayload);
  free(cleanPayload);

  String targetStr = String(topic).substring(16);

  if(targetStr == "gateway")
  {
    if(msg == "getNodes")
    {
      auto nodes = mesh.getNodeList(true);
      String str;
      for (auto &&id : nodes)
        str += String(id) + String(" ");
      mqttClient.publish("painlessMesh/from/gateway", str.c_str());
    }
  }
  else if(targetStr == "broadcast") 
  {
    mesh.sendBroadcast(msg);
  }
  else
  {
    uint32_t target = strtoul(targetStr.c_str(), NULL, 10);
    if(mesh.isConnected(target))
    {
      mesh.sendSingle(target, msg);
    }
    else
    {
      mqttClient.publish("painlessMesh/from/gateway", "Client not connected!");
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // MESH NETWORK 
  mesh.setDebugMsgTypes(ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); 
  mesh.onReceive(&receivedCallback);

  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA, 11);
  mesh.initOTAReceive("bridge"); // TODO: Verify if this method made the connection work

  if (IS_ROOT) {
    mesh.stationManual(STATION_SSID, STATION_PASSWORD);
    mesh.setHostname(ROOT_HOSTNAME);
  } else {
    mesh.setHostname(NODE_HOSTNAME);
  }

  mesh.setRoot(IS_ROOT);
  mesh.setContainsRoot(true);
}

void loop() {
  if (IS_ROOT) {
    // mqttClient.loop();

    // Serial.print("Attempting MQTT connection...");

    // // Attempt to connect
    // if (mqttClient.connect("ESP32Client")) {
    //   Serial.println("connected");
    //   mqttClient.publish("smartpole/temperature","Ready!");
    //   delay(2000);
    // } else {
    //   Serial.print("failed, rc=");
    //   Serial.print(mqttClient.state());
    // }
  } else {
    mesh.sendBroadcast("Message from node1");
  }
  mesh.update();
}

IPAddress getlocalIP() {
  return IPAddress(mesh.getStationIP());
}
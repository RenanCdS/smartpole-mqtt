#include <Arduino.h>
#include <painlessMesh.h>
#include <PubSubClient.h>
// #include <WiFi.h>
#include <cstring>


#include "painlessMesh.h"

#define ID_GATEWAY 1370564033

#define IS_ROOT           true
#define ROOT_HOSTNAME     "SmartPoleRoot"
#define NODE_HOSTNAME     "SmartPoleNode1"

#define   MESH_PREFIX     "SMART_POLES_NETWORK"
#define   MESH_PASSWORD   "PASSWORD"
#define   MESH_PORT       5555

#define STATION_SSID      ""
#define STATION_PASSWORD  ""
#define STATION_PORT     5555
uint8_t   station_ip[4] =  {192,168,0,180};


IPAddress getlocalIP();
void mqttCallback(char* topic, byte* payload, unsigned int length);

IPAddress myIP(0,0,0,0);
IPAddress mqttBroker(192, 168, 211, 117);

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;
WiFiClient wifiClient;
PubSubClient mqttClient;

void sendMessage();

void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  if (IS_ROOT) {
    Serial.print("Attempting MQTT connection...");
    Serial.print(WiFi.status());
    if (mqttClient.connect("ESP32Client")) {
      Serial.println("Sending message from node1...");
      mqttClient.publish("smartpole/temperature", msg.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
    }
  }
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

void configureMqtt() 
{
  WiFi.begin(STATION_SSID, STATION_PASSWORD);
  mqttClient.setServer(mqttBroker, 1883);
  mqttClient.setCallback(mqttCallback);  
  mqttClient.setClient(wifiClient);
}

void setup() {
  Serial.begin(115200);
  
  // MESH NETWORK 
  mesh.setDebugMsgTypes(ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); 

  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA, 11);
  //mesh.initOTAReceive("bridge"); // TODO: Verify if this method made the connection work

  if (IS_ROOT) {
    mesh.stationManual(STATION_SSID, STATION_PASSWORD);
    mesh.setHostname(ROOT_HOSTNAME);
    configureMqtt();
  } else {
    mesh.setHostname(NODE_HOSTNAME);
  }

  mesh.setRoot(IS_ROOT);
  mesh.setContainsRoot(true);
  mesh.onReceive(&receivedCallback);
}

void loop() {
  if (IS_ROOT) {
    // mqttClient.loop();

    // Serial.print("Attempting MQTT connection...");

    // // Attempt to connect
    // if (mqttClient.connect("ESP32Client")) {
    //   Serial.println("connected");
    //   mqttClient.publish("smartpole/temperature","Ready!");
    // } else {
    //   Serial.print("failed, rc=");
    //   Serial.print(mqttClient.state());
    // }
    mqttClient.loop();
  } else {
    mesh.sendSingle(ID_GATEWAY, "Message from node1");
  }
  mesh.update();
}

IPAddress getlocalIP() {
  return IPAddress(mesh.getStationIP());
}
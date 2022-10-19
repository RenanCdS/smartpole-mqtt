#include <Arduino.h>
#include <painlessMesh.h>
#include <PubSubClient.h>
#include <cstring>
#include <ArduinoJson.h>


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

IPAddress getlocalIP();
void mqttCallback(char* topic, byte* payload, unsigned int length);

IPAddress myIP(0,0,0,0);
IPAddress mqttBroker(192, 168, 211, 117);

Scheduler userScheduler;
painlessMesh mesh;
WiFiClient wifiClient;
PubSubClient mqttClient;

void sendMessage() ;

Task taskSendmsg(TASK_SECOND * 3,TASK_FOREVER, &sendMessage);

/// @brief Sends information to mqtt broker based on a topic
/// @param topic The topic that the message will be sent
/// @param message 
void sendToMqttBroker(String topic, String message) {
  Serial.print("Attempting MQTT connection...");
  Serial.print(WiFi.status());
   if (mqttClient.connect("ESP32Client")) {
      Serial.printf("Sending message from %u to topic %s\n...", mesh.getNodeId(), topic.c_str());
      mqttClient.publish(topic.c_str(), message.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
    }
}

void receivedData(uint32_t from, String sensorData) {
  sendToMqttBroker("smartpole/condominio-fesa/data", sensorData);
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
  // In case mesh network does not start comment this line, upload the code in root node.
  // After that uncomment this line and upload the code again in root node
  WiFi.begin(STATION_SSID, STATION_PASSWORD);
  mqttClient.setServer(mqttBroker, 1883);
  mqttClient.setCallback(mqttCallback);  
  mqttClient.setClient(wifiClient);
}

String getDataObject(float sound, float temperature, float humidity, float energy) {
  String output;
  
  DynamicJsonDocument  doc(200);
  doc["sound"] = sound;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["energy"] = energy;

  serializeJson(doc, output);

  return output;
}

float getTemperatureData() {
  return 20.0;
}

float getSoundData() {
  return 1.0;
}

float getEnergyData() {
  return 3.0; 
}

float getHumidityData() {
  return 4.0;
}

void sendMessage() {
  float temperature = getTemperatureData();
  float humidity = getHumidityData();
  float sound = getSoundData();
  float energy = getEnergyData();

  String dataObjectString = getDataObject(sound, temperature, humidity, energy);

  mesh.sendSingle(ID_GATEWAY, dataObjectString);
  taskSendmsg.setInterval(TASK_SECOND * 2);
}

void setup() {
  Serial.begin(115200);
  
  // MESH NETWORK 
  mesh.setDebugMsgTypes(ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); 

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 11);

  if (IS_ROOT) {
    mesh.stationManual(STATION_SSID, STATION_PASSWORD);
    mesh.setHostname(ROOT_HOSTNAME);
    configureMqtt();
  } else {
    mesh.setHostname(NODE_HOSTNAME);
  }

  mesh.setRoot(IS_ROOT);
  mesh.setContainsRoot(true);
  mesh.onReceive(&receivedData);

  if (IS_ROOT == false)
  {
    userScheduler.addTask(taskSendmsg);
    taskSendmsg.enable();
  }
}

void loop() {
  if (IS_ROOT) {
    mqttClient.loop();
  }
  mesh.update();
}

IPAddress getlocalIP() {
  return IPAddress(mesh.getStationIP());
}
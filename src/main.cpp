#include <Arduino.h>
#include <painlessMesh.h>
#include <PubSubClient.h>
#include <cstring>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"

#include "painlessMesh.h"

// Temperature and humidity sensor configuration
#define DHTPIN 4
#define DHTTYPE DHT11  
DHT dht(DHTPIN, DHTTYPE);

#define CURRENT_POLE_ID 2747788829

#define ID_GATEWAY 1370564033 
#define ID_POLE_1 2747788829
#define ID_POLE_2 2224947593

#define TURN_ON_KEYWORD "TURN_ON"

#define IS_ROOT           true
#define ROOT_HOSTNAME     "SmartPoleRoot"
#define NODE_HOSTNAME     "SmartPoleNode2"

#define MESH_PREFIX       "SMART_POLES_NETWORK"
#define MESH_PASSWORD     "PASSWORD"
#define MESH_PORT         5555

#define STATION_SSID      ""
#define STATION_PASSWORD  ""
#define STATION_PORT     5555

int CONDOMINIUM_CODE = 123;
/// @brief Topic pattern = smartpole/<CONDOMINIUM_CODE>/data
String CONDOMINIUM_TOPIC = "smartpole/123/gateway/data";
void mqttCallback(char* topic, byte* payload, unsigned int length);

IPAddress myIP(0,0,0,0);
IPAddress mqttBroker(34, 200, 57, 252);

Scheduler userScheduler;
painlessMesh mesh;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void sendMessage();
void sendMessageToGateway();
IPAddress getlocalIP();

Task taskSendmsg(TASK_SECOND * 3,TASK_FOREVER, &sendMessage);
Task taskSendmsgToGateway(TASK_SECOND * 3,TASK_FOREVER, &sendMessageToGateway);

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

/// @brief Verifies if the data received is the event TURN_ON or broker data.
/// In case it is the turn on data it will turn on the light, otherwise it will send the received data to the broker
/// @param from 
/// @param data 
void receivedData(uint32_t from, String data) {
  if (data == TURN_ON_KEYWORD) {
    turnOnLight();
  } else {
    if (from == ID_POLE_1) 
    {
      sendToMqttBroker("smartpole/123/pole_1/data", data);
    } else {
      sendToMqttBroker("smartpole/123/pole_2/data", data);
    }
  } 
}

void configureMqtt() 
{
  // In case mesh network does not start comment this line, upload the code in root node.
  // After that uncomment this line and upload the code again in root node
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(STATION_SSID, STATION_PASSWORD);
  mqttClient.setServer(mqttBroker, 1883);
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

// TODO: Implement the real sensors
float getTemperatureData() {
  return dht.readTemperature();
}

float getSoundData() {
  return rand() % (10 + 1 - 0) + 0;
}

float getEnergyData() {
  return rand() % (10 + 1 - 0) + 0;
}

float getHumidityData() {
  return dht.readHumidity();
}

String getDataObjectString() {
  float temperature = getTemperatureData();
  float humidity = getHumidityData();
  float sound = getSoundData();
  float energy = getEnergyData();


  return getDataObject(sound, temperature, humidity, energy);
}

void sendMessage() {
  String dataObjectString = getDataObjectString();

  mesh.sendSingle(ID_GATEWAY, dataObjectString);
  taskSendmsg.setInterval(TASK_SECOND * 5);
}

void sendMessageToGateway() {
  // smartpole/{condominiumCode}/gateway/data
  sendToMqttBroker("smartpole/123/gateway/data", getDataObjectString());

  taskSendmsgToGateway.setInterval(TASK_SECOND * 5);
}

void setup() {
  Serial.begin(115200);
  // dht.begin();
  
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

  if (IS_ROOT == true) {
    userScheduler.addTask(taskSendmsgToGateway);
    taskSendmsgToGateway.enable();
  }
  else
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

  verifyPoleLights();
}

/// @brief Check if the presence sensor is active in the current pole and in case it is
/// it will send a message to the next pole via mesh network. This assumes an architecture of 3 poles
void verifyPoleLights() {
  if (isPresenceSensorActive()) {
    switch (CURRENT_POLE_ID) {
      case ID_POLE_1:
        turnOnLightAndsendMessageToNextPole(ID_POLE_2);
        break;
      case ID_POLE_2:
        turnOnLightAndsendMessageToNextPole(ID_GATEWAY);
        turnOnLightAndsendMessageToNextPole(ID_POLE_1);
        break;
      case ID_GATEWAY:
        turnOnLightAndsendMessageToNextPole(ID_POLE_2);
        break;
    }
  }
}

bool isPresenceSensorActive() {
  return true;
}

void turnOnLightAndsendMessageToNextPole(int nextPole) {
  turnOnLight();
  mesh.sendSingle(nextPole, "TURN_ON");
}

// TODO: Implement the logic to turn on the light
void turnOnLight() {

}
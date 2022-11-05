#include <Arduino.h>
#include <painlessMesh.h>
#include <PubSubClient.h>
#include <cstring>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"

#include "painlessMesh.h"

// Light configuration
#define RELE_1  22
#define RELE_2  23
#define LIGHT_SENSOR_PIN 18 // ESP32 pin GIOP36 (ADC0)
#define PRESENCE_SENSOR_PIN 12 // ESP32 pin GIOP12 (ADC5)

bool nightTime;
int intervalLDR;
int DELAY_TIME = 500;
int BASE_TIME_TO_VERIFY_SENSORS = 3000;

// Temperature and humidity sensor configuration
#define DHTPIN 4
#define DHTTYPE DHT11  
DHT dht(DHTPIN, DHTTYPE);

// Pole configuration
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

Scheduler scheduler;
painlessMesh mesh;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void sendMessage();
void sendMessageToGateway();
void verifyLightControl();

IPAddress getlocalIP();

Task taskSendmsg(TASK_SECOND * 3,TASK_FOREVER, &sendMessage);
Task taskSendmsgToGateway(TASK_SECOND * 3,TASK_FOREVER, &sendMessageToGateway);
Task taskVerifyLightControl(TASK_SECOND * 2, TASK_FOREVER, &verifyLightControl);

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

void verifyLightControl()
{
  bool hasPresence = digitalRead(PRESENCE_SENSOR_PIN);
  intervalLDR += DELAY_TIME;
  if (intervalLDR >= BASE_TIME_TO_VERIFY_SENSORS)
  {
    int lightValue = digitalRead(LIGHT_SENSOR_PIN); // read light sensor
    Serial.print("light -> ");
    Serial.println(lightValue);
    nightTime = lightValue; // is night time
    intervalLDR = 0;
  }

  if (nightTime)
  {
    Serial.println("Entrou no digital write");
    digitalWrite(RELE_1, HIGH);
  }
  else
  {
    digitalWrite(RELE_1, LOW);
  }

  if (nightTime && hasPresence)
  {

    digitalWrite(RELE_2, HIGH); // presence detected
  }
  else
  {
    digitalWrite(RELE_2, LOW); // no presence detected
  }
}

// TODO: Implement the logic to turn on the light
void turnOnLight() {

}

void turnOnLightAndsendMessageToNextPole(int nextPole) {
  turnOnLight();
  mesh.sendSingle(nextPole, "TURN_ON");
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
  // Presence sensor and rele configuration
  pinMode(PRESENCE_SENSOR_PIN, INPUT);
  pinMode(RELE_1, OUTPUT);
  pinMode(RELE_1, OUTPUT);

  // Humidity configuration
  // dht.begin();
  
  // MESH NETWORK 
  mesh.setDebugMsgTypes(ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); 
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &scheduler, MESH_PORT, WIFI_AP_STA, 11);

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
    scheduler.addTask(taskSendmsgToGateway);
    taskSendmsgToGateway.enable();
  }
  else
  {
    scheduler.addTask(taskSendmsg);
    taskSendmsg.enable();
  }

  // Task of light control
  scheduler.addTask(taskVerifyLightControl);
  taskVerifyLightControl.enable();
}


void loop() {
  if (IS_ROOT) {
    mqttClient.loop();
  } 
  mesh.update();
}
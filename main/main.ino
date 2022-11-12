#include <Arduino.h>
#include <painlessMesh.h>
#include "PubSubClient.h"
#include <cstring>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#include <driver/i2s.h>
#include "sos-iir-filter.h"
#include "TimedAction.h"
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

#define STATION_SSID      "Net boa"
#define STATION_PASSWORD  "conectaai"
#define STATION_PORT     5555

// Sound sensor configuration

#define LEQ_PERIOD        1           
#define WEIGHTING         C_weighting 
#define LEQ_UNITS         "LAeq"      
#define DB_UNITS          "dBA"       
#define USE_DISPLAY       0

#define MIC_EQUALIZER     ICS43434    
#define MIC_OFFSET_DB     3.0103     

#define MIC_SENSITIVITY   -26        
#define MIC_REF_DB        94.0       
#define MIC_OVERLOAD_DB   116.0       
#define MIC_NOISE_DB      29         
#define MIC_BITS          24         
#define MIC_CONVERT(s)    (s >> (SAMPLE_BITS - MIC_BITS))
#define MIC_TIMING_SHIFT  0  

constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

#define I2S_PORT          I2S_NUM_0

#define SAMPLE_RATE       48000
#define SAMPLE_BITS       32
#define SAMPLE_T          int32_t 
#define SAMPLES_SHORT     (SAMPLE_RATE / 8)
#define SAMPLES_LEQ       (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE     (SAMPLES_SHORT / 16)
#define DMA_BANKS         32

#define I2S_TASK_PRI   4
#define I2S_TASK_STACK 2048

SOS_IIR_Filter DC_BLOCKER = { 
  gain: 1.0,
  sos: {{-1.0, 0.0, +0.9992, 0}}
};

SOS_IIR_Filter ICS43434 = { 
  gain: 0.477326418836803,
  sos: {
   {+0.96986791463971267, 0.23515976355743193, -0.06681948004769928, -0.00111521990688128},
   {-1.98905931743624453, 0.98908924206960169, +1.99755331853906037, -0.99755481510122113}
  }
};

SOS_IIR_Filter ICS43432 = {
  gain: -0.457337023383413,
  sos: {
    {-0.544047931916859, -0.248361759321800, +0.403298891662298, -0.207346186351843},
    {-1.909911869441421, +0.910830292683527, +1.790285722826743, -0.804085812369134},
    {+0.000000000000000, +0.000000000000000, +1.148493493802252, -0.150599527756651}
  }
};

SOS_IIR_Filter INMP441 = {
  gain: 1.00197834654696, 
  sos: {
    {-1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}
  }
};

SOS_IIR_Filter IM69D130 = {
  gain: 1.00124068496753,
  sos: {
    {-1.0, 0.0, +0.9992, 0},
    {-1.994461610298131, 0.994469278738208, +1.997675693595542, -0.997677044195563}
  }
};

SOS_IIR_Filter SPH0645LM4H_B_RB = {
  gain: 1.00123377961525, 
  sos: {
    {-1.0, 0.0, +0.9992, 0},
    {-1.988897663539382, +0.988928479008099, +1.993853376183491, -0.993862821429572}
  }
};

SOS_IIR_Filter A_weighting = {
  gain: 0.169994948147430, 
  sos: {
    {-2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926},
    {+4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332},
    {-0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989}
  }
};

SOS_IIR_Filter C_weighting = {
  gain: -0.491647169337140,
  sos: { 
    {+1.4604385758204708, +0.5275070373815286, +1.9946144559930252, -0.9946217070140883},
    {+0.2376222404939509, +0.0140411206016894, -1.3396585608422749, -0.4421457807694559},
    {-2.0000000000000000, +1.0000000000000000, +0.3775800047420818, -0.0356365756680430}
  }
};

struct sum_queue_t {
  float sum_sqr_SPL;

  float sum_sqr_weighted;

  uint32_t proc_ticks;
};

QueueHandle_t samples_queue;

float samples[SAMPLES_SHORT] __attribute__((aligned(4)));

void mic_i2s_init() {
  const i2s_config_t i2s_config = {
    mode: i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    sample_rate: SAMPLE_RATE,
    bits_per_sample: i2s_bits_per_sample_t(SAMPLE_BITS),
    channel_format: I2S_CHANNEL_FMT_ONLY_LEFT,
    communication_format: i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,
    dma_buf_count: DMA_BANKS,
    dma_buf_len: DMA_BANK_SIZE,
    use_apll: true,
    tx_desc_auto_clear: false,
    fixed_mclk: 0
  };

  const i2s_pin_config_t pin_config = {
    bck_io_num:   I2S_SCK,  
    ws_io_num:    I2S_WS,    
    data_out_num: -1,
    data_in_num:  I2S_SD   
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  #if (MIC_TIMING_SHIFT > 0) 
    REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));   
    REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);  
  #endif
  
  i2s_set_pin(I2S_PORT, &pin_config);
}

void mic_i2s_reader_task(void* parameter) {
  mic_i2s_init();

  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) { 
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

    TickType_t start_tick = xTaskGetTickCount();
    
    SAMPLE_T* int_samples = (SAMPLE_T*)&samples;
    for(int i=0; i<SAMPLES_SHORT; i++) samples[i] = MIC_CONVERT(int_samples[i]);

    sum_queue_t q;
   
    q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);

    q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);

    q.proc_ticks = xTaskGetTickCount() - start_tick;

    xQueueSend(samples_queue, &q, portMAX_DELAY);
  }
}

float soundValue = 0;

void verifySoundControl();

void verifySoundControl(void* parameter) {
    samples_queue = xQueueCreate(8, sizeof(sum_queue_t));

    sum_queue_t q;
    uint32_t Leq_samples = 0;
    double Leq_sum_sqr = 0;
    double Leq_dB = 0;

    // Read sum of samaples, calculated by 'i2s_reader_task'
    while (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {
        // Calculate dB values relative to MIC_REF_AMPL and adjust for microphone reference
        double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
        double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

        // In case of acoustic overload or below noise floor measurement, report infinty Leq value
        if (short_SPL_dB > MIC_OVERLOAD_DB) {
        Leq_sum_sqr = INFINITY;
        } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
        Leq_sum_sqr = -INFINITY;
        }

        Leq_sum_sqr += q.sum_sqr_weighted;
        Leq_samples += SAMPLES_SHORT;

        if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
          double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
          Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
          Leq_sum_sqr = 0;
          Leq_samples = 0;
          
          soundValue = Leq_dB;
        }
    }  
}

// End of sensor configuration

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
Task taskVerifyLightControl(TASK_SECOND * 4, TASK_FOREVER, &verifyLightControl);

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

void presenceLightControl()
{
  bool hasPresence = digitalRead(PRESENCE_SENSOR_PIN);
  hasPresence =1;

  Serial.println("Presença...");
  Serial.println(hasPresence);
  Serial.println("NightTime...");
  Serial.println(nightTime);

  if (nightTime && hasPresence)
  {
    Serial.println("Entrou no RELE 2...");
    digitalWrite(RELE_2, HIGH); // presence detected
  }
  else
  {
    digitalWrite(RELE_2, LOW); // no presence detected
  }
}

void verifyLightControl()
{
  bool hasPresence = digitalRead(PRESENCE_SENSOR_PIN);
  // intervalLDR += DELAY_TIME;
  // if (intervalLDR >= BASE_TIME_TO_VERIFY_SENSORS)
  // {
  int lightValue = digitalRead(LIGHT_SENSOR_PIN); // read light sensor
  Serial.print("light -> ");
  // Serial.println(lightValue);
  nightTime = lightValue; // is night time
  // intervalLDR = 0;
  // }

  if (nightTime)
  {
    Serial.println("Entrou no digital write");
    digitalWrite(RELE_1, HIGH);
  }
  else
  {
    digitalWrite(RELE_1, LOW);
  }
}

// TODO: Implement the logic to turn on the light
void turnOnLight() {
  digitalWrite(RELE_2, HIGH);
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

String getDataObject(float sound, float temperature, float humidity) {
  String output;
  
  DynamicJsonDocument  doc(200);
  doc["sound"] = sound;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;

  serializeJson(doc, output);

  return output;
}

// TODO: Implement the real sensors
float getTemperatureData() {
  return dht.readTemperature();
}

float getSoundData() {
  return soundValue;
}

float getHumidityData() {
  return dht.readHumidity();
}

String getDataObjectString() {
  float temperature = getTemperatureData();
  float humidity = getHumidityData();
  float sound = getSoundData();

  return getDataObject(sound, temperature, humidity);
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
  pinMode(RELE_2, OUTPUT);

  // Humidity configuration
  dht.begin();
  
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

  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);
  xTaskCreate(verifySoundControl, "Sound Control", I2S_TASK_STACK, NULL, 5, NULL);
}


void loop() {
  if (IS_ROOT) {
    mqttClient.loop();
  }
  mesh.update();
}
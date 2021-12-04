#define MQTT_MAX_PACKET_SIZE 512
#define DEEP_SLEEP_TIME 1e6
#define SEALEVELPRESSURE_HPA (1013.25)
#define BMP280 0
#define BME280 1

#define ROOM_CALIBRATION_EEPROM_INDEX 0
#define ROOM_FREQUENCY_EEPROM_INDEX 4

#include <config.h>
#include <BMP280_DEV.h>
#include <Adafruit_BME280.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <EEPROM.h>


float temperature, pressure, altitude, humidity;
BMP280_DEV bmp280;
Adafruit_BME280 bme;

// Sensor settings
String sensorId;
const int sensor = BMP280;
float calibration = 0.0;
int frequency = 60;
String stateTopic;
String calibrationCmdTopic;
String frequencyCmdTopic;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void sendMQTTTemperatureDiscoveryMsg() {
  String discoveryTopic = "homeassistant/sensor/room_sensor_" + sensorId + "/temperature/config";

  DynamicJsonDocument doc(MQTT_MAX_PACKET_SIZE);
  char buffer[MQTT_MAX_PACKET_SIZE];

  doc["name"] = sensorId + " room Temperature";
  doc["unique_id"] = sensorId + "_temperature";
  doc["stat_t"]   = stateTopic;
  doc["unit_of_meas"] = "°C";
  doc["dev_cla"] = "temperature";
  doc["frc_upd"] = true;
  doc["val_tpl"] = "{{ value_json.temperature|default(0) }}";

  size_t n = serializeJson(doc, buffer);

  if (!client.publish(discoveryTopic.c_str(), buffer, n)) {
    Serial.println("Error publishing state");
  }
}

void sendMQTTHumidityDiscoveryMsg() {
  String discoveryTopic = "homeassistant/sensor/room_sensor_" + sensorId + "/humidity/config";

  DynamicJsonDocument doc(MQTT_MAX_PACKET_SIZE);
  char buffer[MQTT_MAX_PACKET_SIZE];

  doc["name"] = sensorId + " room Humidity";
  doc["unique_id"] = sensorId + "_humidity";
  doc["stat_t"]   = stateTopic;
  doc["unit_of_meas"] = "%";
  doc["dev_cla"] = "humidity";
  doc["frc_upd"] = true;
  doc["val_tpl"] = "{{ value_json.humidity|default(0) }}";

  size_t n = serializeJson(doc, buffer);

  if (!client.publish(discoveryTopic.c_str(), buffer, n)) {
    Serial.println("Error publishing state");
  }
}

void sendMQTTCalibrationDiscoveryMsg() {
  String discoveryTopic = "homeassistant/number/room_sensor_" + sensorId + "/calibration/config";

  DynamicJsonDocument doc(MQTT_MAX_PACKET_SIZE);
  char buffer[MQTT_MAX_PACKET_SIZE];

  doc["name"] = sensorId + " room Calibration";
  doc["unique_id"] = sensorId + "_calibration";
  doc["icon"] = "mdi:scale-balance";
  doc["stat_t"] = stateTopic;
  doc["cmd_t"] = calibrationCmdTopic;
  doc["min"] = -20;
  doc["max"] = 20;
  doc["step"] = 0.01;
  doc["unit_of_meas"] = "ºC";
  doc["val_tpl"] = "{{ value_json.calibration|default(0) }}";

  size_t n = serializeJson(doc, buffer);

  if (!client.publish(discoveryTopic.c_str(), buffer, n)) {
    Serial.println("Error publishing state");
  }
}

void sendMQTTFrequencyDiscoveryMsg() {
  String discoveryTopic = "homeassistant/number/room_sensor_" + sensorId + "/frequency/config";

  DynamicJsonDocument doc(MQTT_MAX_PACKET_SIZE);
  char buffer[MQTT_MAX_PACKET_SIZE];

  doc["name"] = sensorId + " room Frequency";
  doc["unique_id"] = sensorId + "_frequency";
  doc["icon"] = "mdi:update";
  doc["stat_t"] = stateTopic;
  doc["cmd_t"] = frequencyCmdTopic;
  doc["min"] = 1;
  doc["max"] = 3600;
  doc["step"] = 1;
  doc["unit_of_meas"] = "s";
  doc["val_tpl"] = "{{ value_json.frequency|default(0) }}";

  size_t n = serializeJson(doc, buffer);

  if (!client.publish(discoveryTopic.c_str(), buffer, n)) {
    Serial.println("Error publishing state");
  }
}

void sendMqttStatus() {
  DynamicJsonDocument doc(MQTT_MAX_PACKET_SIZE);
    char buffer[MQTT_MAX_PACKET_SIZE];
    doc["temperature"] = temperature;
    doc["humidity"] = humidity;
    doc["calibration"] = calibration;
    doc["frequency"] = frequency;
    String docString;
    serializeJson(doc, docString);
    Serial.println(docString);
    size_t n = serializeJson(doc, buffer);
    bool published = client.publish(stateTopic.c_str(), buffer, n);
    delay(100);
    if (!published) {
      Serial.println("Error publishing state");
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length){
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  String message;
  for (unsigned int i = 0; i < length; i++) {
      message = message + (char) payload[i];  // convert *byte to string
  }
  Serial.print(message);
  
  if (calibrationCmdTopic == topic) {
    calibration = message.toFloat();
    EEPROM.put(ROOM_CALIBRATION_EEPROM_INDEX, calibration);
    sendMqttStatus();
  } else if (frequencyCmdTopic == topic) {
    frequency = message.toInt();
    EEPROM.put(ROOM_FREQUENCY_EEPROM_INDEX, frequency);
    sendMqttStatus();
  } else {
    Serial.println("Invalid topic");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2000);
  
  sensorId = String(ESP.getChipId(), HEX);
  stateTopic = "home/rooms/" + sensorId + "/state";
  calibrationCmdTopic = "home/rooms/" + sensorId + "/calibration";
  frequencyCmdTopic = "home/rooms/" + sensorId + "/frequency";

  // Wait for serial to initialize.
  while(!Serial) { }
  Serial.println("start");

  EEPROM.get(ROOM_CALIBRATION_EEPROM_INDEX, calibration);
  EEPROM.get(ROOM_FREQUENCY_EEPROM_INDEX, frequency);
  if (frequency < 1) {
    frequency = 60;
  }
  /*if (tmp == NULL) {
    Serial.println("EEPROM Has no stored calibration, storing 0.");
    EEPROM.put(ROOM_CALIBRATION_EEPROM_INDEX, 0.0);
  } else {
    EEPROM.get(ROOM_CALIBRATION_EEPROM_INDEX, calibration);
  }  */

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  Serial.println("Connected to Wi-Fi");

  client.setBufferSize(MQTT_MAX_PACKET_SIZE);
  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(mqttCallback);

  Serial.println("Connecting to MQTT");

  while (!client.connected()) {
    Serial.print("Trying to connect with id: ");
    Serial.println(sensorId);
    

    if (client.connect(sensorId.c_str(), MQTT_USER, MQTT_PASS)) {

      Serial.println("Connected to MQTT");

      sendMQTTTemperatureDiscoveryMsg();
      //sendMQTTHumidityDiscoveryMsg();
      sendMQTTCalibrationDiscoveryMsg();
      sendMQTTFrequencyDiscoveryMsg();
      client.subscribe(calibrationCmdTopic.c_str());
      client.subscribe(frequencyCmdTopic.c_str());
    } else {

      Serial.print("failed with state ");
      Serial.println(client.state());
      delay(2000);

    }
  }

  if (sensor == BMP280) {
    bmp280.begin(BMP280_I2C_ALT_ADDR); 
  } else if (sensor == BME280) {
    unsigned  status;  
    // You can also pass in a Wire library object like &Wire2
    status = bme.begin(0x76, &Wire);
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    } else {
      Serial.println("ERROR! Invalid sensor config.");
    }
  }
}

void sendStatus() {
  Serial.println();
  Serial.print(temperature);                    // Display the results    
  Serial.print(F("*C   "));
  Serial.print(pressure);    
  Serial.print(F("hPa   "));
  Serial.print(altitude);
  Serial.println(F("m   ")); 
  Serial.print(humidity);
  Serial.println(F("%")); 

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Sending data...");
    sendMqttStatus();
  }
  ESP.deepSleep(frequency*1e6);
}

void loop() {
  client.loop();
  if (sensor == BMP280) {
    bmp280.startForcedConversion();
    if (bmp280.getMeasurements(temperature, pressure, altitude)) {
      temperature += calibration;
      sendStatus();
    }
  } else if (sensor == BME280) {
    temperature = bme.readTemperature() + calibration;
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    humidity = bme.readHumidity();
    sendStatus();
  } else {
    Serial.println("ERROR! Invalid sensor config.");
  }
  delay(10);
}

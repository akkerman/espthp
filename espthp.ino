#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "config.h"

#define STATUS_DISCONNECTED "disconnected"
#define STATUS_ONLINE "online"

ADC_MODE(ADC_VCC);

unsigned long previousMillis = 0;
const long interval = 10 * 60 * 1000;

const String chipId = String(ESP.getChipId());
const String baseTopic = "raw/" + chipId + "/";
const String tempTopic = baseTopic + "temperature";
const String humiTopic = baseTopic + "humidity";
const String presTopic = baseTopic + "pressure";
const String willTopic = baseTopic + "status";
const String vccTopic  = baseTopic + "vcc";
const String ipTopic   = baseTopic + "ip";

IPAddress ip;

WiFiClient WiFiClient;
PubSubClient client(WiFiClient);
Adafruit_BME280 bme; // I2C

void setup() {

  Serial.begin(115200);
  delay(10);

  Wire.begin(0, 2);
  Wire.setClock(100000);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  ip = WiFi.localIP();
  Serial.println(ip);

  client.setServer(MQTT_IP, MQTT_PORT);
}

void loop() {
  yield();
  if (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect(chipId.c_str(), willTopic.c_str(), 1, true, STATUS_DISCONNECTED)) {
      Serial.println("MQTT client connected.");
      client.publish(willTopic.c_str(), STATUS_ONLINE, true);
      client.publish(ipTopic.c_str(), ip.toString().c_str(), true);
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
    }
  }

  unsigned long currentMillis = millis();

  if (client.connected()) {
    if (previousMillis ==0 || currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      bmeReadSend();
      vccReadSend();
    }
    client.loop();
  } else {
    previousMillis = 0;
  }
}

void bmeReadSend() {
  char temperature[6];
  char humidity[6];
  char pressure[7];

  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure()/100.0F;

  dtostrf(t, 5, 1, temperature);
  dtostrf(h, 5, 1, humidity);
  dtostrf(p, 5, 1, pressure);

  Serial.print("t,h,p: ");
  Serial.print(temperature); Serial.print(", ");
  Serial.print(humidity);    Serial.print(", ");
  Serial.print(pressure);    Serial.println();
  client.publish(tempTopic.c_str(), temperature);
  client.publish(humiTopic.c_str(), humidity);
  client.publish(presTopic.c_str(), pressure);
}

void vccReadSend() {
    float v  = ESP.getVcc() / 1000.0;
    char vcc[10];
    dtostrf(v, 5, 1, vcc);
    client.publish(vccTopic.c_str(), vcc);
}

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "config.h"

WiFiClient WiFiClient;
PubSubClient client(WiFiClient);

#define STATUS_DISCONNECTED "disconnected"
#define STATUS_ONLINE "online"

const String chipId = String(ESP.getChipId());
const String baseTopic = "raw/" + chipId + "/";
const char* tempTopic = (baseTopic + "temperature").c_str();
const char* humiTopic = (baseTopic + "humidity").c_str();
const char* presTopic = (baseTopic + "pressure").c_str();;
const char* willTopic = (baseTopic + "status").c_str();


void setup() {
  
  Serial.begin(115200);
  delay(10);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(MQTT_IP, MQTT_PORT);
}

void loop() {
  yield();
  if (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect(chipId.c_str(), willTopic, 1, true, STATUS_DISCONNECTED)) {
      Serial.println("MQTT client connected.");
      client.publish(willTopic, STATUS_ONLINE, true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" will try again in 5 seconds");      
      delay(5000);    
    }
  }

  if (client.connected()) {
    bmeReadSend();
    client.loop();
    delay(10000);
  }
}

void bmeReadSend() {
  Serial.println("Sending values");    
  int t = 20; 
  int h = 50;
  int p = 1014;  
  client.publish(tempTopic, String(t).c_str());
  client.publish(humiTopic, String(h).c_str());
  client.publish(presTopic, String(p).c_str());
}


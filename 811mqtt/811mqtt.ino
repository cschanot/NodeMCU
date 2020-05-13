/*
  ccs811basic.ino - Demo sketch printing results of the CCS811 digital gas sensor for monitoring indoor air quality from ams.
  Created by Maarten Pennings 2017 Dec 11
*/


#include <Wire.h>    // I2C library
#include "ccs811.h"  // CCS811 library
#include "DHT.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

const char* ssid = "120Mesh";
const char* password = "1801ca1326";
char* server = "192.168.1.20";
char* hellotopic = "OfficeNode";
long previous811Time = 0;
#define CCS_REPORT_INTERVAL 10000 // in milliseconds

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

String clientName;
WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);
CCS811 ccs811(D3); // nWAKE on D3

void setup() {
  Serial.begin(115200);
  Serial.println("");
  Serial.println("setup: Starting CCS811 basic demo");
  Serial.print("setup: ccs811 lib  version: "); Serial.println(CCS811_VERSION);

  // Enable I2C
  Wire.begin(); 
  
  // Enable CCS811
  ccs811.set_i2cdelay(50); // Needed for ESP8266 because it doesn't handle I2C clock stretch correctly
  bool ok= ccs811.begin();
  if( !ok ) Serial.println("setup: CCS811 begin FAILED");

  // Print CCS811 versions
  Serial.print("setup: hardware    version: "); Serial.println(ccs811.hardware_version(),HEX);
  Serial.print("setup: bootloader  version: "); Serial.println(ccs811.bootloader_version(),HEX);
  Serial.print("setup: application version: "); Serial.println(ccs811.application_version(),HEX);
  
  // Start measuring
  ok= ccs811.start(CCS811_MODE_1SEC);
  if( !ok ) Serial.println("setup: CCS811 start FAILED");
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  clientName += "NodeMCU_Office2";
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac);
  clientName += "-";
  clientName += String(micros() & 0xff, 16);

  Serial.print("Connecting to ");
  Serial.print(server);
  Serial.print(" as ");
  Serial.println(clientName);

  if (client.connect((char*) clientName.c_str())) {
    Serial.println("Connected to MQTT broker");
    
    if (client.publish(hellotopic, "Hello from NodeMCU")) {
      Serial.println("Publish ok");
    }
    else {
      Serial.println("Publish failed");
    }
  }
  else {
    Serial.println("MQTT connect failed");
    Serial.println("Will reset and try again...");
    abort();
  }

}


void loop() {
  // Read
  uint16_t eco2, etvoc, errstat, raw;
  unsigned long currentMillis = millis();
  
  if(currentMillis - previous811Time > CCS_REPORT_INTERVAL) {
    // save the last time you detected Motion
    previous811Time = currentMillis;   
  ccs811.read(&eco2,&etvoc,&errstat,&raw); 
  // Print measurement results based on status
  if( errstat==CCS811_ERRSTAT_OK ) { 
    Serial.print("CCS811: ");
    Serial.print("eco2=");  Serial.print(eco2);     Serial.print(" ppm  ");
    Serial.print("etvoc="); Serial.print(etvoc);    Serial.print(" ppb  ");
    String payload = "{\"CO2\":";
    payload += eco2;
    payload += ",\"VOC\":";
    payload += etvoc;
    payload += "}";
    sendMQTT("Office/811",payload);
    Serial.println();
  } else if( errstat==CCS811_ERRSTAT_OK_NODATA ) {
    Serial.println("CCS811: waiting for (new) data");
  } else if( errstat & CCS811_ERRSTAT_I2CFAIL ) { 
    Serial.println("CCS811: I2C error");
  } else {
    Serial.print("CCS811: errstat="); Serial.print(errstat,HEX); 
    Serial.print("="); Serial.println( ccs811.errstat_str(errstat) ); 
  }
  }  
}
void sendMQTT(String topicMQTT, String payload) {
  if (!client.connected()) {
    if (client.connect((char*) clientName.c_str())) {
      Serial.println("Connected to MQTT broker again");
      Serial.print("Topic is: ");
      Serial.println(topicMQTT);
    }
    else {
      Serial.println("MQTT connect failed");
      Serial.println("Will reset and try again...");
      abort();
    }
  }

  if (client.connected()) {
    Serial.print("Sending payload: ");
    Serial.println(payload);

    if (client.publish((char *)topicMQTT.c_str(), (char*) payload.c_str())) {
      Serial.println("Publish ok");
    }
    else {
      Serial.println("Publish failed");
    }
  }

}


String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

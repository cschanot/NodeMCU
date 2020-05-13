// Code based on
// https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHTtester/DHTtester.ino
// https://gist.github.com/igrr/7f7e7973366fc01d6393
// https://github.com/iot-playground/Arduino/blob/master/ESP8266ArduinoIDE/DS18B20_temperature_sensor/DS18B20_temperature_sensor.ino
// Must use Adafruit DHT library <=1.3.0 or you will get errors
// esp8266 + dht22 + mqtt

#include "DHT.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

const char* ssid = "120Mesh";
const char* password = "1801ca1326";
char* server = "192.168.1.20";
char* hellotopic = "OfficeNode";
long previousDHTTime = 0;
long previousPIRTime = 0;
long previousMOTIONTime = 0;
#define PIRPIN 13
#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define DHT_REPORT_INTERVAL 30000 // in milliseconds
#define PIR_REPORT_INTERVAL 1000
#define MOTION_DETECT_INTERVAL 60000 // I don't want a message every second..every 6 min report Motion


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
DHT dht(DHTPIN, DHTTYPE, 15);
WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);

float oldH ;
float oldF ;

void setup() {
  Serial.begin(115200);
  pinMode(PIRPIN,INPUT);
  Serial.println("DHTxx test!");
  delay(20);

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

  clientName += "NodeMCU_Office";
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

  dht.begin();
  oldH = -1;
  oldF = -1;
}

void loop() {

unsigned long currentMillis = millis();

// PIR Code (every second)
  if(currentMillis - previousPIRTime > PIR_REPORT_INTERVAL) {
    // save the last time you detected Motion
    previousPIRTime = currentMillis;   
   long state = digitalRead(PIRPIN);
   if (state == HIGH) {
    Serial.println("Motion detected!");
    if(currentMillis - previousMOTIONTime > MOTION_DETECT_INTERVAL) {
      previousMOTIONTime = currentMillis;
    String Mpayload = "{\"Motion\":";
    Mpayload += 1;
    Mpayload += "}";
    sendMQTT("Office/PIR",Mpayload); 
    }
   }
   else {
    // No motion
   }
  }

// DHT CODE (every 30 seconds)
if(currentMillis - previousDHTTime > DHT_REPORT_INTERVAL) {
  previousDHTTime = currentMillis;
  float h = dht.readHumidity();
  float f = dht.readTemperature(true);
 
  if (isnan(h) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  
  float hi = dht.computeHeatIndex(f, h);

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hi);
  Serial.println(" *F");

  String payload = "{\"Humidity\":";
  payload += h;
  payload += ",\"Temperature\":";
  payload += f;
  payload += "}";
 
  if (f != oldF || h != oldH )
  {
    sendMQTT("Office/DHT22",payload);
    oldF = f;
    oldH = h;
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

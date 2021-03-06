// Code based on
// https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHTtester/DHTtester.ino
// https://gist.github.com/igrr/7f7e7973366fc01d6393
// https://github.com/iot-playground/Arduino/blob/master/ESP8266ArduinoIDE/DS18B20_temperature_sensor/DS18B20_temperature_sensor.ino

// esp8266 + dht22 + CCS811 + PIR + MQTT + Node Red code. This does OTA Update via httpupdate

// This is meant to be used along with MQTT/Node Red to create a UI with OTAUPDATE/RESET/OFF and ON buttons for PIR Sensor

#include "DHT.h"
#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <assert.h>
#include <Wire.h>    // I2C library
#include "ccs811.h"  // CCS811 library
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <time.h>

bool OTAUPDATE = false; // Update flag, if this goes to TRUE we fucking update this biiiiitch
int  _soldercorrection;
const char* hostname = "Downstairsnode2";
const char* ssid = "120Mesh";
const char* password = "1801ca1326";
char* MQTTserver = "192.168.1.20";
String MQTTTopic = "Downstairs";
const char* MQTTSubscribe = "DownstairsWEBButton";

char* hellotopic = "MQTTTest"; // Just to see if we are working with MQTT
long previousDHTTime = 0;
long previousPIRTime = 0;
long previousMOTIONTime = 0;
long previous811Time = 0;

#define PIRPIN 13
#define DHTPIN 10     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define DHT_REPORT_INTERVAL 20000 // in milliseconds
#define PIR_REPORT_INTERVAL 1000
#define MOTION_DETECT_INTERVAL 60000 // I don't want a message every second..every 1 min report Motion
#define CCS_REPORT_INTERVAL 20000 // in milliseconds
#define IDIV(n,d)                ((n)>0 ? ((n)+(d)/2)/(d) : ((n)-(d)/2)/(d))
bool PIRON = true; // MQTT can send OFF or ON payload to control motion sensor


void hollaback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  if (!strncmp((char *)payload, "RESTART", length)) {
    ESP.restart();
  } else if (!strncmp((char *)payload, "PIRON", length)) {
    PIRON = true;
  } else if (!strncmp((char *)payload, "PIROFF", length)) {
    PIRON = false;
  } else if (!strncmp((char *)payload, "OTAUPDATE", length)) {
    OTAUPDATE = true;
  }
}

String clientName;
DHT dht(DHTPIN, DHTTYPE);
WiFiClient wifiClient;
PubSubClient client(MQTTserver, 1883, hollaback, wifiClient);
CCS811 ccs811(D3); // nWAKE on D3
float oldH ; // Don't send the same goddamn
float oldF ;

void setup() {
  Serial.begin(115200);
  delay(20);

  // WIFI TIME BOYS

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  //  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostname);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // WIFI DONE

  // MQTT TIME
  clientName += hostname;
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac);
  clientName += "-";
  clientName += String(micros() & 0xff, 16);

  Serial.print("Initalizing Connection to MQTT SERVER");
  Serial.print(MQTTserver);
  Serial.print(" as ");
  Serial.println(clientName);

  if (client.connect((char*) clientName.c_str())) {
    Serial.println("Connected to MQTT broker");

    if (client.publish(hellotopic, sendStats().c_str())) {
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
  if (client.subscribe(MQTTSubscribe)) {
    Serial.println("Subscribe ok");
  }
  else {
    Serial.println("Subscribe failed");
  }
  // END MQTT

  pinMode(PIRPIN, INPUT); // PIR Init

  dht.begin(); // DHT22 INIT
  oldH = -1;
  oldF = -1;
  // Enable I2C

  // CCS811 BEGIN INIT
  Wire.begin();

  // Enable CCS811
  ccs811.set_i2cdelay(50); // Needed for ESP8266 because it doesn't handle I2C clock stretch correctly
  bool ok = ccs811.begin();
  if ( !ok ) Serial.println("setup: CCS811 begin FAILED");

  // Print CCS811 versions
  Serial.print("setup: hardware    version: "); Serial.println(ccs811.hardware_version(), HEX);
  Serial.print("setup: bootloader  version: "); Serial.println(ccs811.bootloader_version(), HEX);
  Serial.print("setup: application version: "); Serial.println(ccs811.application_version(), HEX);
  // Start measuring
  ok = ccs811.start(CCS811_MODE_1SEC);
  if ( !ok ) Serial.println("setup: CCS811 start FAILED");

}

void loop() {
  uint16_t eco2, etvoc, errstat, raw;
  unsigned long currentMillis = millis();


  // CHECK IF OTAUPDATE FLAG WAS SET BY MQTT CALL
  if (OTAUPDATE) {
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);

    // Add optional callback notifiers
    ESPhttpUpdate.onStart(update_started);
    ESPhttpUpdate.onEnd(update_finished);
    ESPhttpUpdate.onProgress(update_progress);
    ESPhttpUpdate.onError(update_error);

    t_httpUpdate_return ret = ESPhttpUpdate.update(wifiClient, "http://www.sechouse.tech/ota/Downstairsnode2.bin");
    // Or:
    //t_httpUpdate_return ret = ESPhttpUpdate.update(client, "server", 80, "file.bin");

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
  } else { // NO UTAUPDATE FLAG..GO TO PROGRAM LOOP
    // client.loop looks for MQTT subscribe data
    if (client.loop()) { // If no then we will check connection below, delay and try to reconnect

      // CCS811 JAZZ

      if (currentMillis - previous811Time > CCS_REPORT_INTERVAL) {
        // save the last time you detected Motion
        previous811Time = currentMillis;
        ccs811.read(&eco2, &etvoc, &errstat, &raw);
        // Print measurement results based on status
        if ( errstat == CCS811_ERRSTAT_OK ) {
          Serial.print("CCS811: ");
          Serial.print("eco2=");  Serial.print(eco2);     Serial.print(" ppm  ");
          Serial.print("etvoc="); Serial.print(etvoc);    Serial.print(" ppb  ");
          String payload = "{\"CO2\":";
          payload += eco2;
          payload += ",\"VOC\":";
          payload += etvoc;
          payload += "}";
          sendMQTT(MQTTTopic + "/811", payload);
          Serial.println();
        } else if ( errstat == CCS811_ERRSTAT_OK_NODATA ) {
          Serial.println("CCS811: waiting for (new) data");
        } else if ( errstat & CCS811_ERRSTAT_I2CFAIL ) {
          Serial.println("CCS811: I2C error");
        } else {
          Serial.print("CCS811: errstat="); Serial.print(errstat, HEX);
          Serial.print("="); Serial.println( ccs811.errstat_str(errstat) );
        }
      }

      // END CCS811

      // PIR Code (every second)
      if ((currentMillis - previousPIRTime > PIR_REPORT_INTERVAL) && PIRON) {
        // save the last time you detected Motion
        previousPIRTime = currentMillis;
        long state = digitalRead(PIRPIN);
        if (state == HIGH) {
          Serial.println("Motion detected!");
          if (currentMillis - previousMOTIONTime > MOTION_DETECT_INTERVAL) {
            previousMOTIONTime = currentMillis;
            String Mpayload = "{\"Motion\":";
            Mpayload += 1;
            Mpayload += "}";
            sendMQTT(MQTTTopic + "/PIR", Mpayload);
          }
        }
        else {
          //  No motion
          //  String Mpayload = "{\"Motion\":";
          //  Mpayload += 0;
          //  Mpayload += "}";
          //  sendMQTT(MQTTTopic + "/PIR", Mpayload);
        }
      }

      // DHT CODE (every 30 seconds)
      if (currentMillis - previousDHTTime > DHT_REPORT_INTERVAL) {
        previousDHTTime = currentMillis;
        float h = dht.readHumidity();
        float f = dht.readTemperature(true);
        float c = dht.readTemperature();
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
          // Tell CCS811 Temp in C and Humidity Percent..this may or may not work

          ccs811.set_envdata210(toCelsius((int)c, 1000), toPercentageH((int)h, 1000));
          sendMQTT(MQTTTopic + "/DHT22", payload);
          oldF = f;
          oldH = h;
        }

      }
    } else {
      Serial.println("MQTT Error, trying again in 5 seconds");
      delay(5000); // MQTT Probably down. Delay 1 second and don't execute any other code. If we can't get our data why bother
      if (!client.connected()) {
        if (client.connect((char*) clientName.c_str())) {
          Serial.println("Connected to MQTT broker");
        } else {
          Serial.println("Could not reconnect to MQTT broker");
        }
      }
    }
  } // END OTA UPDATE BYPASS
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

int32_t toCelsius(int t_data, int multiplier) {
  assert( (1 <= multiplier) && (multiplier <= 1024) );
  // Force 32 bits
  int32_t t = t_data & 0xFFFF;
  // Compensate for soldering effect
  t -= _soldercorrection;
  // Return m*C. This equals m*(K-273.15) = m*K - 27315*m/100 = m*t/64 - 27315*m/100
  // Note m is the multiplier, C is temperature in Celsius, K is temperature in Kelvin, t is raw t_data value.
  // Uses C=K-273.15 and K=t/64.
  return IDIV(multiplier * t, 64) - IDIV(27315L * multiplier, 100);
}

int32_t toPercentageH(int h_data, int multiplier) {
  assert( (1 <= multiplier) && (multiplier <= 1024) );
  // Force 32 bits
  int32_t h = h_data & 0xFFFF;
  // Return m*H. This equals m*(h/512) = (m*h)/512
  // Note m is the multiplier, H is the relative humidity in %RH, h is raw h_data value.
  // Uses H=h/512.
  return IDIV(multiplier * h, 512);
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

void update_started() {
  Serial.println("CALLBACK:  HTTP update process started");
}

void update_finished() {
  Serial.println("CALLBACK:  HTTP update process finished");
  OTAUPDATE = false; // Turn off  Update Flag, continue with regularly scheduled program
}

void update_progress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}

String sendStats() {
  int ourDBM;
 
  if (WiFi.status() != WL_CONNECTED)
    abort();
  
  int dBm = WiFi.RSSI();
  if (dBm <= -100)
    ourDBM = 0;
  if (dBm >= -50)
    ourDBM = 100;
  ourDBM =  2 * (dBm + 100);
  
   String payload = "{\"Signal\":";
         payload += ourDBM;
         payload += ",\"Uptime\":";
         payload += millis();
         payload += ",\"Hostname\":";
         payload += hostname;
         payload += ",\"IP\":";
         payload += WiFi.localIP().toString().c_str();
         payload += "}";
  return payload;       
}

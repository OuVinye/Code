#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP.h>
#include <SPIFFS.h>

#include <string.h>

#define M0 5
#define M1 4

int countReconnectWiFi = 0;
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long interval = 10;

const char* ssid = "AI FARM ROBOTICS FACTORY UNIFI"; // AI FARM ROBOTICS FACTORY UNIFI
const char* password = "@AIFARM2022";                // @AIFARM2022

const char* mqttServer = "demo.thingsboard.io"; // "thingsboard.cloud";
const int mqttPort = 1883;
// const char* accessToken = "8kybfxjsgw4ikbnkjm5j"; // me (WiFi Manager Device --> Dashboard: 	Show Data by Using WiFi Manager)
const char* accessToken = "UZ2FMO8O7g66VU99CRoQ"; 
const char* clientID = "ESP32_Client";

WiFiClient espClient;
PubSubClient client(espClient);

typedef struct MQ_series{
  uint8_t start;
	float CO;
	float H2;
	float LPG;
	float CO2;
	float NH4;
	float O3;
	float H2S;
	uint8_t PM1_0;
	uint8_t PM2_5;
	uint8_t PM10;

}MQ7_sensor;
MQ7_sensor SENSOR;
int size = sizeof(MQ7_sensor);

char buffer[128];

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial2.begin(9600);
  delay(50);

  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  digitalWrite(M0, 0);
  digitalWrite(M1, 0);

  // Wifi
  WiFi.begin(ssid, password); // Connect to Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting to WiFi ......");
    countReconnectWiFi += 1;
    delay(50);
    if(countReconnectWiFi > 20){
      Serial.println("Restarting ESP32 after disconnect wifi over 20 times......");
      esp_restart();
      delay(100);
    }
  }

  // Set up MQTT
  client.setServer(mqttServer, mqttPort);

  if (client.connect(clientID, accessToken, NULL)) {
    Serial.println("Connected to MQTT server");
  }
}

void loop() {
  currentTime = millis();

  size_t freeHeap = ESP.getFreeHeap();
  // Serial.print("Free Heap: ");
  Serial.println(freeHeap);
  // delay(50);

  // testWithoutReceiveDatafromLora();
  // receiveData();
  receive(&SENSOR);
  
  if(SENSOR.start == 1){
    // printReceiveData();
    if(currentTime - previousTime > 60){
      previousTime += interval;
      publishData();
      delay(50);
    } 
  }
  if (SENSOR.start != 1){
    esp_restart();
    delay(50);
  }
  if(currentTime - previousTime >= 90000){ // 90000
    Serial.println("######### Restarting every 15 minutes ########## ");
    esp_restart();
    delay(50);
  }
  if(freeHeap > 260000 | freeHeap == 259628 | freeHeap == 258044){ 
    esp_restart();
    delay(50);
  }
}

bool receive(MQ_series* table){
  return (Serial2.readBytes((char*)table, sizeof(MQ_series)) == sizeof(MQ_series));
}
void publishData(){
  // Adjust the buffer size as needed [7 = 128 default, 8=256, 9bit=512, 10-1024, 11=2048, 12=4096..]
  DynamicJsonDocument jsonDoc(1024); 

  jsonDoc["CO"] = String(SENSOR.CO, 2); // 2 decimal places
  jsonDoc["H2"] = String(SENSOR.H2, 2);
  jsonDoc["LPG"] = String(SENSOR.LPG, 2);

  jsonDoc["CO2"] = String(SENSOR.CO2, 2);
  jsonDoc["NH4"] = String(SENSOR.NH4, 2);
  jsonDoc["O3"] = String(SENSOR.O3, 2);

  jsonDoc["H2S"] = String(SENSOR.H2S, 2);
  jsonDoc["PM1_0"] = String(SENSOR.PM1_0);
  jsonDoc["PM2_5"] = String(SENSOR.PM2_5);

  jsonDoc["PM10"] = String(SENSOR.PM10);

  String jsonString;
  serializeJson(jsonDoc, jsonString);

  client.publish("v1/devices/me/telemetry", jsonString.c_str());
  delay(500);

}

/*
// Testing
void testWithoutReceiveDatafromLora(){
  SENSOR.CO = random(1, 100);
  SENSOR.H2 = random(1, 100);
  SENSOR.LPG = random(1, 100);

  SENSOR.CO2 = random(1, 100);
  SENSOR.NH4 = random(1, 100);
  SENSOR.O3 = random(1, 100);

  SENSOR.H2S = random(1, 100);
  SENSOR.PM1_0 = random(1, 100);
  SENSOR.PM2_5 = random(1, 100);

  SENSOR.PM10 = random(1, 100);

  // SENSOR.CO = 1.15;
  // SENSOR.H2 = 12.69;
  // SENSOR.LPG = 14.49;

  // SENSOR.CO2 = 405.8;
  // SENSOR.NH4 = 6.38;
  // SENSOR.O3 = 31.88;

  // SENSOR.H2S = 0.12;
  // SENSOR.PM1_0 = 28;
  // SENSOR.PM2_5 = 44;

  // SENSOR.PM10 = 53;

}
void receiveData(){
  while(1){
    if (Serial2.available() > 0) {
      byte buffer[size];
      int bytesRead = 0;
      while(bytesRead < size){
        if (Serial2.available()){
          buffer[bytesRead] = Serial2.read();
          bytesRead++;
        }
      }
      memcpy(&SENSOR, buffer, size);
      delay(50);
    }
  }
}
void printReceiveData(){
  Serial.print("Start: "+String(SENSOR.start)+" CO: "+String(SENSOR.CO)+" H2: "+String(SENSOR.H2)+" LPG: "+String(SENSOR.LPG)+" CO2: "+String(SENSOR.CO2)+" NH4: "+String(SENSOR.NH4)+" O3: "+String(SENSOR.O3)+" H2S: "+String(SENSOR.H2S)+" PM1_0: "+String(SENSOR.PM1_0)+" PM2_5: "+String(SENSOR.PM2_5)+" PM10: "+String(SENSOR.PM10));
  Serial.println();
  delay(1000);
}
*/

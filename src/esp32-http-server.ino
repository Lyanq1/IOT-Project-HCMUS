#include <WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>

const int DHT_pin = 14;

DHTesp dhtSensor;

const char* ssid = "Wokwi-GUEST";
const char* password = "";

//***Set server***
const char* mqttServer = "broker.hivemq.com"; 
int port = 1883;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void wifiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

void mqttConnect() {
  while(!mqttClient.connected()) {
    Serial.println("Attemping MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if(mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");

      //***Subscribe all topic you need***
      mqttClient.subscribe("/22127344/DoorControl");
    }
    else {
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

//MQTT Receiver
void callback(char* topic, byte* message, unsigned int length) {
  Serial.println(topic);
  String strMsg;
  for(int i=0; i<length; i++) {
    strMsg += (char)message[i];
  }
  Serial.println(strMsg);
  pinMode(15, OUTPUT);

  //***Code here to process the received package***
  if (strMsg == "on")
  {
    digitalWrite(15, HIGH);

  }
  else {
    digitalWrite(15,LOW);
  }
}

void setup() {
  Serial.begin(9600);
  Serial.print("Connecting to WiFi");

  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive( 90 );
  dhtSensor.setup(DHT_pin, DHTesp::DHT22);
}


void loop() {
  if(!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();

  //***Publish data to MQTT Server***
  int temp = random(0,100);
  char buffer[50];
  sprintf(buffer, "%d", temp);
  mqttClient.publish("/22127344/DoorControl", buffer);

  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  Serial.println("Temp: " + String(data.temperature, 2) + "C");
  Serial.println("Humid: " + String(data.humidity, 1) + "%");
  String tempStr = String(data.temperature, 2);
  String humidStr = String(data.humidity, 1);
  String payload = tempStr + "," + humidStr;
  mqttClient.publish("/SmartLock/Sensor/Temperature", payload.c_str());
  // mqttClient.publish("/SmartLock/Sensor/Humidity", humidStr.c_str());


  delay(5000);
}


#include <WiFi.h>

const char* ssid = "Wokwi-GUEST";
const char* password = "";

const char* host = "www.pushsafer.com";
const int port = 80;
const char* request = "/api?k=qDJi4rt2kNLkOOzwheFx&i=176&s=39&v=3&m=Testing";

void sendRequest() {
  WiFiClient client;
  while(!client.connect(host, port)) {
    Serial.println("connection fail");
    delay(1000);
  }
  client.print(String("GET ") + request + " HTTP/1.1\r\n"
              + "Host: " + host + "\r\n"
              + "Connection: close\r\n\r\n");
  delay(500);

  while(client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
}

void wifiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

void setup() {
  Serial.begin(9600);
  Serial.print("Connecting to WiFi");

  wifiConnect();
  sendRequest();
}

void loop() {
  
  delay(100);
}
// #include <WiFi.h>
// #include <PubSubClient.h>
// #include <DHTesp.h>

// const int DHT_pin = 14;
// DHTesp dhtSensor;

// const char* ssid = "Wokwi-GUEST";
// const char* password = "";

// const char* mqttServer = "broker.mqtt-dashboard.com";
// int port = 1883;

// WiFiClient wifiClient;
// PubSubClient mqttClient(wifiClient);

// void wifiConnect() {
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println(" Connected!");
// }

// void mqttConnect() {
//   while(!mqttClient.connected()) {
//     Serial.println("Attempting MQTT connection...");
//     String clientId = "ESP32Client-";
//     clientId += String(random(0xffff), HEX);
//     if(mqttClient.connect(clientId.c_str())) {
//       Serial.println("connected");
//       mqttClient.subscribe("/22127344/Control/DoorControl");
//     } else {
//       Serial.println("try again in 5 seconds");
//       delay(5000);
//     }
//   }
// }

// void callback(char* topic, byte* message, unsigned int length) {
//   Serial.println(topic);
//   String strMsg;
//   for(int i = 0; i < length; i++) {
//     strMsg += (char)message[i];
//   }
//   Serial.println(strMsg);

//   pinMode(15, OUTPUT);

//   // Process the received message
//   if (strMsg == "on") {
//     digitalWrite(15, HIGH);

//     // Publish data when "on" message is received
//     TempAndHumidity data = dhtSensor.getTempAndHumidity();
//     Serial.println("Temp: " + String(data.temperature, 2) + "C");
//     Serial.println("Humid: " + String(data.humidity, 1) + "%");
//     String payload = String(data.temperature, 2) + "," + String(data.humidity, 1);
//     mqttClient.publish("/SmartLock/Sensor/Temperature", payload.c_str());

//   } else if (strMsg == "off") {
//     digitalWrite(15, LOW);
//   }
// }

// void setup() {
//   Serial.begin(9600);
//   Serial.print("Connecting to WiFi");
//   wifiConnect();
//   mqttClient.setServer(mqttServer, port);
//   mqttClient.setCallback(callback);
//   mqttClient.setKeepAlive(90);
//   dhtSensor.setup(DHT_pin, DHTesp::DHT22);
// }

// void loop() {
//   if(!mqttClient.connected()) {
//     mqttConnect();
//   }
//   mqttClient.loop();

//   // No data publishing here; it will only occur in the callback function when "on" is received
//   delay(5000);
// }





// #include <WiFi.h>
// #include <PubSubClient.h>
// #include <DHTesp.h>

// const int DHT_pin = 14;

// DHTesp dhtSensor;

// const char* ssid = "Wokwi-GUEST";
// const char* password = "";

// //***Set server***
// // const char* mqttServer = "broker.hivemq.com"; 
// const char* mqttServer = "broker.mqtt-dashboard.com";
// int port = 1883;

// WiFiClient wifiClient;
// PubSubClient mqttClient(wifiClient);

// void wifiConnect() {
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println(" Connected!");
// }

// void mqttConnect() {
//   while(!mqttClient.connected()) {
//     Serial.println("Attemping MQTT connection...");
//     String clientId = "ESP32Client-";
//     clientId += String(random(0xffff), HEX);
//     if(mqttClient.connect(clientId.c_str())) {
//       Serial.println("connected");

//       //***Subscribe all topic you need***
//       mqttClient.subscribe("/22127344/Control/DoorControl");
//     }
//     else {
//       Serial.println("try again in 5 seconds");
//       delay(5000);
//     }
//   }
// }

// //MQTT Receiver
// void callback(char* topic, byte* message, unsigned int length) {
//   Serial.println(topic);
//   String strMsg;
//   for(int i=0; i<length; i++) {
//     strMsg += (char)message[i];
//   }
//   Serial.println(strMsg);
//   pinMode(15, OUTPUT);

//   //***Code here to process the received package***
//   if (strMsg == "on")
//   {
//     digitalWrite(15, HIGH);

//   }
//   else {
//     digitalWrite(15,LOW);
//   }
// }

// void setup() {
//   Serial.begin(9600);
//   Serial.print("Connecting to WiFi");

//   wifiConnect();
//   mqttClient.setServer(mqttServer, port);
//   mqttClient.setCallback(callback);
//   mqttClient.setKeepAlive( 90 );
//   dhtSensor.setup(DHT_pin, DHTesp::DHT22);
// }


// void loop() {
//   if(!mqttClient.connected()) {
//     mqttConnect();
//   }
//   mqttClient.loop();

//   //***Publish data to MQTT Server***
//   int temp = random(0,100);
//   char buffer[50];
//   // sprintf(buffer, "%d", temp);
//   sprintf(buffer, "%d");

//   mqttClient.publish("/22127344/Control/DoorControl", buffer);

//   TempAndHumidity data = dhtSensor.getTempAndHumidity();
//   Serial.println("Temp: " + String(data.temperature, 2) + "C");
//   Serial.println("Humid: " + String(data.humidity, 1) + "%");
//   String tempStr = String(data.temperature, 2);
//   String humidStr = String(data.humidity, 1);
//   String payload = tempStr + "," + humidStr;
//   mqttClient.publish("/SmartLock/Sensor/Temperature", payload.c_str());
//   // mqttClient.publish("/SmartLock/Sensor/Humidity", humidStr.c_str());


//   delay(5000);
// }



#include <WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// WiFi Credentials
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT Server Details
const char* mqttServer = "broker.mqtt-dashboard.com";
int port = 1883;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// DHT22 Configuration
DHTesp dht;
const int DHT_PIN = 14;

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Photoresistor Configuration
const int LDR_PIN = 34;  // Analog pin for LDR
const int LIGHT_THRESHOLD = 400;

// RGB LED Configuration
const int RED_PIN = 25;
const int GREEN_PIN = 32;
const int BLUE_PIN = 35;

// PIR Sensor Configuration
const int PIR_PIN = 12;
bool motionDetected = false;

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
    Serial.println("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if(mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");

      // Subscribe to topics
      mqttClient.subscribe("/22127344/DoorControl");
    }
    else {
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

// MQTT Callback
void callback(char* topic, byte* message, unsigned int length) {
  Serial.println(topic);
  String strMsg;
  for(int i = 0; i < length; i++) {
    strMsg += (char)message[i];
  }
  Serial.println(strMsg);
  pinMode(15, OUTPUT);
  
  // Process the received message
  if (strMsg == "on") {
    digitalWrite(15, HIGH);
  } else {
    digitalWrite(15, LOW);
  }
}

// Randomly change RGB color
void setRandomColor() {
  analogWrite(RED_PIN, random(0, 256));
  analogWrite(GREEN_PIN, random(0, 256));
  analogWrite(BLUE_PIN, random(0, 256));
}

void setup() {
  Serial.begin(115200);
  Serial.print("Connecting to WiFi");
  
  // WiFi and MQTT Setup
  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive(90);

  // DHT22 Setup
  dht.setup(DHT_PIN, DHTesp::DHT22);

  // LCD Setup
  lcd.init();
  lcd.backlight();
  
  // LED and PIR Setup
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);

  lcd.clear();
}

void loop() {
  if(!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();

  // Read DHT22 sensor
  TempAndHumidity data = dht.getTempAndHumidity();
  // float temperature = data.temperature;
  // float humidity = data.humidity;
  String temperatureStr = "Temp: " + String(data.temperature, 2) + " C";
  String humidityStr = "Humidity: " + String(data.humidity, 1) + "%";

  // Display temperature and humidity on LCD
  lcd.setCursor(0, 2); // Left bottom corner

  lcd.print(temperatureStr);


  lcd.setCursor(0, 3); // Right bottom corner
  lcd.print(humidityStr);
  
  // Read LDR value
    int ldrValue = analogRead(LDR_PIN);
    float voltage = ldrValue * 5/4095.0;
    float resistance = 2000 * voltage / (1 - voltage / 5);
    float lux = pow(50 * 1e3 * pow(10, 0.7) / resistance, (1 / 0.7));
  // Check light level
  if (lux < LIGHT_THRESHOLD) {  // It's dark
    lcd.setCursor(0, 0);
    lcd.print("Status: DARK");
    setRandomColor();
  } else {  // It's bright
    lcd.setCursor(0, 0);
    lcd.print("Status: LIGHT");
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 0);
  }

  // Detect motion
  motionDetected = digitalRead(PIR_PIN);
  if (motionDetected) {
    setRandomColor();
  }
  Serial.println(temperatureStr);
  Serial.println(humidityStr);
  Serial.println(lux);
  Serial.println("---");
  String payload = temperatureStr + "," + humidityStr;
mqttClient.publish("/SmartLock/Sensor/Temperature", payload.c_str());
  delay(5000);
}
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>  // Include the ESP32Servo library

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
const int GREEN_PIN = 15;
const int BLUE_PIN = 35;

// PIR Sensor Configuration
const int PIR_PIN = 12;
bool motionDetected = false;
// Servo, Buzzer, and LED Configuration
Servo myServo;  // ESP32Servo object
const int SERVO_PIN = 16;
const int BUZZER_PIN = 2;
const int LED_PIN = 4;  // Separate LED pin

// Button Configuration
const int BUTTON_PIN = 13;
bool previousButtonState = false;

bool sensorOn = true;


int userChoice = 0;

bool doorOpened = false;
bool accessAllowed = false;
// Predefined RFID IDs in the database
String validRFID1 = "66b71e88c1c0fb673c77c6c1";
String validRFID2 = "66c4b24474ba516a448ee024";

void wifiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

void mqttConnect() {
  while (!mqttClient.connected()) {
    Serial.println("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected");

      // Subscribe to topics
      mqttClient.subscribe("/SmartLock/Control/DoorControl");
      mqttClient.subscribe("/SmartLock/RFID/Accessed");
      mqttClient.subscribe("/SmartLock/Sensor/Control");

    } else {
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}

// MQTT Callback
void callback(char* topic, byte* message, unsigned int length) {
    Serial.println(topic);
    String strMsg;
    for (int i = 0; i < length; i++) {
        strMsg += (char)message[i];
    }
    Serial.println(strMsg);

    if (String(topic) == "/SmartLock/Control/DoorControl") {
        if (strMsg == "Door open") {
            // Rotate servo to 90 degrees without resetting
            myServo.write(0);
            Serial.println("Door Opened");
        } else if (strMsg == "Door locked") {
            myServo.write(90);
            Serial.println("Door Locked");
        }
    } else if (String(topic) == "/SmartLock/RFID/Accessed") {
        if (strMsg == "Access granted") {
            Serial.println("Valid RFID! Access Granted.");
            
            // Thực hiện các hành động như xoay servo, bật đèn, bật loa
            myServo.write(90);
            digitalWrite(LED_PIN, HIGH);  // Bật đèn LED
            tone(BUZZER_PIN, 1000);       // Kích hoạt buzzer với tần số 1kHz

            delay(2000);  // Giữ các thiết bị hoạt động trong 2 giây
            
            // Reset lại servo, đèn và buzzer
            myServo.write(0);
            delay(500);
            digitalWrite(LED_PIN, LOW);   // Tắt đèn LED
            noTone(BUZZER_PIN);

            Serial.println("Servo, LED, và Buzzer đã được reset.");
            
        } else if (strMsg == "Access Denied") {
            Serial.println("Invalid RFID! Access Denied.");
        }
    } else if (String(topic) == "/SmartLock/Sensor/Control") {
        if (strMsg == "Sensor on") {
            sensorOn = true;  // Enable publishing of sensor data
Serial.println("Temperature Sensor turned on.");
        } else if (strMsg == "Sensor off") {
            sensorOn = false;  // Stop publishing sensor data
            Serial.println("Temperature Sensor turned off.");
        }
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

  // LED, PIR, Servo, Button, and Buzzer Setup
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);  // Setup the separate LED pin
  myServo.attach(SERVO_PIN);  // Attach the servo to the ESP32 pin
  pinMode(BUZZER_PIN, OUTPUT);

  lcd.clear();
  Serial.println("Please enter 0 or 1 to select the mode:");
  while (true) {
    if (Serial.available() > 0) {
        char input = Serial.read();  // Đọc một ký tự từ Serial
        if (input == '0' || input == '1') {
            userChoice = input - '0';  // Chuyển đổi ký tự thành số nguyên (0 hoặc 1)
            Serial.print("User choice set to: ");
            Serial.println(userChoice);
            break;  // Thoát vòng lặp khi người dùng đã nhập giá trị hợp lệ
        } else {
            Serial.println("Invalid input! Please enter 0 or 1.");
        }
    }
  }
}


void loop() {
  if (!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();
  // Read DHT22 sensor
        TempAndHumidity data = dht.getTempAndHumidity();
        String temperatureStr = "Temp: " + String(data.temperature, 2) + " C";
        String humidityStr = "Humidity: " + String(data.humidity, 1) + "%";

        // Display temperature and humidity on LCD
        lcd.setCursor(0, 2); // Left bottom corner
        lcd.print(temperatureStr);
        lcd.setCursor(0, 3); // Right bottom corner
        lcd.print(humidityStr);

  if (sensorOn) {
        
        // Publish sensor data to MQTT server
        String payload = String(data.temperature, 2) + "," + String(data.humidity, 1);
        mqttClient.publish("/SmartLock/Sensor/Temperature", payload.c_str());

        Serial.println("Published temperature and humidity data.");
    }

  // Read LDR value
  int ldrValue = analogRead(LDR_PIN);
  float voltage = ldrValue * 5 / 4095.0;
  float resistance = 2000 * voltage / (1 - voltage / 5);
  float lux = pow(50 * 1e3 * pow(10, 0.7) / resistance, (1 / 0.7));

  // Check light level
  if (lux < LIGHT_THRESHOLD) {  // It's dark
    lcd.setCursor(0, 0);
    lcd.print("Status: DARK ");
setRandomColor();
  } else {  // It's bright
    lcd.setCursor(0, 0);
    lcd.print("Status: LIGHT ");
    analogWrite(RED_PIN, 0);
    analogWrite(GREEN_PIN, 0);
    analogWrite(BLUE_PIN, 0);
  }

  // Detect motion
  motionDetected = digitalRead(PIR_PIN);
  if (motionDetected) {
    setRandomColor();
  }
  
  if (Serial.available() > 0) {
        char input = Serial.read();  // Read a character from Serial
        if (input == '0' || input == '1') {
            userChoice = input - '0';  // Convert char to int (0 or 1)
            Serial.print("User choice set to: ");
            Serial.println(userChoice);
        } else {
            Serial.println("Invalid input! Please enter 0 or 1.");
        }
    }
    
   bool currentButtonState = digitalRead(BUTTON_PIN);
    if (currentButtonState == LOW && previousButtonState == false) {
        previousButtonState = true;  // Đánh dấu nút đã được nhấn

        // Phát sinh ngẫu nhiên hoặc sử dụng RFID tag cố định dựa trên lựa chọn của người dùng
        String rfidTag;
        if (userChoice == 0) {
            // Tạo mã RFID ngẫu nhiên và giả định rằng server sẽ trả về "Access Denied"
            rfidTag = String(random(0xffff), HEX) + String(random(0xffff), HEX);
        } else if (userChoice == 1) {
            // Mã RFID cố định
            rfidTag = "66c8c87cba896247f462e724";
             
        }
        
        mqttClient.publish("/SmartLock/RFID", rfidTag.c_str());

        Serial.println("RFID Scanned: " + rfidTag);

    } else if (currentButtonState == HIGH && previousButtonState == true) {
        previousButtonState = false;  // Reset trạng thái nút khi nút được nhả

    }

    delay(5000);  // Điều chỉnh delay theo nhu cầu
}




















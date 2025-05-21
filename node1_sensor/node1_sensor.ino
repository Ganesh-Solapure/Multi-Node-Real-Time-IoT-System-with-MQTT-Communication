/*
   Copyright [2025] [Ganesh Solapure]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>

// WiFi credentials
const char* ssid = "1136";
const char* password = "wifi8889";

// MQTT Broker settings
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_Sensor_Node";

// Pin definitions
#define DHT_PIN 4
#define DHTTYPE DHT11
#define TRIG_PIN 5
#define ECHO_PIN 18
#define IR_PIN 34  // Changed to analog pin
#define IR_THRESHOLD 2000  // Adjust this threshold based on your sensor

// Sound speed definitions
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

// MPU6050 settings
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int minVal = 265;
int maxVal = 402;
double x, y, z;

// Initialize objects
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHT_PIN, DHTTYPE);

// Timing variables
unsigned long lastMsg = 0;
const long interval = 2000; // Send data every 2 seconds
unsigned long lastWifiCheck = 0;
const long wifiCheckInterval = 30000; // Check WiFi every 30 seconds

// Connection status
bool wifiConnected = false;
bool mqttConnected = false;

void setup_wifi() {
  Serial.println("Connecting to CAN...");
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nCAN connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength (RSSI): ");
    Serial.println(WiFi.RSSI());
  } else {
    wifiConnected = false;
    Serial.println("\nCAN connection failed!");
  }
}

void reconnect() {
  if (!wifiConnected) {
    setup_wifi();
    return;
  }

  if (!client.connected()) {
    Serial.print("Attempting CAN connection...");
    if (client.connect(mqtt_client_id)) {
      mqttConnected = true;
      Serial.println("connected");
    } else {
      mqttConnected = false;
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void checkConnections() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastWifiCheck >= wifiCheckInterval) {
    lastWifiCheck = currentMillis;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("CAN connection lost. Reconnecting...");
      wifiConnected = false;
      setup_wifi();
    }
  }

  if (!client.connected()) {
    mqttConnected = false;
    reconnect();
  }
}

float getDistance() {
  // Clears the trigPin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate the distance
  float distanceCm = duration * SOUND_SPEED/2;
  float distanceInch = distanceCm * CM_TO_INCH;
  
  // Print to Serial Monitor
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);
  Serial.print("Distance (inch): ");
  Serial.println(distanceInch);
  
  return distanceCm;
}

void updateMPUAngles() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  
  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);
  
  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);
  
  // Print to Serial Monitor
  Serial.println("MPU6050 Angles:");
  Serial.print("X: "); Serial.print(x); Serial.println("°");
  Serial.print("Y: "); Serial.print(y); Serial.println("°");
  Serial.print("Z: "); Serial.print(z); Serial.println("°");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\nInitializing Sensor Node...");
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  // Initialize other sensors
  dht.begin();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(60);
}

void loop() {
  checkConnections();
  
  if (wifiConnected && mqttConnected) {
    client.loop();

    unsigned long currentMillis = millis();
    if (currentMillis - lastMsg >= interval) {
      lastMsg = currentMillis;

      // Read DHT sensor
      float h = dht.readHumidity();
      float t = dht.readTemperature();
      float f = dht.readTemperature(true);

      // Check if DHT read failed
      if (isnan(h) || isnan(t) || isnan(f)) {
        Serial.println("Failed to read from DHT sensor!");
      } else {
        // Calculate heat index
        float hic = dht.computeHeatIndex(t, h, false);
        
        // Print DHT values
        Serial.print("Humidity: ");
        Serial.print(h);
        Serial.print("%  Temperature: ");
        Serial.print(t);
        Serial.print("°C ");
        Serial.print(f);
        Serial.print("°F  Heat index: ");
        Serial.print(hic);
        Serial.println("°C");
        
        // Publish DHT data
        client.publish("sensors/temperature", String(t).c_str());
        client.publish("sensors/humidity", String(h).c_str());
      }

      // Read distance
      float distance = getDistance();
      client.publish("sensors/distance", String(distance).c_str());

      // Read IR sensor (analog)
      int irValue = analogRead(IR_PIN);
      String irStatus = (irValue > IR_THRESHOLD) ? "YES" : "NO";
      
      // Print IR values
      Serial.print("IR Sensor - Raw Value: ");
      Serial.print(irValue);
      Serial.print(" Status: ");
      Serial.println(irStatus);
      
      // Publish IR data
      client.publish("sensors/ir_value", String(irValue).c_str());
      client.publish("sensors/ir_status", irStatus.c_str());
      
      // Update and publish MPU angles
      updateMPUAngles();
      client.publish("sensors/gyro_x", String(x).c_str());
      client.publish("sensors/gyro_y", String(y).c_str());
      client.publish("sensors/gyro_z", String(z).c_str());
    }
  } else {
    delay(1000);
  }
} 
/*
   Copyright [2025] [Ganesh Solapure]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// WiFi credentials
const char* ssid = "1136";
const char* password = "wifi8889";

// MQTT Broker settings
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_Monitor_Node";

// LCD settings
#define I2C_ADDR 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_ROWS);

// Initialize objects
WiFiClient espClient;
PubSubClient client(espClient);

// Sensor values
float temperature = 0;
float humidity = 0;
float distance = 0;
int irValue = 0;
String irStatus = "NO";
float gyroXangle = 0;
float gyroYangle = 0;
float gyroZangle = 0;

// Display mode
bool showMPU = false;
unsigned long lastDisplayChange = 0;
const long displayInterval = 2000; // Switch display every 2 seconds

// Connection management
unsigned long lastWifiCheck = 0;
const long wifiCheckInterval = 30000; // Check WiFi every 30 seconds
bool wifiConnected = false;
bool mqttConnected = false;

// Data validation
bool dataUpdated = false;
unsigned long lastDataUpdate = 0;
const long dataTimeout = 5000; // Consider data stale after 5 seconds

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

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  if (String(topic) == "sensors/temperature") {
    temperature = message.toFloat();
    dataUpdated = true;
    lastDataUpdate = millis();
  }
  else if (String(topic) == "sensors/humidity") {
    humidity = message.toFloat();
    dataUpdated = true;
    lastDataUpdate = millis();
  }
  else if (String(topic) == "sensors/distance") {
    distance = message.toFloat();
    dataUpdated = true;
    lastDataUpdate = millis();
  }
  else if (String(topic) == "sensors/ir_value") {
    irValue = message.toInt();
    dataUpdated = true;
    lastDataUpdate = millis();
  }
  else if (String(topic) == "sensors/ir_status") {
    irStatus = message;
    dataUpdated = true;
    lastDataUpdate = millis();
  }
  else if (String(topic) == "sensors/gyro_x") {
    gyroXangle = message.toFloat();
    dataUpdated = true;
    lastDataUpdate = millis();
  }
  else if (String(topic) == "sensors/gyro_y") {
    gyroYangle = message.toFloat();
    dataUpdated = true;
    lastDataUpdate = millis();
  }
  else if (String(topic) == "sensors/gyro_z") {
    gyroZangle = message.toFloat();
    dataUpdated = true;
    lastDataUpdate = millis();
  }
  
  updateDisplay();
}

void updateDisplay() {
  unsigned long currentMillis = millis();
  
  // Check if data is stale
  if (currentMillis - lastDataUpdate > dataTimeout) {
    dataUpdated = false;
  }
  
  // Switch display mode every 2 seconds
  if (currentMillis - lastDisplayChange >= displayInterval) {
    lastDisplayChange = currentMillis;
    showMPU = !showMPU;
  }
  
  lcd.clear();
  
  if (!dataUpdated) {
    lcd.setCursor(0, 0);
    lcd.print("Waiting for data");
    lcd.setCursor(0, 1);
    lcd.print("...");
    return;
  }
  
  if (showMPU) {
    // Display MPU6050 angle data
    lcd.setCursor(0, 0);
    lcd.print("X:");
    lcd.print(gyroXangle, 1);
    lcd.print(" Y:");
    lcd.print(gyroYangle, 1);
    
    lcd.setCursor(0, 1);
    lcd.print("Z:");
    lcd.print(gyroZangle, 1);
    lcd.print(" deg");
  } else {
    // Display other sensor data
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(temperature, 1);
    lcd.print("C H:");
    lcd.print(humidity, 1);
    lcd.print("%");
    
    lcd.setCursor(0, 1);
    lcd.print("D:");
    lcd.print(distance, 1);
    lcd.print("cm IR:");
    lcd.print(irStatus);
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
      // Subscribe to sensor topics
      client.subscribe("sensors/temperature");
      client.subscribe("sensors/humidity");
      client.subscribe("sensors/distance");
      client.subscribe("sensors/ir_value");
      client.subscribe("sensors/ir_status");
      client.subscribe("sensors/gyro_x");
      client.subscribe("sensors/gyro_y");
      client.subscribe("sensors/gyro_z");
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

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\nInitializing Monitor Node...");
  
  // Initialize LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setKeepAlive(60);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");
  delay(1000);
}

void loop() {
  checkConnections();
  
  if (wifiConnected && mqttConnected) {
    client.loop();
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connecting...");
    delay(1000);
  }
} 
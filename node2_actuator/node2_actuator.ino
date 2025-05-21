/*
   Copyright [2025] [Ganesh Solapure]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// WiFi credentials
const char* ssid = "1136";
const char* password = "wifi8889";

// MQTT Broker settings
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_Actuator_Node";

// Pin definitions
#define RELAY_PIN 4
#define BUZZER_PIN 5
#define LED_PIN 18
#define MOTOR_PIN 19
#define SERVO_PIN 23

// Initialize objects
WiFiClient espClient;
PubSubClient client(espClient);
Servo motor;     // For DC motor control
Servo servo;     // For IR-based control

// Servo positions
#define SERVO_OPEN 180
#define SERVO_CLOSED 0

// Temperature thresholds
#define TEMP_THRESHOLD 30.0
#define HUMIDITY_THRESHOLD 70.0

void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  if (String(topic) == "sensors/temperature") {
    float temperature = message.toFloat();
    // Control relay based on temperature
    if (temperature > TEMP_THRESHOLD) {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Temperature high - Relay ON");
    } else {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Temperature normal - Relay OFF");
    }
  }
  else if (String(topic) == "sensors/humidity") {
    float humidity = message.toFloat();
    // Control buzzer based on humidity
    if (humidity > HUMIDITY_THRESHOLD) {
      digitalWrite(BUZZER_PIN, HIGH);
      Serial.println("Humidity high - Buzzer ON");
    } else {
      digitalWrite(BUZZER_PIN, LOW);
      Serial.println("Humidity normal - Buzzer OFF");
    }
  }
  else if (String(topic) == "sensors/distance") {
    float distance = message.toFloat();
    // Control LED and buzzer based on distance
    if (distance < 10.0) {
      digitalWrite(LED_PIN, HIGH);
      if (distance < 5.0) {
        digitalWrite(BUZZER_PIN, HIGH);
        Serial.println("Object very close - LED and Buzzer ON");
      } else {
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("Object near - LED ON, Buzzer OFF");
      }
    } else {
      digitalWrite(LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      Serial.println("No object - LED and Buzzer OFF");
    }
  }
  else if (String(topic) == "sensors/ir") {
    int irState = message.toInt();
    // Control servo based on IR sensor
    if (irState == 1) {
      servo.write(SERVO_OPEN);
      Serial.println("IR Detected - Servo Opening");
    } else {
      servo.write(SERVO_CLOSED);
      Serial.println("No IR Detection - Servo Closing");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_client_id)) {
      Serial.println("connected");
      // Subscribe to sensor topics
      client.subscribe("sensors/temperature");
      client.subscribe("sensors/humidity");
      client.subscribe("sensors/distance");
      client.subscribe("sensors/ir");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize motors
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  
  // Setup DC motor
  motor.setPeriodHertz(50);
  motor.attach(MOTOR_PIN);
  
  // Setup servo motor
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN);
  servo.write(SERVO_CLOSED);  // Start in closed position
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
} 
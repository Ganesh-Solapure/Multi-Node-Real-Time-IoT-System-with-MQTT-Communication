# IoT-Based Automation with Multi-Node MQTT Communication

## Personal Project | 2025

This project showcases a robust, **3-node Internet of Things (IoT) system** designed for real-time sensor data acquisition, intelligent actuator control, and comprehensive monitoring, all powered by **MQTT (Message Queuing Telemetry Transport)** communication.

---

## 💡 Project Highlights

* **Multi-Node Architecture:** Engineered a distributed IoT system featuring dedicated sensor, actuator, and monitor nodes for efficient data flow and processing.
* **Real-time Data:** Leveraged MQTT for seamless and reliable publication of diverse sensor data, including temperature, humidity, distance, infrared, and gyroscope readings.
* **Intelligent Automation:** Implemented conditional control logic on the actuator node, enabling automated responses to sensor inputs, such as relay activation, buzzer alerts, LED indications, and precise motor control.
* **Live Monitoring:** Developed a dedicated monitor node to display live sensor values and MPU6050 gyroscope data on an I2C LCD, providing immediate system insights.
* **Robust Connectivity:** Ensured dependable Wi-Fi connectivity across all nodes and utilized topic-based messaging for organized and scalable communication.
* **Modular Firmware:** Developed flexible and maintainable firmware using the **Arduino IDE**, simplifying future expansions and modifications.

---

## 🚀 Technologies Used

* **MQTT:** Lightweight messaging protocol for IoT communication.
* **ESP32:** Powerful Wi-Fi enabled microcontroller for all nodes.
* **Wi-Fi:** Wireless connectivity for inter-node communication.
* **DHT11:** Temperature and humidity sensor.
* **HC-SR04:** Ultrasonic distance sensor.
* **IR Sensor:** Infrared proximity sensor.
* **MPU6050:** 3-axis accelerometer and gyroscope for motion sensing.
* **LCD (I2C):** 16x2 character display for monitoring.
* **Servo Motor:** For precise angular control.
* **DC Motor:** For rotational movement with PWM control.

---

## 🏗️ System Architecture

Our IoT system is comprised of three distinct nodes, each with a specialized role:

### Node 1: Sensor Node

This node is the data collection hub, continuously gathering environmental and motion data.

* **DHT11:** Measures ambient temperature and humidity.
* **HC-SR04:** Detects object distance.
* **IR Sensor:** Identifies object presence.
* **MPU6050:** Provides 3-axis angular velocity data (gyroscope).
* **ESP32 Wi-Fi Module:** Handles wireless communication.

### Node 2: Actuator Node

The brain of our automation, this node subscribes to sensor data and controls various outputs.

* **Relay:** Controls high-power devices.
* **Buzzer:** Provides audible alerts.
* **LED:** Offers visual feedback.
* **PWM DC Motor:** Controls speed of a DC motor.
* **Servo Motor:** Executes precise angular movements.
* **ESP32 Wi-Fi Module:** Manages wireless communication.

### Node 3: Monitor Node

This node provides a real-time visual interface for the system's state.

* **I2C 16x2 LCD Display:** Shows live sensor readings.
* **ESP32 Wi-Fi Module:** Facilitates wireless communication.

---

## 📡 MQTT Topic Structure

Our MQTT implementation uses a clear, hierarchical topic structure for efficient data exchange:

### Sensor Data Topics

* `sensors/temperature`: Temperature readings from DHT11.
* `sensors/humidity`: Humidity readings from DHT11.
* `sensors/distance`: Distance readings from HC-SR04.
* `sensors/ir`: State of the IR sensor.
* `sensors/gyro_x`: MPU6050 X-axis angular velocity.
* `sensors/gyro_y`: MPU6050 Y-axis angular velocity.
* `sensors/gyro_z`: MPU6050 Z-axis angular velocity.

---

## 🔌 Hardware Connections

Careful wiring is crucial for proper operation. Here's a detailed breakdown of connections for each node:

### Node 1 (Sensor Node)

* **DHT11:**
    * VCC → 3.3V
    * DATA → **GPIO 4**
    * GND → GND
* **HC-SR04:**
    * TRIG → **GPIO 5**
    * ECHO → **GPIO 18**
    * VCC → 5V
    * GND → GND
* **IR Sensor:**
    * VCC → 3.3V
    * OUT → **GPIO 19**
    * GND → GND
* **MPU6050:**
    * VCC → 3.3V
    * GND → GND
    * SDA → **GPIO 21**
    * SCL → **GPIO 22**

### Node 2 (Actuator Node)

* **Relay:**
    * IN → **GPIO 23**
    * VCC → 5V
    * GND → GND
* **Buzzer:**
    * IN → **GPIO 22**
    * VCC → 5V
    * GND → GND
* **LED:**
    * IN → **GPIO 21**
    * GND → GND (with 220Ω resistor)
* **DC Motor:**
    * IN1 → **GPIO 18**
    * IN2 → **GPIO 19**
    * VCC → 5V
    * GND → GND
* **Servo Motor:**
    * Signal → **GPIO 5**
    * VCC → 5V
    * GND → GND

### Node 3 (Monitor Node)

* **I2C LCD:**
    * VCC → 5V
    * GND → GND
    * SDA → **GPIO 21**
    * SCL → **GPIO 22**

---

## ⚙️ System Operation

Dive deeper into how each node functions within the system:

### Sensor Node (Node 1)

1.  Continuously reads data from all connected sensors (DHT11, HC-SR04, IR, MPU6050).
2.  Publishes the collected sensor data to their respective MQTT topics **every 2 seconds**.
3.  The MPU6050 provides angular velocity data in **radians per second**.

### Actuator Node (Node 2)

1.  Subscribes to all relevant sensor data topics from the MQTT broker.
2.  Controls actuators based on predefined conditions derived from sensor readings:
    * **Relay:** Activates when temperature exceeds **30°C**.
    * **Buzzer:** Sounds an alert when humidity surpasses **70%**.
    * **LED:** Illuminates when an object is detected within **20cm**.
    * **Servo Motor:** Adjusts its position based on the state of the IR sensor (e.g., opens/closes a gate).
    * **DC Motor:** Its speed is dynamically adjusted in response to temperature fluctuations (e.g., fan speed control).

### Monitor Node (Node 3)

1.  Subscribes to all sensor data topics from the MQTT broker.
2.  Displays the live sensor data on the I2C LCD.
3.  Alternates the displayed information **every 2 seconds**:
    * **First screen:** Shows Temperature, Humidity, Distance, and IR sensor state.
    * **Second screen:** Displays MPU6050 gyroscope data (X, Y, Z angular velocities).

---

## 🛠️ Setup Instructions

To get this project up and running, follow these simple steps:

1.  **Install Required Libraries:** Open your Arduino IDE and install the following libraries via the Library Manager (`Sketch > Include Library > Manage Libraries...`):
    * `PubSubClient` by Nick O'Leary
    * `DHT sensor library` by Adafruit
    * `Adafruit MPU6050` by Adafruit
    * `Adafruit Unified Sensor` by Adafruit
    * `LiquidCrystal I2C` by Frank de Brabander

2.  **Configure Wi-Fi Credentials:** In the code for each node, locate the section where Wi-Fi credentials are defined and update them with your network's **SSID** and **password**.

3.  **Upload Firmware:** Upload the respective code (`sensor_node.ino`, `actuator_node.ino`, `monitor_node.ino` - *filenames are examples, adjust as per your actual project files*) to each of your ESP32 boards.

4.  **Power On:** Once the code is uploaded, power on all three ESP32 nodes.

5.  **Monitor (Optional):** For initial debugging and verification, open the Serial Monitor in the Arduino IDE for each node to observe their output and connection status.

---

## ✅ Testing Your Setup

To ensure everything is working as expected:

1.  **Verify MQTT Connection:** Confirm that all nodes successfully connect to your MQTT broker. Check the Serial Monitor output for connection messages.
2.  **Check Sensor Readings:** Observe the Serial Monitor output from the Sensor Node to ensure accurate sensor readings are being published.
3.  **Verify Actuator Responses:** Interact with the sensors (e.g., change temperature, bring an object close to the IR sensor) and confirm that the actuators on the Actuator Node respond as programmed.
4.  **Confirm LCD Display Updates:** Watch the I2C LCD on the Monitor Node to ensure it's displaying live sensor values and periodically updating with gyroscope data.

---

## ⚠️ Troubleshooting Tips

If you encounter any issues, consider these common problems and solutions:

1.  **Wi-Fi Connectivity:**
    * Double-check your **SSID and password** in the code.
    * Ensure your ESP32 boards are within range of your Wi-Fi router.
    * Restart your Wi-Fi router.
2.  **MQTT Broker Connection:**
    * Verify that your MQTT broker is running and accessible.
    * Confirm the MQTT broker's **IP address or hostname** in your code is correct.
    * Check for firewall issues that might block MQTT ports.
3.  **Incorrect Pin Connections:**
    * Carefully review the **Hardware Connections** section and verify that all sensors and actuators are wired to the correct GPIO pins on each ESP32.
4.  **Insufficient Power Supply:**
    * Ensure that all components, especially the actuators and sensors, are receiving adequate power (voltage and current). Some components might require external power sources.

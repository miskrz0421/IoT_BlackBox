# BlackBox IoT Project

**BlackBox** is an Internet of Things (IoT) system designed to monitor environmental parameters during the transportation of packages. It enables real-time tracking of temperature, pressure, linear acceleration, angular velocity, and infrared radiation, ensuring the integrity and safety of transported goods.

## Features

- **Environmental Monitoring:** Records temperature, pressure, linear acceleration, angular velocity, and infrared radiation.
- **Time Synchronization:** Uses the SNTP protocol for precise timestamping of measurements.
- **Remote Configuration:** Allows configuration via Bluetooth Low Energy (BLE) and MQTT.
- **Data Transmission:** Sends collected data to a server via MQTT.
- **User Management:** A web application enables user registration, device management, and data visualization.

## Hardware Components

- **Microcontroller:** ESP32 S3 N16R8 with 16 MB FLASH memory and 8 MB PSRAM.
- **Sensors:**
  - **BMP280:** Temperature and pressure sensor.
  - **MPU6050:** Accelerometer and gyroscope.
  - **KY026:** Infrared sensor.
- **LEDs:** Three LEDs connected to GPIO pins 7, 16, and 17 for status indication.
- **Button:** One button connected to GPIO pin 6 for user input.

## System Architecture

- **ESP32 S3 Microcontroller:** Central unit managing sensors, communication, and data storage.
- **Sensors:** Collect environmental data.
- **BLE Module:** Used for initial device configuration.
- **Wi-Fi Module:** Enables MQTT communication and time synchronization.
- **Web Application:** Allows users to manage devices, configure sensors, and view data.

## Device States

The device can be in one of five states, each indicated by a different LED color:

1. **DEVICE_STATE_NOT_CONFIGURED (-1)** - **Orange:** The device is not assigned to a user. Configuration via BLE is required (Wi-Fi, MQTT broker, username, PIN).
2. **DEVICE_STATE_CONFIGURING (0)** - **Blue:** The device is assigned to a user and awaits sensor configuration via the web application and MQTT.
3. **DEVICE_STATE_READY_TO_TRAVEL (1)** - **Yellow:** The device is configured and ready for travel. Travel mode is activated by holding the BOOT button for 3 seconds.
4. **DEVICE_STATE_TRAVELLING (2)** - **Green:** The device collects and stores sensor data during travel.
5. **DEVICE_STATE_AFTER_TRAVEL (3)** - **Purple:** The device sends collected data to the server via MQTT.

## User Management

To reset the device to the unconfigured state (-1) and allow assignment to a new user, press the dedicated red button. This will:

- Change the state to -1.
- Delete all configuration data from NVS.
- Automatically remove the device from the previous user's account upon assignment to a new user (after PIN verification).

## Power Loss Handling

The device's state is saved in non-volatile memory (NVS), ensuring it is retained after a restart. Depending on the state at the time of power loss:

- **State -1:** Retains partial configuration; only missing data needs to be provided.
- **State 0:** Attempts to reconnect to Wi-Fi and MQTT, awaiting sensor configuration.
- **State 1:** Attempts to synchronize time via SNTP; configuration data is read from NVS.
- **State 2:** Stops collecting new data; data saved in FLASH is preserved.
- **State 3:** Attempts to reconnect to Wi-Fi and MQTT to send data from FLASH.

## Sensors

### BMP280 - Temperature and Pressure Sensor

- **Communication:** I2C (SCL: GPIO 9, SDA: GPIO 8).
- **Configurable Parameters:**
  - Temperature oversampling.
  - Pressure oversampling.
  - IIR filter.
  - Measurement frequency.
- **Note:** The BMP280 library was developed by authors.

### MPU6050 - Accelerometer and Gyroscope

- **Communication:** I2C (SCL: GPIO 9, SDA: GPIO 8).
- **Configurable Parameters:**
  - Gyroscope range (±250°/s to ±2000°/s).
  - Accelerometer range (±2g to ±16g).
  - Low-pass filter.

### KY026 - Infrared Sensor

- **Connection:** Analog output to ADC 1 (GPIO 1), digital output to GPIO 4.
- **Function:** Detects infrared radiation, e.g., to signal package opening.

## Inputs and Outputs

### Digital Inputs

- **NVS Clear Button:** Resets NVS and restores the unconfigured state.
- **BOOT Button:**
  - Short press: Toggles BLE advertising.
  - Hold for 3 seconds: Starts/ends travel mode.

### Digital Outputs

- **RGB LED:** Indicates the current device state.
- **Red LED:** Indicates BLE advertising status.
- **Yellow LED:** Flashes when starting/ending travel and during data transmission.
- **Green LED:** Lights up when connected to Wi-Fi and MQTT.

## Bluetooth Low Energy (BLE)

BLE is used for initial device configuration:

- Setting Wi-Fi SSID and password.
- Setting MQTT broker URL.
- Setting username.
- Sending PIN for user verification.

Configuration data (except PIN) is stored in NVS. BLE advertising is enabled by pressing the BOOT button, indicated by the red LED.

## Non-Volatile Memory

### NVS

Stores key configuration data:

- Device state.
- Wi-Fi SSID and password.
- Username.
- MQTT broker URL.
- Sensor configuration.

### FLASH

Stores sensor data during travel:

- Data is buffered in PSRAM and periodically written to FLASH.
- Up to 12 MB of data (4 MB per sensor).
- Data is preserved after power loss.

After travel, data from FLASH is combined with PSRAM data and sent to the server.

## MQTT Communication

MQTT is used for:

- User verification (PIN).
- Receiving sensor configuration.
- Sending data to the server.

### Topic Structure

Format: `BlackBox/{User}/{MAC}/{Type}`

- **Control Topics:**
  - `.../Pin`: PIN verification.
  - `... Response`: Verification response.
  - `.../Config`: Sensor configuration.
  - `.../ConfigResponse`: Configuration confirmation.
  - `.../Control`: Data transmission control.
- **Data Topics:**
  - `.../ky026`: Infrared data.
  - `.../bmp280`: Temperature and pressure data.
  - `.../mpu6050`: Acceleration and gyroscope data.

### Message Format

Data is sent in JSON format, including:

- Timestamp.
- Sensor-specific measurements.

## Web Application Features

The web application provides:

- **User Management:**
  - Registration and login.
  - Device management (add, remove, rename).
- **Travel Management:**
  - Creating new travels.
  - Configuring sensor parameters.
  - Viewing travel history.
- **Data Visualization:**
  - Data presentation in tables.
  - Automatic generation of charts (temperature, pressure, acceleration, angular velocity).
  - Data export to CSV.

The server for the BlackBox IoT project is located in a separate repository. You can access it via the link below:

- [BlackBox Server Repository](https://github.com/Tomciom/IoT_WebServer)

## Development Environment

The project was developed using the **Espressif IoT Development Framework (IDF)**. To build and flash the firmware to the ESP32 S3, you need to install and configure the IDF. More information can be found in the [Espressif documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html).

## Authors

- **[Kacper Machnik](https://github.com/KacperMachnik)**
- **[Michał Krzempek](https://github.com/miskrz0421)**
- **[Tomasz Madeja](https://github.com/Tomciom)**

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---

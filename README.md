# Incubator Project

This repository contains the Arduino code for a DIY temperature and humidity-controlled incubator. It is designed to maintain a stable environment for hatching eggs.

The system uses a DHT11 sensor to monitor real-time temperature and humidity. A PID (Proportional-Integral-Derivative) controller algorithm is implemented in `PID.cpp` and `PID.h` [1, 2] to precisely manage a heating element, ensuring the temperature stays at the desired setpoint.

The `Protection.cpp` and `Protection.h` [4, 5] files define a safety system that monitors for excessive or low temperature and humidity levels. If these thresholds are breached, it can trigger alarms using a buzzer and LEDs.

The main `Arduino.ino` file [24] integrates these components, reads sensor data, updates the PID controller, and displays the current status (temperature, humidity, setpoint) on an I2C LCD screen.

## Core Features:

* **PID Temperature Control:** Actively manages a heating element to maintain a stable target temperature.
* **Environment Monitoring:** Uses a DHT11 sensor for real-time temperature and humidity readings.
* **Safety Alarms:** A `Protection` class monitors the system for unsafe conditions (e.g., overheating) and triggers alerts.
* **User Feedback:** Displays current status and setpoints on an attached I2C LCD.

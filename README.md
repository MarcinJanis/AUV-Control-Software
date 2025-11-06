🌊 AUV Control System
This repository contains the source code for an engineering thesis project: <br> `Autonomous Underwater Vehicle (AUV) control system` <br>
---
The system is based on a dual-microcontroller architecture (STM32 and ESP8266) and allows for remote control and monitoring via a Wi-Fi network.

🎯 Project Goal
The main objective of this project was to create a system for controlling the AUV's attitude (Roll, Pitch, and Yaw). The system allows an operator to send high-level commands and monitor the vehicle's telemetry in real-time through a web-based control panel.

🛠️ Architecture and Implementation

STM32 Controller: The heart of the system is an STM32F4 microcontroller running a real-time operating system (FreeRTOS) . It is responsible for:



Reading data from the IMU (MPU-6050).


Sensor fusion using a Kalman filter.


Executing PID control algorithms for each axis of rotation.


Controlling servos and the main motor (PWM).



Communicating (UART) with the ESP8266 module.




ESP8266 Wi-Fi Bridge: The ESP8266 module acts as a Wi-Fi-to-UART bridge.

It hosts a web server that provides control panel.



It translates user commands from the browser into serial commands for the STM32.

It receives telemetry data from the STM32 and serves it to the web panel.


📂 File Structure
./
   ├── ControlServerESP8266/
   │   ├── ControlPanel.json
   │   └── ControlServerESP8266.ino   # ESP8266 Web Server & Wi-Fi Bridge Code
   │
   ├── AuvControl.c                    # Implementation of control functions (PID, Kalman)
   ├── AuvControl.h                    # Header file for AuvControl.c
   └── main.c                        # Main STM32 file (init, RTOS task setup)
   
---
# Hardware:

STM32F4 development board (e.g., Nucleo)

ESP8266 module (e.g., NodeMCU, D1 Mini)

MPU-6050 IMU sensor

Digital controlled waterproof servos

Brished DC Motor

Self-made electric circut:
  - DC motor controller
  - Voltage translator for servo controll
  - Communication buses
  - Power supply (Voltage stabilisation)

# Software developed and launch with:

STM32CubeIDE

Arduino IDE (with the ESP8266 board package installed)


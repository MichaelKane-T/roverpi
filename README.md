
 RoverPi 🚗🤖
 
An embedded robotics rover platform built around a **Raspberry Pi Zero 2 W** and **ESP32**, combining real-time motor control, camera streaming, web-based remote control, and future autonomous navigation / machine learning features.
 
![Build Status](https://img.shields.io/badge/status-active-green)
![Platform](https://img.shields.io/badge/platform-ESP32%20%7C%20Pi%20Zero%202W-blue)
![License](https://img.shields.io/badge/license-MIT-gray)
 
---
 
## 📸 Project Media
 
### Rover Picture
<!-- Insert rover image below -->
![Rover Image](docs/images/rover-photo.jpg)
 
### Demo Video
<!-- Replace with your YouTube / Drive / GitHub video link -->
[▶ Watch Rover Demo](https://your-video-link-here.com)
 
---
 
## 🚀 Project Overview
 
RoverPi uses a **distributed architecture** across two processors:
 
### Raspberry Pi Zero 2 W — Mission Computer
- Live camera streaming
- Flask web dashboard
- UI control buttons
- UART communication with ESP32
- Future machine learning / autonomy logic
- Telemetry display
### ESP32 — Real-Time Controller
- FreeRTOS task scheduling
- Motor driver control
- Ultrasonic obstacle detection
- Servo sweep scanning
- Finite State Machine (FSM)
- Safety stop logic
- UART communication with Pi
---
 
## 🧠 System Architecture
 
```
             +----------------------+
             | Raspberry Pi Zero 2W |
             |----------------------|
             | Flask Dashboard      |
             | Camera Stream        |
             | ML / Navigation      |
             | UART Commands        |
             +----------+-----------+
                        |
                      UART
                        |
             +----------v-----------+
             |        ESP32         |
             |----------------------|
             | FreeRTOS Tasks       |
             | FSM Safety Logic     |
             | Motor Control        |
             | Ultrasonic Sensor    |
             | Servo Scanner        |
             +----------+-----------+
                        |
                  TB6612FNG Driver
                        |
                     Motors
```
 
---
 
## ⚙️ Current Features
 
### ✅ Working
- Live camera streaming
- UART communication (Pi ↔ ESP32)
- Web server dashboard
- PCA9685 pan/tilt camera mount
- Motor control via ESP32
- Obstacle distance reporting
- Real-time stop behavior
- Headless Raspberry Pi deployment
### 🚧 Planned
- Web joystick control
- Battery level monitor
- Improved UI dashboard
- Camera pan/tilt browser control
---
 
## 🤖 Control Modes
 
### Autonomous Mode (Default)
Rover starts in autonomous mode. Future behavior:
- Sensor-based movement
- Camera + ML decisions
- Path following
- Smart obstacle avoidance
### Manual Mode
When the user presses a button on the web dashboard:
```
AUTO → STOP → MANUAL
```
Human operator takes over motor control.
 
---
 
## 🔁 ESP32 Finite State Machine
 
```
INIT  → STOP
STOP  → DRIVE
DRIVE → STOP
ANY   → FAULT
```
 
**Safety Priority:** `FAULT > STOP > MANUAL > AUTO`
 
Obstacle detected:
```
DRIVE → STOP  (immediate)
```
 
---
 
## 🔌 Hardware
 
| Category | Component |
|----------|-----------|
| Compute | Raspberry Pi Zero 2 W |
| Compute | ESP32 Dev Board |
| Motion | 4× DC gear motors |
| Motion | TB6612FNG motor driver |
| Sensors | HC-SR04 ultrasonic sensor |
| Sensors | GY-521 MPU6050 *(planned)* |
| Vision | Raspberry Pi Camera |
| Vision | ArduCam Pan/Tilt PCA9685 mount |
| Power | LiPo battery system |
| Power | Battery monitoring *(planned)* |
 
---
 
## 🛠 Software Stack
 
### Raspberry Pi
- Python 3
- Flask
- OpenCV
- Picamera2
- pyserial
- smbus2
### ESP32
- ESP-IDF
- FreeRTOS
- UART driver
- PWM / GPIO / timers
---
 
## 📂 Project Structure
 
```
roverpi/
├── app/
│   ├── main.py
│   ├── pantilt.py
│   └── templates/
│
├── esp32/
│   ├── main.c
│   ├── motor.c
│   ├── sensor.c
│   └── fsm.c
│
├── docs/
│   ├── images/
│   └── videos/
│
└── README.md
```
 
---
 
## 🧪 How to Run
 
### Raspberry Pi
 
```bash
cd ~/roverpi
source .venv/bin/activate
cd app
python main.py
```
 
Open in browser:
```
http://<pi-ip>:5000
```
 
### ESP32
 
```bash
idf.py build
idf.py flash monitor
```
 
---
 
## 🚀 Roadmap
 
### Near Term
- [ ] Manual web joystick control
- [ ] Battery level monitor
- [ ] Improved UI dashboard
- [ ] Camera pan/tilt browser control
### Mid Term
- [ ] Autonomous roaming
- [ ] Object tracking
- [ ] IMU-based heading control
- [ ] Path memory
### Advanced
- [ ] TinyML on ESP32
- [ ] Vision-based autonomy on Pi
- [ ] SLAM / mapping
- [ ] Multi-agent swarm rovers 😎
---
 
## 🎯 Why This Project Matters
 
This project demonstrates:
- Embedded systems design
- Real-time control systems
- Distributed robotics architecture
- FreeRTOS task management
- Hardware/software integration
- Networking and web UI
- Future ML integration
---
 
## 👤 Author
 
**Michael Kane**
Electrical & Computer Engineering
*Robotics • Embedded Systems • FPGA • Autonomous Systems*
 
---
 
## ⭐ Like This Project?
 
Star the repo, fork it, or build your own rover.
 
*Built with curiosity, broken parts, and determination.*
 

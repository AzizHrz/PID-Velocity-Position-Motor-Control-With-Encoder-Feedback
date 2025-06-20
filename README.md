# PID Velocity & Position Motor Control with Encoder Feedback

This repository contains the full implementation of a DC motor control system using encoder feedback and PID algorithms. It features both **velocity** and **position** control loops, making it ideal for robotics, automation, and embedded systems projects.

## ❗ Why This Project?

Controlling a motor precisely isn't as simple as just setting a speed. Motors face load changes, friction, voltage drops, and more. Without feedback, motion becomes inaccurate and unstable.

This project solves that using:
- **Encoder feedback** to measure real motion
- **Interrupts** for high-resolution pulse counting
- **Software filters** to remove measurement noise
- **PID control** to dynamically adjust motor speed and direction

With this system, the motor maintains consistent RPM or moves to precise positions — even when pushed or resisted.

👉 **See it in action** and get a full explanation here:  
[🔗 My LinkedIn Post with Demo Video](https://www.linkedin.com/posts/azizharzallah_embeddedsystems-pidcontrol-robotics-activity-7341442268337164291-9MnU?utm_source=social_share_send&utm_medium=member_desktop_web&rcm=ACoAAEU6P-MBBEqjAbo-QGZuhCHm6gCQ93MUGGc)  

---

## 💡 Features

- Real-time PID velocity control with encoder feedback
- Position-based turning using encoder counts
- PWM control with smooth ramp-up
- Low-pass filter to stabilize noisy RPM readings
- Turn and move in rectangular paths with precision

---

## 🚀 Getting Started

You can run this project using either **Arduino IDE** or **VSCode with PlatformIO**.

### Option 1: Arduino IDE
1. Open the main `.ino` or `.cpp` file from the `src/` folder.
2. Select your board (e.g., Arduino Uno or Mega).
3. Upload the code via USB.
4. Open Serial Monitor at **115200 baud** to view debug output.

### Option 2: VSCode with PlatformIO
1. Install the PlatformIO extension in VSCode.
2. Clone this repo:
   
   git clone https://github.com/your-username/PID-Motor-Control-With-Encoder-Feedback.git
   cd PID-Motor-Control-With-Encoder-Feedback

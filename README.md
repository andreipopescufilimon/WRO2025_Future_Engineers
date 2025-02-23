# WRO2025 Future Engineers – Nerdvana Taurus Team

Repository of Nerdvana Taurus Team competing in the **World Robot Olympiad (WRO) 2025**, Future Engineers category. 

---

## 📚 Table of Contents

- [👥 The Team](#the-team)
- [🎯 Challenge Overview](#challenge-overview)
- [🤖 The Robot](#the-robot)
- [⚙️ Mobility Management](#mobility-management)
- [🛠️ Power and Sense Management](#power-and-sense-management)
- [📝 Obstacle Management](#obstacle-management)
- [📽️ Performance Videos](#performance-videos)
- [💰 Cost Analysis](#cost-analysis)
- [📂 Resources](#resources)
- [📜 License](#license)

## 📂 Folder Structure

This repository is organized as follows:

```
📦 WRO2025_Future_Engineers
├── 📁 3D-models             # Contains 3D design files for the robot's components
│   ├── 📁 old-3D-models     # Previous versions of 3D models
│   └── 📁 step-models       # 3D models in STEP format
├── 📁 electrical-schematics # Circuit diagrams and wiring
├── 📁 github-commits        # Commit logs and change tracking details for this repository
├── 📁 media                 # Images and videos 
│   ├── 📁 robot-photos      # Photos of the robot
│   ├── 📁 team-photos       # Pictures of team members and teamwork
│   └── 📁 video             # Recorded testing
├── 📁 other                 # Contains files that do not fit into other categories
├── 📁 src                   # Main source code for the robot
├── 📁 technical-draws       # Technical drawings and mechanical blueprints
├── 📄 LICENSE               # MIT License for the project
└── 📄 README.md             # Main documentation for the project
```

---

## 👥 The Team <a id="the-team"></a>

### Popescu Filimon Andrei Cosmin  
![image](media/team-photos/Andrei.png) <br>
**Age:** 16 <br>
**High School:** International Computer High School Bucharest (ICHB)  

**Description:**  
Hi! I’m Andrei from Romania, and this is my fifth WRO season. This is my first season in Future Engineers, as before I competed in Robomission category. I am passionate about robotics especially electronics and latest algorithms and tech. Over the years, I have worked on multiple robotics projects, including line followers, sumo bots, and air quality modules. Apart from robotics, I also enjoy videography, programming, and cycling.

---

### Horia Simion  
![image](media/team-photos/Horia.png) <br>
**Age:** 15 <br>
**High School:** [High School Name]  

**Description:**  
Hi! I’m Horia...

---

## 🎯 Challenge Overview <a id="challenge-overview"></a>

The **WRO 2025 Future Engineers** challenge pushes teams to develop a **fully autonomous vehicle** capable of navigating a **dynamic and randomized racetrack** using **sensors, computer vision, and advanced control algorithms**. The goal is to complete **multiple laps** while adapting to randomized obstacles, following **strict driving rules**, and successfully executing a **parallel parking maneuver** at the end of the course.

### 📌 Competition Format

- **🏁 Open Challenge**: The vehicle must complete **three (3) laps** on a track with **randomly placed inside walls**.

- **🚦 Obstacle Challenge**: The vehicle must complete **three (3) laps** while detecting and responding to **randomly placed red and green traffic signs**:
  - 🟥 **Red markers** → The vehicle must stay on the **right side of the lane**.
  - 🟩 **Green markers** → The vehicle must stay on the **left side of the lane**.
  
  After completing the three laps, the vehicle must **locate the designated parking zone** and perform a **precise parallel parking maneuver** within a limited space, adding an extra layer of difficulty.
  
- **📑 Documentation**: Each team must maintain a **public GitHub repository** showcasing their **engineering process, vehicle design, and source code**.

### 🏆 Scoring & Evaluation
Scoring is based on **accuracy, technical documentation and speed**, rewarding teams that balance **efficiency, adaptability, and innovation**. This challenge not only tests **robotics and programming skills** but also promotes **problem-solving, teamwork, and engineering creativity**.

🔗 **Find out more about the challenge [here](https://wro-association.org/wp-content/uploads/WRO-2025-Future-Engineers-Self-Driving-Cars-General-Rules.pdf).** 🚀

---

## 🤖 The Robot <a id="the-robot"></a>
| ![Top View](#)      | ![Bottom View](#)   |
|---------------------|---------------------|
| <p align="center"><b>Top</b></p> | <p align="center"><b>Bottom</b></p> |

| ![Left View](#)    | ![Right View](#)    |
|---------------------|---------------------|
| <p align="center"><b>Left</b></p> | <p align="center"><b>Right</b></p> |

| ![Front View](#)   | ![Back View](#)     |
|---------------------|---------------------|
| <p align="center"><b>Front</b></p> | <p align="center"><b>Back</b></p> |

---

## ⚙️ Mobility Management <a id="mobility-management"></a>
*To be completed – Explanation of problem-solving methods, mechanical design, and CV implementation.*

---

## 🛠️ Power and Sense Management <a id="power-and-sense-management"></a>
### **Main Components Used**
Below is a list of core hardware components used in our robot:

| Component           | Part & Link                                      | Function                          |
|---------------------|------------------------------------------------|----------------------------------|
| **Main Controller** | [Arduino NANO ESP32]([https://www.jsumo.com/xmotion-robot-controller](https://store.arduino.cc/en-ro/products/nano-esp32?srsltid=AfmBOoqi1bKUrg9Tf_EzO6Rsk0S4Sve9OK52Us1N13vKBl_Mdtdb_NRZ)) | Controls all components         |
| **Drive Motor**     | [30:1 Micro Metal Gearmotor HPCB 12V w/Encoder](https://www.pololu.com/product/3038) | Drives the robot |
| **Steering Servo**  | [MG90S](https://towerpro.com.tw/product/mg90s-3/) | Controls steering               |
| **Camera**         | [OpenMV H7 Camera](https://openmv.io/products/openmv-cam-h7) | Detects traffic signs           |
| **Gyroscope**      | [MPU-6050](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) | Tracks orientation              |
| **Battery**        | [3S Li-Po Battery (11.1V, 450mAh)](https://gensace.de/products/gens-ace-g-tech-soaring-450mah-11-1v-30c-3s1p-lipo-battery-pack-with-jst-syp-plug) | Powers the robot |
| **Wheels**         | Lego Spike Wheels           | Provides grip                   |
| **Gearbox**        | 3D Printed and Lego gears   | Transfers power                 |

---

## 📝 Obstacle Management <a id="obstacle-management"></a>
*To be completed – Overview of algorithms, sensor fusion, motor control, and logic.*

---

## 📽️ Performance Videos <a id="performance-videos"></a>
*To be completed – Links to YouTube videos demonstrating different test scenarios.*

---

## 💰 Cost Analysis <a id="cost-analysis"></a>
We have optimized our robot for **performance vs. cost efficiency**. The total cost includes motors, sensors, electronics, and custom parts.
### 🏗️ **Component Costs**
| Component              | Quantity | Unit Price ($) | Total ($) |
|------------------------|----------|--------------|-----------|
| **Main Controller**    | 1        | 21.42        | 21.42     |
| **Drive Motor**        | 2        | 22.45        | 22.45     |
| **Steering Servo**     | 1        | 4.05         | 4.05      |
| **Camera**            | 1        | 80.00        | 80.00     |
| **Gyroscope**         | 1        | 3.25         | 3.25      |
| **LiPo Battery**      | 1        | 8.99         | 8.99      |
| **Wheels**            | 4        | 1.64         | 6.56      |
| **Gearbox**           | 1        | -            | -         |
| **Other Materials**   | -        | -            | -         |
| **TOTAL COST**        | -        | -            | **146.72** |

* Prices are approximate, based on current market prices.

*To be completed*

---

## 📂 Resources <a id="resources"></a>
*To be completed*

---

## 📜 License <a id="license"></a>
```
MIT License

Copyright (c) 2025 Popescu Filimon Andrei Cosmin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

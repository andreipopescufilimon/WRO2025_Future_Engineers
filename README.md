# WRO2025 Future Engineers – Nerdvana Taurus Team

Repository of Nerdvana Taurus Team competing in the **World Robot Olympiad (WRO) 2025**, Future Engineers category. 

---

## 📚 Table of Contents

- [👥 The Team](#the-team)
- [🎯 Challenge Overview](#challenge-overview)
- [🤖 Our Robot](#our-robot)
- [⚙️ Mobility Management](#mobility-management)
  - [🚗 Drivebase](#drivebase)
    - [🔧 Drivetrain](#drivetrain)
    - [⚙️ Motor](#motor)
    - [🔌 Motor Driver TB6612FNG](#motor-driver)
  - [🔄 Steering](#steering)
    - [🔄 Steering Servo Motor](#servo-motor)
  - [🏎️ Chassis & Component Mounting](#chassis)
    - [✨ Key Features & Advantages](#key-features)
    - [🔧 Assembly Process](#assembly-process)
- [🛠️ Power and Sense Management](#power-and-sense-management)
  - [🔋 Li-Po Battery](#li-po-battery)
  - [🖥️ Arduino Nano ESP32](#arduino-nano-esp32)
  - [🧭 IMU Sensor MPU-6050](#imu-sensor-mpu-6050)
  - [📷 OpenMV H7 Camera](#openmv-h7-camera)
  - [⚙️ Drive Motor](#drive-motor)
  - [⚙️ Motor Driver TB6612FNG](#motor-driver)
  - [🔄 Steering Servo MG90S](#steering-servo)
  - [🔌 Voltage Regulator L7805CV](#voltage-regulator)
  - [🛠️ PCB Design](#pcb-design)
  - [⚡ Power Consumption](#power-consumption)
- [📝 Obstacle Management](#obstacle-management)
  - [🏁 Open Round](#open-round) 
  - [⚡ Final Round](#final-round)
  - [🅿️ Parking](#parking)
- [📽️ Performance Videos](#performance-videos)
- [💰 Cost Analysis](#cost-analysis)
  - [📦 Component Costs](#components-costs)
  - [🔌 PCB Manufacturing Cost](#pcb-cost)
  - [🖨️ 3D Printing Cost](#3d-printing-cost)
  - [🔩 Other Materials (Screws, Nuts, Miscellaneous)](#other-materials-cost)
  - [💵 Total Cost](#total-cost)
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

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/media/team-photos/team.png" width="450">

**This image was taken at the **WRO 2023 International Final** in Panama, where we placed **14th in the Robomission Junior Category**.*

### **Popescu Filimon Andrei Cosmin** (Left Side)
**Age:** 16 <br>

**High School:** International Computer High School Bucharest (ICHB)  

**Description:**  
Hi! I’m Andrei from Romania, and this is my fifth WRO season. This is my first season in Future Engineers, as before I competed in Robomission category. I am passionate about robotics especially electronics and latest algorithms and tech. Over the years, I have worked on multiple robotics projects, including line followers, sumo bots, and air quality modules. Apart from robotics, I also enjoy videography, programming, and cycling.

---

### **Horia Simion** (Right Side)
**Age:** 16 <br>
**High School:** National College "Mihai Viteazul" (CNMV)

**Description:**  
Hi! I’m Horia from Romania, and this is my second WRO season competing alongside Andrei. I have participated in RoboMission multiple times, gaining valuable experience in solving various problems that may arise while building a robot. I have a strong interest in technology and robotics and am always eager to learn and experiment with new ideas.

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

## 🤖 Our Robot <a id="our-robot"></a>
| <img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/tree/main/media/robot-photos/top.png" width="300">         | <img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/tree/main/media/robot-photos/bottom.png" width="300">            |
|----------------------------------|-------------------------------------|
| <p align="center"><b>Top</b></p> | <p align="center"><b>Bottom</b></p> |

| <img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/tree/main/media/robot-photos/left.png" width="300">          | <img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/tree/main/media/robot-photos/right.png" width="300">           |
|-----------------------------------|------------------------------------|
| <p align="center"><b>Left</b></p> | <p align="center"><b>Right</b></p> |

| <img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/tree/main/media/robot-photos/front.png" width="300">           | <img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/tree/main/media/robot-photos/back.png" width="300">          |
|------------------------------------|-----------------------------------|
| <p align="center"><b>Front</b></p> | <p align="center"><b>Back</b></p> |

---

## 🚗 Mobility Management <a id="mobility-management"></a>

The robot's mobility is controlled through **a fully 3D-printed chassis, a servo-based steering system, and a drivetrain featuring a Lego differential and axles**. These components work together to ensure smooth, **precise movement with optimized traction, stability, and efficient power management**.


## ⚙️ **Drivebase** <a id="drivebase"></a>

### 🔧 **Drivetrain** <a id="drivetrain"></a>

To optimize performance and minimize **energy loss due to friction**, the drivetrain uses a **Lego 5-gear differential combined with Lego axles to connect the wheels**. The **motor is attached to the drivetrain using a custom 3D-printed bracket**, ensuring **a secure and efficient power transfer**.

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/technical-draws/motor-bracket-technical-drawing.jpg" width="700">

A **custom 3D-printed gear with a D-shaped axle mount** is directly connected to the motor shaft. This **gear provides a 1:1 gear ratio between the motor and the differential**, ensuring that the **motor’s full speed and torque are transferred efficiently to the drivetrain** without unnecessary losses. This setup allows for **smooth acceleration, precise speed control, and optimal wheel synchronization**, improving overall handling and maneuverability.

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/technical-draws/gear-D-axle-technical-drawing.jpg" width="700">

By using **Lego components**, we gained the advantages of **precision-engineered mechanical parts** while maintaining a **lightweight and modular system** that allows easy modifications. The **rear 5 gears differential enables smoother turns** by balancing wheel speeds dynamically.

### ⚙️ **Motor – 30:1 Micro Metal Pololu Gearmotor HPCB** <a id="motor"></a>
Following past testing, we selected **a high-power 30:1 Micro Metal Gearmotor (12V)** for the drive system. This motor provides an **optimal balance of speed and torque**, allowing the robot to maintain stability while navigating turns.

| <img src="https://a.pololu-files.com/picture/0J6410.600x480.jpg?7b215efad0f55f0963d9f951be03d3d1" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** 30:1 HPCB | **Voltage:** 12V |
| **No-load Speed:** 1000 RPM | **No-load Current:** 120mA |
| **Stall Torque:** ~0.4 kg·cm | **Stall Current:** 1.6A |
| 🔗 **[Buy Here](https://www.pololu.com/product/3038)** | **Function:** Drives the robot |

➜ **Why We Chose This Motor?**  
✔ **Gear ratio provides sufficient torque** without sacrificing efficiency.  
✔ **Compact design**, allowing integration into a lightweight robot.  

### 🔌 **Motor Driver – Sparkfun TB6612FNG** <a id="motor-driver"></a>
To control the motor's speed and direction, we integrated a **Sparkfun Dual TB6612FNG motor driver** into the PCB. This dual motor driver enables **precise adjustments for acceleration, braking, and turning** through **PWM control**.

| <img src="https://www.sparkfun.com/media/catalog/product/cache/a793f13fd3d678cea13d28206895ba0c/1/4/14451-01.jpg" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** TB6612FNG | **Operating Voltage:** 2.5V – 13.5V |
| **PWM Frequency:** Up to 100 kHz | **Max Continuous Current:** 1.2A |
| 🔗 **[Buy Here](https://www.sparkfun.com/sparkfun-motor-driver-dual-tb6612fng-1a.html)** | **Function:** Controls the drive motor |

## 🔄 **Steering** <a id="steering"></a>

The **steering system** is a critical part of the robot, ensuring precise maneuverability and smooth turns. Our design is based on a **parallelogram steering mechanism**, where both front wheels turn at the same angle through a single servo-controlled linkage. This setup provides predictable and stable steering, making it easy to use for an autonomous vehicle. Instead of using an **Ackermann steering system**, which requires more complex calculations and linkages, we opted for a **simpler and more lightweight solution** that offers consistent control. Our steering system allows for a maximum turning angle of 50 degrees in both the left and right directions. This range provides precise maneuverability, enabling the robot to navigate sharp turns efficiently while maintaining stability.

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/other/steering-3D-model.png" width="700">

Our **steering arm is directly connected to the servo**, which moves the two front wheels simultaneously. This ensures that the turning response is immediate and proportional to the servo's motion. The **wheels are mounted on special mounts**, allowing for smooth and precise movement without excessive friction. To ensure **structural integrity and long-term reliability**, the steering system is **assembled using M3 screws and lock nuts**. These prevent the nuts from loosening due to vibrations, ensuring a **secure and stable connection**.

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/technical-draws/right-front-wheel-mount-technical-drawing.jpg" width="700">

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/technical-draws/steering-connector-bar-technical-drawing.jpg" width="700">

**⚙️ Design Considerations & Improvements** <br>
While the parallelogram steering system is effective, some potential enhancements could improve its performance. Additionally, optimizing the **steering linkages** could reduce mechanical play, making the system more precise. In future iterations, we may experiment with **Ackermann geometry** to better distribute wheel angles during turns, further improving efficiency and reducing tire slippage.

---

### **🔄 Steering Servo – MG90S** <a id="servo-motor"></a>

To control the steering system, we use an **MG90S micro servo**, known for its high torque and fast response. This servo enables quick and precise adjustments while maintaining a compact and lightweight design. Featuring **metal gears**, it ensures durability and reliability over extended use. The servo is securely mounted onto the chassis with two screws, and the steering arm is directly attached to its output shaft, providing smooth and efficient motion transfer for accurate steering.

| <img src="https://static.optimusdigital.ro/20565-large_default/mg90s-servomotor.jpg" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** MG90S | **Voltage:** 5V |
| **Torque:** 2.2kg/cm | **Signal Type:** PWM |
| **Current Draw (Avg):** 120mA | **Peak Current:** 500mA |
| **Weight:** ~13.4g | **Gears:** Plastic |
| 🔗 **[Buy Here](https://www.optimusdigital.ro/ro/motoare-servomotoare/271-servomotor-mg90s.html?srsltid=AfmBOooTrDsx2UoJ3Px8J26kkCbcuYhlpKYmuIYkivK_5ZSzPJx0ZNo8)** | **Function:** Controls steering |

---

## 🏎️ **Chassis & Component Mounting** <a id="chassis"></a>

The chassis is the **main structure** of the robot, providing a **stable and lightweight base** for all components. Since this is our **first year competing**, we decided to **skip Lego-based designs** and go straight for a **fully 3D-printed chassis**. This allowed us to create a **more compact, lightweight, and optimized structure**.

The robot is driven by **a single high-performance motor**, which is connected to a **Lego 5 gears differential**. This setup allows reducing complexity while maintaining precise control over movement.

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/technical-draws/chassis-technical-drawing.jpg" width="700">

## ✨ **Key Features & Advantages** <a id="key-features"></a>

✔️ **Lightweight & Durable** – The **3D-printed chassis** ensures a strong yet lightweight structure, optimizing performance.  
✔️ **Hexagonal Cutouts** – Reduce weight while maintaining strength **+ they enhance the robot's aesthetics**.  
✔️ **Balanced Design** – The **battery is centrally placed**, ensuring even weight distribution and stability.  
✔️ **Easy Component Mounting** – Pre-designed slots for **motor, PCB, steering servo, and camera** make assembly quick and efficient.  
✔️ **Secure PCB Mounting** – The **PCB is attached using 4 screws**, keeping it firmly in place during operation.  
✔️ **Organized Wiring & Cable Management**  
&nbsp; &nbsp; 🔹 **Integrated PCB routing** eliminates unnecessary wiring, ensuring a cleaner and more reliable setup.  
&nbsp; &nbsp; 🔹 **Hot glue secures servo and camera wires**, preventing loose cables from interfering with movement.  

---

**🔧 Assembly Process** <a id="assembly-process"></a>

1️⃣ **Attach the drive motor** using its dedicated mounting bracket.  
2️⃣ **Secure the steering servo** in its designated slot and secure it using to screws.  
3️⃣ **Install the Lego differential** and connect it to the drivetrain.  
4️⃣ **Place the battery in the center compartment** to keep weight evenly distributed.  
5️⃣ **Mount the PCB using 4 screws**, ensuring a solid connection to minimize vibrations that can interfere with the IMU.  
6️⃣ **Use hot glue for cable management**, keeping servo and camera wires in place.  


---

## 🛠️ Power and Sense Management <a id="power-and-sense-management"></a>

The robot's **power and sensing system** is designed for **efficiency, precision, and adaptability**. It consists of an **Arduino Nano ESP32, a Li-Po battery, MPU-6050 IMU, OpenMV H7 Camera, motors, and a voltage regulator**, all working together to ensure **stable operation and accurate navigation**.  

Each component has been carefully selected to provide **optimal performance**, minimize power consumption, and ensure **reliability in competition environments**.

---

### **🔋 Li-Po Battery – 3S 450mAh** <a id="li-po-battery"></a>

The **Li-Po battery** provides a **compact, lightweight, and high-discharge** power source, making it ideal for our robot. The **11.1V output** is suitable for running our **motors and voltage regulator**, ensuring stable power delivery.

| <img src="https://gensace.de/cdn/shop/files/1_113_10.jpg?v=1722466075&width=900" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** 3S Li-Po | **Capacity:** 450mAh |
| **Voltage:** 11.1V | **Discharge Rate:** 30C |
| **Weight:** 38g | **Size:** 56.5 × 31 × 9mm |
| **Output Current:** Varies by load | **Connector Type:** JST |
| 🔗 **[Buy Here](https://gensace.de/products/gens-ace-g-tech-soaring-450mah-11-1v-30c-3s1p-lipo-battery-pack-with-jst-syp-plug)** | **Function:** Powers the entire robot |

---

### **🖥️ Arduino Nano ESP32 – Main Controller** <a id="arduino-nano-esp32">

The **Arduino Nano ESP32** provides **high-speed processing, built-in Wi-Fi and Bluetooth, and extensive GPIO capabilities** in a **compact form factor**. This allows it to handle **sensor data, motor control, and vision processing efficiently**.

| <img src="https://store.arduino.cc/cdn/shop/files/ABX00092_01.iso_804x603.jpg?v=1727101612" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Microcontroller:** ESP32 | **Flash Memory:** 4MB |
| **SRAM:** 520KB | **Frequency:** 240MHz |
| **Pins:** 22 | **Input Voltage:** 5V |
| **Current Draw (Avg):** 200mA | **Peak Current:** 500mA |
| 🔗 **[Buy Here](https://store.arduino.cc/en-ro/products/nano-esp32)** | **Function:** Controls all robot components |

---

### **🧭 IMU Sensor – MPU-6050** <a id="imu-sensor-mpu-6050">

The **MPU-6050 IMU** is used to **measure the robot's angular velocity and acceleration**, helping it maintain **stability and precise movement control**. It is essential for **calculating turns and avoiding drift**.

| <img src="https://static.optimusdigital.ro/7330-large_default/mpu6050-accelerometer-and-gyroscope-module.jpg" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Gyroscope Range:** ±2000°/s | **Accelerometer Range:** ±16g |
| **Interface:** I2C | **Supply Voltage:** 3.3V – 5V |
| **Current Draw:** 3.5mA | **Weight:** 5g |
| 🔗 **[Buy Here](https://www.optimusdigital.ro/en/inertial-sensors/96-mpu6050-accelerometer-and-gyroscope-module.html?srsltid=AfmBOoqUolw8nE5zvAGPay7WXfhQYGzOXsF0TSSw_YOopMMU_cKW5CZV)** | **Function:** Tracks orientation & motion |

---

### **📷 OpenMV H7 Camera – Vision Processing** <a id="openmv-h7-camera">

The **OpenMV H7 Camera** processes **traffic signs, lane detection, and other visual cues** in real-time. Unlike traditional cameras, it has a **built-in microcontroller**, reducing the processing load on the Arduino and allowing it to **focus on movement control**.

| <img src="https://openmv.io/cdn/shop/products/new-cam-v4-angle-hero-web_1000x.jpg?v=1715735352" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Microcontroller:** STM32H7 | **Flash Memory:** 32MB |
| **RAM:** 512KB | **Frequency:** 480MHz |
| **Resolution:** 640x480 | **Frame Rate:** 60fps |
| **Current Draw (Avg):** 300mA | **Peak Current:** 400mA |
| 🔗 **[Buy Here](https://openmv.io/products/openmv-cam-h7)** | **Function:** Detects traffic signs & lanes |

---

### **⚙️ Drive Motor – 30:1 Micro Metal Gearmotor HPCB 12V w/ Encoder** <a id="drive-motor">

The **drive motor is responsible for propelling the robot forward**. The **30:1 gearbox** provides an excellent **balance of speed and torque**, while the **built-in encoder** allows for **precise speed control**.

| <img src="https://a.pololu-files.com/picture/0J6410.600x480.jpg?7b215efad0f55f0963d9f951be03d3d1" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Type:** Micro Metal Gearmotor | **Gear Ratio:** 30:1 |
| **Voltage:** 12V | **Encoder:** Yes |
| **Current Draw (Avg):** 120mA | **Peak Current:** 1.6A |
| **Weight:** ~10g | **Shaft Diameter:** 3mm |
| 🔗 **[Buy Here](https://www.pololu.com/product/3038)** | **Function:** Drives the robot |

---

### **⚙️ Dual Motor Driver – TB6612FNG** <a id="motor-driver"></a>

The **TB6612FNG** motor driver is used to **control the robot’s two drive motors** efficiently. It supports **precise speed and direction control** using **PWM signals**, making it ideal for **differential drive robots**. The **TB6612FNG is directly integrated into our PCB**, enabling smooth communication with the **Arduino Nano ESP32**.

| <img src="https://www.sparkfun.com/media/catalog/product/cache/a793f13fd3d678cea13d28206895ba0c/1/4/14451-01.jpg" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** TB6612FNG | **Operating Voltage:** 2.5V – 13.5V |
| **Logic Voltage:** 2.7V – 5.5V | **PWM Frequency:** Up to 100 kHz |
| **Max Continuous Current:** 1.2A per channel | **Max Peak Current:** 3.2A per channel |
| **Number of Channels:** 2 (Dual Motor Control) | **Built-in Protections:** Thermal & Overcurrent |
| 🔗 **[Buy Here](https://www.sparkfun.com/sparkfun-motor-driver-dual-tb6612fng-1a.html)** | **Function:** Controls drive motors |

---

### **🔄 Steering Servo – MG90S** <a id="steering-servo">

The **MG90S servo is used for precise steering control**, enabling the robot to **navigate turns with accuracy**. It provides **high torque output in a compact size**.

| <img src="https://static.optimusdigital.ro/20565-large_default/mg90s-servomotor.jpg" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** MG90S | **Voltage:** 5V |
| **Torque:** 2.2kg/cm | **Signal Type:** PWM |
| **Current Draw (Avg):** 120mA | **Peak Current:** 500mA |
| **Weight:** ~13.4g | **Gears:** Plastic |
| 🔗 **[Buy Here](https://www.optimusdigital.ro/ro/motoare-servomotoare/271-servomotor-mg90s.html?srsltid=AfmBOooTrDsx2UoJ3Px8J26kkCbcuYhlpKYmuIYkivK_5ZSzPJx0ZNo8)** | **Function:** Controls steering |

---

### **🔌 L7805CV Voltage Regulator – Power Management** <a id="voltage-regulator"> 

The **L7805CV** regulates the **11.1V Li-Po battery output** to a **stable 5V**, ensuring **safe power delivery** to the **Arduino, sensors, and camera**. It prevents **overvoltage damage** and includes **thermal & short-circuit protection** for reliability.

| <img src="https://ce8dc832c.cloudimg.io/v7/_cdn_/5D/D0/90/00/0/593365_1.jpg?width=640&height=480&wat=1&wat_url=_tme-wrk_%2Ftme_new.png&wat_scale=100p&ci_sign=32c0b49b36a510891beaad3401e2b2b50bdee888" width="300">  | **Specifications** |
|------------------------------|------------------------------|
| **Model:** L7805CV | **Input Voltage:** 7V – 35V |
| **Output Voltage:** 5V | **Output Current:** 1.5A |
| **Efficiency:** Linear Regulator | **Dropout Voltage:** ~2V |
| **Protection:** Short-circuit & thermal shutdown | **Mounting Type:** TO-220 Package |
| 🔗 **[Buy Here](https://www.tme.eu/ro/details/l7805cv/regulatoare-de-tensiune-neregulata/stmicroelectronics/)** | **Function:** Converts battery voltage to 5V |

---

### **🛠️ PCB Design** <a id="pcb-design"></a>
| **Electrical Schematics** | **PCB Design** |
|---------------------------|---------------------------|
| <img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/electrical-schematics/Schematic_MainBoard.png" width="400"> | <img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/electrical-schematics/MainBoardPCB_img4.png" width="400"> |

**🔎 Advantages of a Custom PCB**  

✔ **Organized layout** → Prevents loose connections & messy wiring  
✔ **Power stability** → Ensures consistent voltage supply to all components  
✔ **Compact design** → Reduces weight & optimizes space  
✔ **Reliability** → Minimizes risk of failure due to poor wiring  

---

### **⚡ Power Consumption Summary** <a id="power-consumption">

| **Component**                 | **Voltage** | **Avg Current Draw** | **Peak Current** |
|-------------------------------|------------|-----------------------|------------------|
| **Arduino Nano ESP32**        | 5V         | 200mA                 | 500mA            |
| **Drive Motor (x2)**          | 12V        | 240mA (120mA each)    | 3.2A (1.6A each) |
| **Steering Servo MG90S**      | 5V         | 120mA                 | 500mA            |
| **OpenMV H7 Camera**          | 3.3V/5V    | 300mA                 | 400mA            |
| **IMU Sensor MPU-6050**       | 3.3V/5V    | 3.5mA                 | 5mA              |
| **TB6612FNG Motor Driver**    | 5V         | 50mA                  | 100mA            |
| **Voltage Regulator L7805CV** | 7.4V -> 5V | Power Management      | -                |
| **Total Robot Power Usage**   | Mixed      | ~1.3A (Avg)           | ~3.8A (Peak)     |

---

## 📝 Obstacle Management <a id="obstacle-management"></a>

### 🏁 Open Round <a id="open-round"></a>
During the **Open Round**, our robot follows a **straight trajectory using a PID controller based on gyro yaw**, ensuring stable movement. To determine turns, the **camera detects Orange and Blue lines** on the track:
- **Orange Line → Right Turn**
- **Blue Line → Left Turn**
- The turn is executed when the robot reaches an approximativ **distance from the front black wall**.

#### **Camera Processing for Line Detection**
The camera captures frames in **RGB565** format and processes color blobs using **LAB thresholds** to detect relevant track elements. Below is a **snippet of the camera’s core logic** that identifies **track lines and determines turning direction**:

```python
# Detect Blobs
red_blobs = img.find_blobs(red_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
green_blobs = img.find_blobs(green_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)
blue_blobs = img.find_blobs(blue_threshold, roi=lines_roi, pixels_threshold=min_line_size, area_threshold=min_line_size, merge=True)
orange_blobs = img.find_blobs(orange_threshold, roi=lines_roi, pixels_threshold=min_line_size, area_threshold=min_line_size, merge=True)
black_blobs = img.find_blobs(black_threshold, roi=cubes_roi, pixels_threshold=min_cube_size, area_threshold=min_cube_size, merge=True)

...

# Determine Direction
if direction == 0:
    if orange_line and not is_invalid_orange(orange_line, red_blobs):
        direction = 2  # Orange line first → turn right
    elif blue_line:
        direction = 1  # Blue line first → turn left

# Send Direction Command
uart.write(str(direction) + '\n')
```

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/other/line-detection.png" width="900">


### ⚡ Final Round <a id="final-round"></a>
*To be completed – Overview of how the robot handles the final round challenges, including obstacle adaptation and speed adjustments.*

### 🅿️ Parking <a id="parking"></a>
*To be completed – Explanation of how the robot identifies and executes the parallel parking maneuver at the end of the course.*

---

## 📽️ Performance Videos <a id="performance-videos"></a>
🔗 **[Click here to watch the video on YouTube]()**  

---

## 💰 Cost Analysis <a id="cost-analysis"></a>
We have optimized our robot for **performance vs. cost efficiency**. The total cost includes motors, sensors, electronics, PCB, 3D printing and custom parts.

### 📦 **Components Costs** <a id="components-costs"></a>
| Component                     | Quantity | Unit Price ($) | Total ($)  |
|-------------------------------|----------|---------------|-------------|
| **Arduino Nano ESP32**        | 1        | **21.42**     | **21.42**   |
| **Drive Motor (30:1 HPCB)**   | 1        | **22.45**     | **22.45**   |
| **Steering Servo (MG90S)**    | 1        | **4.05**      | **4.05**    |
| **OpenMV H7 Camera**          | 1        | **80.00**     | **80.00**   |
| **Gyroscope (MPU-6050)**      | 1        | **3.25**      | **3.25**    |
| **LiPo Battery (3S 450mAh)**  | 1        | **8.99**      | **8.99**    |
| **L7805CV Voltage Regulator** | 1        | **1.50**      | **1.50**    |
| **Lego Spike Wheels**         | 4        | **1.64**      | **6.56**    |
| **Lego Differential**         | 1        | **4.00**      | **4.00**    |
| **TOTAL COMPONENT COST**      | -        | **-**         | **152.22**  |

---

### **🔌 PCB Cost (Manufactured via JLCPCB)** <a id="pcb-cost"></a>
| PCB Component                  | Quantity | Unit Price ($) | Total ($)  |
|--------------------------------|----------|--------------|--------------|
| **PCB Manufacturing (JLCPCB)** | 5        | **0.40**     | **2.00**     |
| **Connectors & Soldering**     | -        | **3.50**     | **3.50**     |
| **TOTAL PCB COST**             | -        | **-**        | **5.50**     |

---

### **🖨️ 3D Printing Cost Breakdown** <a id="3d-printing-cost"></a>
| 3D Printed Parts                          | Quantity | Unit Price ($) | Total ($)   |
|-------------------------------------------|----------|--------------|---------------|
| **1000g filament (PLA from Aliexpress)**  | 1        | **10.00**    | **10.00**     |
| **TOTAL 3D PRINTING COST**                | -        | **-**        | **10.00**     |

---

### **🔩 Other Materials (Screws, Nuts, and Miscellaneous)** <a id="other-materials-cost"></a>
| Material                       | Quantity | Unit Price ($) | Total ($)   |
|--------------------------------|----------|--------------|---------------|
| **M3 Screws & Nuts Set**       | -        | **4.00**     | **4.00**      |
| **Wiring & Connectors**        | -        | **5.00**     | **5.00**      |
| **TOTAL OTHER MATERIALS COST** | -        | **-**        | **9.00**      |

---

### **💵 TOTAL** <a id="total-cost"></a>
| Category                      | Total Cost ($) |
|-------------------------------|----------------|
| **Components**                | **152.22**     |
| **PCB (JLCPCB + Components)** | **5.50**       |
| **3D Printing**               | **10.00**      |
| **Other Materials**           | **9.00**       |
| **TOTAL PROJECT COST**        | **176.72**     |

**Prices are approximate, based on current market prices.*

---

## 📂 Resources <a id="resources"></a>

Below is a list of **external images** used in this repository.
- **[Arduino Nano ESP32](https://store.arduino.cc/cdn/shop/files/ABX00092_01.iso_804x603.jpg?v=1727101612)**
- **[MPU-6050 IMU Sensor](https://static.optimusdigital.ro/7330-large_default/mpu6050-accelerometer-and-gyroscope-module.jpg)**
- **[OpenMV H7 Camera](https://openmv.io/cdn/shop/products/new-cam-v4-angle-hero-web_1000x.jpg?v=1715735352)**
- **[Pololu 30:1 Gearmotor](https://a.pololu-files.com/picture/0J6410.600x480.jpg?7b215efad0f55f0963d9f951be03d3d1)**
- **[TB6612FNG Motor Driver](https://www.sparkfun.com/media/catalog/product/cache/a793f13fd3d678cea13d28206895ba0c/1/4/14451-01.jpg)**
- **[MG90S Steering Servo](https://static.optimusdigital.ro/20565-large_default/mg90s-servomotor.jpg)**
- **[L7805CV Voltage Regulator](https://ce8dc832c.cloudimg.io/v7/_cdn_/5D/D0/90/00/0/593365_1.jpg?width=640&height=480&wat=1&wat_url=_tme-wrk_%2Ftme_new.png&wat_scale=100p&ci_sign=32c0b49b36a510891beaad3401e2b2b50bdee888)**
- **[3S 450mAh Li-Po Battery](https://gensace.de/cdn/shop/files/1_113_10.jpg?v=1722466075&width=900)**
  
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

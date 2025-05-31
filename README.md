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
      - [🔧 Assembly Process Video](#assembly-process-video)
      - [1: 3D Print the Parts](#3d-print-the-parts)
      - [2: Assemble the Steering System](#2-assemble-the-steering-system)
      - [3: Assemble the drive base](#3-assemble-the-drive-base)
      - [4: Install the Electronics](#4-install-the-electronics)
      - [5: Attach the Wheels](#5-attach-the-wheels)
      - [6: Add weight](#6-add-weight)
      - [7: Upload the Code](#7-upload-the-code)
- [🛠️ Power and Sense Management](#power-and-sense-management)
  - [🔋 Li-Po Battery](#li-po-battery)
  - [🖥️ Arduino Nano ESP32](#arduino-nano-esp32)
  - [🧭 IMU Sensor BMI088](#imu-sensor-bmi088)
  - [📷 OpenMV H7 Camera](#openmv-h7-camera)
  - [⚙️ Drive Motor](#drive-motor)
  - [⚙️ Motor Driver TB6612FNG](#motor-driver)
  - [🔄 Steering Servo MG90S](#steering-servo)
  - [📏 Distance Sensor – JS40F](#distance-sensor-js40f)
  - [🔌 Voltage Regulator L7805CV](#voltage-regulator)
  - [🛠️ PCB Design](#pcb-design)
  - [⚡ Power Consumption](#power-consumption)
- [📝 Obstacle Management](#obstacle-management)
  - [🏁 Open Round](#open-round) 
  - [⚡ Final Round](#final-round)
  - [🅿️ Start from Parking](#start-from-parking)
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
**Age:** 17 <br>

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
| <img src="https://raw.githubusercontent.com/andreipopescufilimon/WRO2025_Future_Engineers/main/media/robot-photos/top.png" width="300">         | <img src="https://raw.githubusercontent.com/andreipopescufilimon/WRO2025_Future_Engineers/main/media/robot-photos/bottom.png" width="300">            |
|----------------------------------|-------------------------------------|
| <p align="center"><b>Top</b></p> | <p align="center"><b>Bottom</b></p> |

| <img src="https://raw.githubusercontent.com/andreipopescufilimon/WRO2025_Future_Engineers/main/media/robot-photos/left.png" width="300">          | <img src="https://raw.githubusercontent.com/andreipopescufilimon/WRO2025_Future_Engineers/main/media/robot-photos/right.png" width="300">           |
|-----------------------------------|------------------------------------|
| <p align="center"><b>Left</b></p> | <p align="center"><b>Right</b></p> |

| <img src="https://raw.githubusercontent.com/andreipopescufilimon/WRO2025_Future_Engineers/main/media/robot-photos/front.png" width="300">           | <img src="https://raw.githubusercontent.com/andreipopescufilimon/WRO2025_Future_Engineers/refs/heads/main/media/robot-photos/back.png" width="300">          |
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

| <img src="https://a.pololu-files.com/picture/0J12418.220.jpg?8f026fe1675b1109ea574290d3d26081" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** 30:1 HPCB | **Voltage:** 12V |
| **No-load Speed:** 1000 RPM | **No-load Current:** 120mA |
| **Stall Torque:** ~0.4 kg·cm | **Stall Current:** 1.6A |
| 🔗 **[Buy Here](https://www.pololu.com/product/3038)** | **Function:** Drives the robot |

**Why We Chose This Motor?**  
**- Gear ratio provides sufficient torque** without sacrificing efficiency.  
**- Compact and lightweight design**, allowing integration into a lightweight robot.  

### 🔌 **Motor Driver – Sparkfun TB6612FNG** <a id="motor-driver"></a>
To control the motor's speed and direction, we integrated a **Sparkfun Dual TB6612FNG motor driver** into the PCB. This dual motor driver enables **precise adjustments for acceleration, braking, and turning** through **PWM control**.

| <img src="https://www.sparkfun.com/media/catalog/product/cache/a793f13fd3d678cea13d28206895ba0c/1/4/14451-01.jpg" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** TB6612FNG | **Operating Voltage:** 2.5V – 13.5V |
| **PWM Frequency:** Up to 100 kHz | **Max Continuous Current:** 1.2A |
| 🔗 **[Buy Here](https://www.sparkfun.com/sparkfun-motor-driver-dual-tb6612fng-1a.html)** | **Function:** Controls the drive motor |

## 🔄 **Steering** <a id="steering"></a>

The **steering system** is a critical part of the robot, ensuring precise maneuverability and smooth turns. Our design is based on a **parallelogram steering mechanism**, where both front wheels turn at the same angle through a single servo-controlled linkage. This setup provides predictable and stable steering, making it easy to use for an autonomous vehicle. Instead of using an **Ackermann steering system**, which requires more complex calculations and linkages, we opted for a **simpler and more lightweight solution** that offers consistent control. Our steering system allows for a maximum turning angle of 50 degrees in both the left and right directions. This range provides precise maneuverability, enabling the robot to navigate sharp turns efficiently while maintaining stability.

![Demo GIF](https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/other/steering.gif)

Our **steering arm is directly connected to the servo**, which moves the two front wheels simultaneously. This ensures that the turning response is immediate and proportional to the servo's motion. The **wheels are mounted on special mounts**, allowing for smooth and precise movement without excessive friction. To ensure **structural integrity and long-term reliability**, the steering system is **assembled using 2mm carbon fiber bars that are glued at the ends to the chassis**.

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/technical-draws/right-front-wheel-mount-technical-drawing.jpg" width="700">

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/technical-draws/steering-connector-bar-technical-drawing.jpg" width="700">

**⚙️ Design Considerations & Improvements** <br>
While the parallelogram steering system is effective, some potential enhancements could improve its performance. Additionally, optimizing the **motor mounts** could reduce mechanical play, making the system more precise. In future iterations, we may experiment with **Ackermann geometry** and suspension to better distribute wheel angles during turns, further improving efficiency and reducing tire slippage.

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

**Key Features & Advantages** <a id="key-features"></a>

➜ **Lightweight & Durable** – The **3D-printed chassis** ensures a strong lightweight structure, optimizing performance.  
➜ **Balanced Design** – The **battery is centrally placed**, ensuring even weight distribution and stability.  
➜ **Easy Component Mounting** – Pre-designed slots for **motor, PCB, steering servo, and camera** make assembly quick and efficient.  
➜ **Secure PCB Mounting** – The **PCB is attached using 4 screws**, keeping it firmly in place.  
➜ **Organized Wiring & Cable Management**  
&nbsp; &nbsp; 🔹 **Integrated PCB routing** eliminates unnecessary wiring, ensuring a cleaner and more reliable setup.  
&nbsp; &nbsp; 🔹 **Hot glue secures servo and camera wires**, preventing loose cables from interfering with movement.  

---

## 🔧 Assembly Process <a id="assembly-process"></a>

🔗 **[Click here to watch the assembly video on YouTube](https://youtu.be/sz8ePobdi_c)** <a id="assembly-process-video"></a>

### 1: 3D Print the Parts <a id="3d-print-the-parts"></a>
The 3D model files are available in the [`/3D-models`](https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/tree/main/3D-models) folder. We used a **BambuLab X1-Carbon**, but any good quality printer will work.

**Our Print Settings:**
- **Material:** PLA or PLA-CF  
- **Layer Height:** 0.12mm  
- **Infill:** 15%  

---

### 2: Assemble the Steering System <a id="2-assemble-the-steering-system"></a>
- Mount the **MG90S servo** into the front slot of the chassis using **2 M2 crews**.
- Add 4 **2mm metal or carbon rod** to act as the steering axles.
- Insert the **steering mounts(left&right)** between the chassis and the top steering small bars.
- In each wheel hub insert a **Lego Axle with Stop** into the 3D print for wheel mounting.

---

### 3: Assemble the drive base <a id="3-assemble-the-drive-base"></a>
- Mount the **Pololu 30:1 Gearmotor** with the bracket using **2 M3 screws**.
- Press-hard on the **D-shaped axle gear** until it will be fixed on the motor axle.
- Attach a **Lego 5 gears Diferential** into its dedicated chassis cut.
- Use **Lego Axles** to secure the differential from both left and right sides.
- Add **Lego Bushes** on both sides to eliminate axle play.

---

### 4: Install the Electronics <a id="4-install-the-electronics"></a>

- Use the **custom-designed PCB** and solder all required components directly onto it:
  - **Arduino Nano ESP32**  
  - **TB6612FNG Motor Driver**  
  - **L7805 Voltage Regulator**  
  - **Power switch** and **start button**  
  - All necessary **pin headers and connectors** based on the schematic.

- Once assembled, fix the PCB to the chassis using **four 3D-printed mounts**.

- Insert the **JSumo 3S 450mAh Li-Po battery** into the 3D-printed mount.

- Mount the **BMI088 IMU** to the motor support with under the PCB by using some double sided tape.  

- Fix the **OpenMV H7 Camera** to the front of the chassis using screws, tilted slightly upward for optimal track and sign visibility.

- Connect all modules:
  - **Camera → UART**
  - **IMU → I2C**
  - **Steering servo → appropriate pins**

- Use **wires of custom lenght** for clean wiring.
- 
- Organize and secure all wiring using **hot glue**.

---

### 5: Attach the Wheels <a id="5-attach-the-wheels"></a>
- Front Wheels:
  - Connect to **Lego Axle with Stop**.
  - Secure with **Bush 1/2** on both sides.
- Rear Wheels:
  - Mount to the **2 Axles** from the differential.
  - Use additional **1/2 Bush** spacers to align height with the front wheels.

---

### 6: Add weight <a id="6-add-weight"></a>
- Place **~100g weight** under the robot chassis
- This will increases grip and stability.
- Use **double side tape or glue** to hold weights in place.

---

### 7: Upload the Code <a id="7-upload-the-code"></a>
- Plug the **Arduino Nano ESP32** into your computer using a USB cable.  
- Launch the **Arduino IDE**, ensure the **ESP32 board package** is installed, and upload the code onto the board.

- Next, connect the **OpenMV H7 Camera** via USB.  
- Open the **OpenMV IDE**, open the corresponding `.py` script, and upload it to the camera.

---

## 🛠️ Power and Sense Management <a id="power-and-sense-management"></a>

The robot's **power and sensing system** is designed for **efficiency, precision, and adaptability**. It consists of an **Arduino Nano ESP32, a Li-Po battery, MPU-6050 IMU, OpenMV H7 Camera, motors, and a voltage regulator**, all working together to ensure **stable operation and accurate navigation**.  

Each component has been carefully selected to provide **optimal performance**, minimize power consumption, and ensure **reliability in competition environments**.

---

### **🔋 Li-Po Battery – 3S 450mAh** <a id="li-po-battery"></a>

The **Li-Po battery** provides a **compact, lightweight, and high-discharge** power source, making it ideal for our robot. The **11.1V output** is suitable for running our **motors and voltage regulator**, ensuring stable power delivery.

| <img src="./other/battery.png" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** 3S Li-Po | **Capacity:** 450mAh |
| **Voltage:** 11.1V | **Discharge Rate:** 25C |
| **Weight:** 38g | **Size:** 56.5 × 31 × 9mm |
| **Output Current:** Varies by load | **Connector Type:** JST |
| 🔗 **[Buy Here](https://www.jsumo.com/jsumo-3s-111-volt-450-mah-lipo-battery)** | **Function:** Powers the entire robot |

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

### 🧭 IMU Sensor – BMI088 <a id="imu-sensor-bmi088"></a>

The **BMI088 IMU** is used to **measure the robot's angular velocity and acceleration**, helping it maintain **stability and precise movement control**. It is essential for **calculating turns and avoiding drift**.

| <img src="https://files.seeedstudio.com/wiki/Grove-6-Axis_Accelerometer-Gyroscope-BMI088/img/main.jpg" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Gyroscope Range:** ±2000°/s | **Accelerometer Range:** ±24g |
| **Interface:** I2C / SPI | **Supply Voltage:** 3.0V – 3.6V |
| **Current Draw:** ~3.2mA | **Weight:** ~1g |
| 🔗 [Buy Here](https://wiki.seeedstudio.com/Grove-6-Axis_Accelerometer&Gyroscope_BMI088/) | **Function:** Tracks orientation & motion |

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

| <img src="https://a.pololu-files.com/picture/0J12418.220.jpg?8f026fe1675b1109ea574290d3d26081" width="300"> | **Specifications** |
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

### 📏 Distance Sensor – JS40F IR Digital Sensor <a id="distance-sensor-js40f"></a>

The **JS40F Digital Infrared Sensor** is used to **detect obstacles or walls in front of the robot** and is especially helpful at the start of the match for determining the **lap direction** when exiting the parking zone. Its fast digital response and reliable detection of black objects make it ideal for this task.

| <img src="https://www.jsumo.com/js40f-digital-infrared-ir-distance-sensor-min-40-cm-range-2780-71-B.jpg" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Detection Range:** 40–80 cm (depends on surface) | **Type:** Reflective IR Digital Output |
| **Voltage Supply:** 3.3V – 5V | **Current Draw:** ~15mA |
| **Signal Type:** Digital (0 = no object, 1 = object detected) | **Reverse Polarity Protection:** Yes |
| **Dimensions:** 17.7mm × 11.5mm × 12.6mm | **Weight:** 4g (with cable) |
| 🔗 [Buy Here](https://www.jsumo.com/js40f-digital-infrared-ir-distance-sensor-min-40-cm-range) | **Use Case:** Detects lap direction when starting from parking |

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

### **⚡ Power Consumption Summary** <a id="power-consumption"></a>

| **Component**                 | **Voltage** | **Avg Current Draw** | **Peak Current** |
|-------------------------------|------------|-----------------------|------------------|
| **Arduino Nano ESP32**        | 5V         | 200mA                 | 500mA            |
| **Drive Motor (x2)**          | 12V        | 240mA (120mA each)    | 3.2A (1.6A each) |
| **Steering Servo MG90S**      | 5V         | 120mA                 | 500mA            |
| **OpenMV H7 Camera**          | 3.3V/5V    | 300mA                 | 400mA            |
| **IMU Sensor BMI088**         | 3.3V       | 3.2mA                 | 4mA              |
| **JS40F Distance Sensor**     | 5V         | 15mA                  | 20mA             |
| **TB6612FNG Motor Driver**    | 5V         | 50mA                  | 100mA            |
| **Voltage Regulator L7805CV** | 7.4V → 5V  | Power Management      | -                |
| **Total Robot Power Usage**   | Mixed      | ~1.4A (Avg)           | ~3.9A (Peak)     |

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

### 🅿️ Starting from Parking <a id="start-from-parking"></a>
At the start of the round, our robot is placed in the designated parking zone (the parking zone is 1.5x robot lenght). To determine the direction of the first lap (clockwise or counterclockwise), we use a JS40F object sensor mounted on the left side of the robot.

- If the sensor returns 1, it detects the wall on the left, meaning the lap must be executed in the clockwise direction, as if we had detected an orange line in first turn.

- If the sensor returns 0, there is no wall on the left, meaning the lap should be counterclockwise, like seeing a blue line in first turn.

Once the direction is determined, the robot performs an initial steering exit maneuver (to left or right) using our custom PD-based turning function:

```cpp
void steer_to_angle(double target_angle, int speed) {
  read_gyro_data();
  double current_angle = gz;                    
  double error = target_angle - current_angle;

  // Determine rotation direction: +1 = turn CW, -1 = turn CCW
  int direction = (error >= 0) ? 1 : -1;

  pid_last_error = 0;

  // Loop until the heading error is within ±10°
  while (abs(error) >= 10.0) {
    // Update gyro data
    read_gyro_data();
    error = target_angle - gz;

    // PD controller:
    pid_error = error * kp + (error - pid_last_error) * kd;
    pid_last_error = error;  

    steer(pid_error * direction);
    move(speed);
  }

  move(0);
  flush_messages();  // Clear any pending commands from the camera so we don't double proces
}

```

The robot turns ~75° toward the main track depending on the detected direction. Following this parking exit, the robot enters its standard operating state:
- If it detects a cube, it enters **FOLLOW_CUBE** mode, where it tries to center itself on the cube using a PD algorithm.
- If the cube gets **too** close, it enters **AVOID_CUBE** mode, passing the cube left or right based on its color.
- After **AVOID_CUBE**, it automatically enters **AFTER_CUBE**, where it centers itself on the center of the track to prepare for searching the next cube.
- When no cube is detected, it enters **PID** mode, in which it simply navigates straight using the gyro—just as in qualification runs.

### 🅿️ Parking <a id="parking"></a>
*To be completed – Explanation of how the robot identifies and executes the parallel parking maneuver at the end of the course.*

---

## 📽️ Performance Videos <a id="performance-videos"></a>
🔗 **[Click here to watch the video on YouTube]()**  

---

## 💰 Cost Analysis <a id="cost-analysis"></a>
We have optimized our robot for **performance vs. cost efficiency**. The total cost includes motors, sensors, electronics, PCB, 3D printing and custom parts.

### 📦 **Components Costs** <a id="components-costs"></a>

| Component                      | Quantity | Unit Price ($) | Total ($)  |
|--------------------------------|----------|----------------|-------------|
| **Arduino Nano ESP32**         | 1        | **21.42**      | **21.42**   |
| **Drive Motor (30:1 HPCB)**    | 1        | **22.45**      | **22.45**   |
| **Steering Servo (MG90S)**     | 1        | **4.05**       | **4.05**    |
| **OpenMV H7 Camera**           | 1        | **80.00**      | **80.00**   |
| **Gyroscope (BMI088)**         | 1        | **8.50**       | **8.50**    |
| **JS40F Distance Sensor**      | 1        | **12.50**      | **12.50**   |
| **LiPo Battery (3S 450mAh)**   | 1        | **8.99**       | **8.99**    |
| **L7805CV Voltage Regulator**  | 1        | **1.50**       | **1.50**    |
| **Lego Spike Wheels**          | 4        | **1.64**       | **6.56**    |
| **Lego Differential**          | 1        | **4.00**       | **4.00**    |
| **Experimental Parts**         |          | **35.00**      | **35.00**   |
| **TOTAL COMPONENT COST**       | -        | **-**          | **204.97**  |

**Experimental Parts**

- **MPU6050**  
  A compact IMU that's easy to integrate. However, it comes with several disadvantages, such as significant drifting over short time, even when using a Kalman filter for correction.

- **Pololu 30:1 Micro Metal Gearmotor**  
  A powerful and compact motor. The main issue is that it was the no encoder version and couldn't add the encoder, making it difficult to achieve precise control.

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
| **1000g filament (PLA and PLA-CF)**       | 1        | **20.00**    | **20.00**     |
| **TOTAL 3D PRINTING COST**                | -        | **-**        | **20.00**     |

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
| **Components**                | **204.97**     |
| **PCB (JLCPCB + Components)** | **5.50**       |
| **3D Printing**               | **20.00**      |
| **Other Materials**           | **9.00**       |
| **TOTAL PROJECT COST**        | **239.47**     |

**Prices are approximate, based on current market prices.*

---

## 📂 Resources <a id="resources"></a>

Below is a list of **external images** used in this repository.
- **[Arduino Nano ESP32](https://store.arduino.cc/cdn/shop/files/ABX00092_01.iso_804x603.jpg?v=1727101612)**
- **[BMI088 IMU Sensor](https://files.seeedstudio.com/wiki/Grove-6-Axis_Accelerometer-Gyroscope-BMI088/img/main.jpg)**
- **[JS40F Distance Sensor](https://www.jsumo.com/js40f-digital-infrared-ir-distance-sensor-min-40-cm-range-2780-71-B.jpg)**  
- **[OpenMV H7 Camera](https://openmv.io/cdn/shop/products/new-cam-v4-angle-hero-web_1000x.jpg?v=1715735352)**
- **[Pololu 30:1 Gearmotor](https://a.pololu-files.com/picture/0J12418.220.jpg?8f026fe1675b1109ea574290d3d26081)**
- **[TB6612FNG Motor Driver](https://www.sparkfun.com/media/catalog/product/cache/a793f13fd3d678cea13d28206895ba0c/1/4/14451-01.jpg)**
- **[MG90S Steering Servo](https://static.optimusdigital.ro/20565-large_default/mg90s-servomotor.jpg)**
- **[L7805CV Voltage Regulator](https://ce8dc832c.cloudimg.io/v7/_cdn_/5D/D0/90/00/0/593365_1.jpg?width=640&height=480&wat=1&wat_url=_tme-wrk_%2Ftme_new.png&wat_scale=100p&ci_sign=32c0b49b36a510891beaad3401e2b2b50bdee888)**
- **[3S 450mAh Li-Po Battery](https://www.jsumo.com/jsumo-3s-111-volt-450-mah-lipo-battery-4126-14-B.jpg)**
  
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

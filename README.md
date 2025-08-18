# WRO2025 Future Engineers – Nerdvana Taurus Team


## **This repository is under updates for upcoming international competition**

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
- [🛠️ Power and Sense Management](#power-and-sense-management)
  - [🔋 Li-Po Battery](#li-po-battery)
  - [🖥️ Arduino Nano ESP32](#arduino-nano-esp32)
  - [🧭 IMU Sensor BMI088](#imu-sensor-bmi088)
  - [📷 OpenMV H7 Camera](#openmv-h7-camera)
  - [⚙️ Drive Motor](#drive-motor)
  - [⚙️ Motor Driver IFX9201SG](#motor-driver)
  - [🔄 Steering Servo MG90S](#steering-servo)
  - [⚙️ Impeller](#impeller)
  - [📏 Pololu PWM Distance Sensor](#distance-sensors)
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
├── 📁 video                 # Videos of our robot
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
Hi! I’m Andrei from Romania, and this is my fifth WRO season. This is my first season in Future Engineers, as before I competed in Robomission category. I am passionate about robotics especially electronics and latest algorithms and tech. Over the years, I have worked on multiple robotics projects, including line followers, sumo bots, and air quality modules. Apart from robotics, I also enjoy cybersecurity, programming, and cycling.

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
  - 🟥 **Red markers** ➜ The vehicle must stay on the **right side of the lane**.
  - 🟩 **Green markers** ➜ The vehicle must stay on the **left side of the lane**.
  
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

The robot's mobility is controlled through **a fully PCB chassis, a servo-based steering system, and a drivetrain featuring a RC differential and axes**. These components work together to ensure smooth, **precise movement with optimized traction, stability, and efficient power management**.


## ⚙️ **Drivebase** <a id="drivebase"></a>

### 🔧 **Drivetrain** <a id="drivetrain"></a>

*To be updated...*

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/technical-draws/motor-bracket-technical-drawing.jpg" width="700">

*To be updated...*

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/technical-draws/gear-D-axle-technical-drawing.jpg" width="700">
<img src="https://hpi-racing.ro/34028-thickbox_default/diferential-complet-arrma-mojave-grom-118-30t-08mod-v2.jpg" width="700">

*To be updated...*

### ⚙️ **Motor – 30:1 Micro Metal Pololu Gearmotor HPCB** <a id="motor"></a>
Following past testing, we selected **a high-power 30:1 Micro Metal Gearmotor (12V)** for the drive system. This motor provides an **optimal balance of speed and torque**, allowing the robot to maintain stability while navigating turns.

| <img src="https://a.pololu-files.com/picture/0J12418.220.jpg?8f026fe1675b1109ea574290d3d26081" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** 30:1 HPCB | **Voltage:** 6V |
| **No-load Speed:** 1000 RPM | **No-load Current:** 120mA |
| **Stall Torque:** ~0.4 kg·cm | **Stall Current:** 1.6A |
| 🔗 **[Buy Here](https://www.pololu.com/product/3038)** | **Function:** Drives the robot |

**Why We Chose This Motor?**  
**- Gear ratio provides suffi cient torque** without sacrificing efficiency.  
**- Compact and lightweight design**, allowing integration into a lightweight robot.  

### 🔌 **IFX9201SG Motor Driver** <a id="motor-driver"></a>

The **IFX9201SG** motor driver is used to control the robot’s high-performance drive or impeller motor with precision and efficiency. It supports PWM-based speed control and direction control while integrating advanced protection features, making it ideal for demanding robotics applications. The IFX9201SG is directly integrated into our PCB, ensuring compact design and reliable communication with the Arduino Nano ESP32.

| <img src="https://assets.lcsc.com/images/lcsc/900x900/20230316_Infineon-Technologies-IFX9201SG_C112633_front.jpg" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** IFX9201SG | **Operating Voltage:** 5.5V – 45V |
| **Logic Voltage:** 3.3V / 5V compatible | **PWM Frequency:** Up to 20 kHz |
| **Max Continuous Current:** 5A | **Max Peak Current:** 8A per channel |
| **Control Interface:** PWM + Direction pins | **Built-in Protections:** Overtemperature, Overcurrent, Undervoltage, Short-to-GND/Battery |
| 🔗 **[Buy Here](https://www.lcsc.com/product-image/C112633.html)** | **Function:** Controls drive motors |


## 🔄 **Steering** <a id="steering"></a>

The **steering system** is a critical part of the robot, ensuring precise maneuverability and smooth turns. Our design is based on a **parallelogram steering mechanism**, where both front wheels turn at the same angle through a single servo-controlled linkage. This setup provides predictable and stable steering, making it easy to use for an autonomous vehicle. Instead of using an **Ackermann steering system**, which requires more complex calculations and linkages, we opted for a **simpler and more lightweight solution** that offers consistent control. Our steering system allows for a maximum turning angle of 80 degrees in both the left and right directions. This range provides precise maneuverability, enabling the robot to navigate sharp turns efficiently while maintaining stability.

![Demo GIF](https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/other/steering.gif)

Our **steering arm is directly connected to the servo**, which moves the two front wheels simultaneously. This ensures that the turning response is immediate and proportional to the servo's motion. The **wheels are mounted on special mounts hubs**, allowing for smooth and precise movement without excessive friction. To ensure **structural integrity and long-term reliability**, the steering system is **assembled using 2 M2 screws on which the hubs can turn**. 

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

*To be updated...*

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

*To be updated...*
---

## 🛠️ Power and Sense Management <a id="power-and-sense-management"></a>

The robot's **power and sensing system** is designed for **efficiency, precision, and adaptability**. It consists of an **Arduino Nano ESP32, a Li-Po battery, BMI088 IMU, OpenMV H7 Camera, motors(drive motor and servo motor), and a voltage regulator**, all working together to ensure **stable operation and accurate navigation**.  

Each component has been carefully selected to provide **optimal performance**, minimize power consumption, and ensure **reliability in competition environments**.

---

### **🔋 Li-Po Battery – 2S 300mAh** <a id="li-po-battery"></a>

The **Li-Po battery** provides a **compact, lightweight, and high-discharge** power source, making it ideal for our robot. The **7.4V output** is suitable for running our **motors and voltage regulator**, ensuring stable power delivery.

| <img src="https://img-va.myshopline.com/image/store/2000408386/1640672930478/GNB3002S60AHV-(4)_1800x.jpeg?w=1000&h=1000" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** 2S Li-Po | **Capacity:** 300mAh |
| **Voltage:** 7.4V | **Discharge Rate:** 60C |
| **Weight:** 12g | **Size:** 13.8 x 12 x 52.5mm |
| **Output Current:** Varies by load | **Connector Type:** JST |
| 🔗 **[Buy Here](https://www.jsumo.com/jsumo-3s-111-volt-450-mah-lipo-battery)** | **Function:** Powers the entire robot |

---

### **🖥️ Arduino Nano ESP32 – Main Controller** <a id="arduino-nano-esp32">

The **Arduino Nano ESP32** provides **high-speed processing, built-in Wi-Fi and Bluetooth, and extensive GPIO capabilities** in a **compact form**. This allows it to handle **sensor data, motor control, and vision processing efficiently**.

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

### **⚙️ Drive Motor – 30:1 Micro Metal Gearmotor HPCB 6V w/ Encoder** <a id="drive-motor">

The **drive motor is responsible for propelling the robot forward**. The **30:1 gearbox** provides an excellent **balance of speed and torque**, while the **built-in encoder** allows for **precise speed control**.

| <img src="https://a.pololu-files.com/picture/0J12418.220.jpg?8f026fe1675b1109ea574290d3d26081" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Type:** Micro Metal Gearmotor | **Gear Ratio:** 30:1 |
| **Voltage:** 6V | **Encoder:** Yes |
| **Current Draw (Avg):** 120mA | **Peak Current:** 1.6A |
| **Weight:** ~10g | **Shaft Diameter:** 3mm |
| 🔗 **[Buy Here](https://www.pololu.com/product/3038)** | **Function:** Drives the robot |

---

### **⚙️ IFX9201SG Motor Driver** <a id="motor-driver"></a>

The **IFX9201SG** motor driver is used to control the robot’s high-performance drive or impeller motor with precision and efficiency. It supports PWM-based speed control and direction control while integrating advanced protection features, making it ideal for demanding robotics applications. The IFX9201SG is directly integrated into our PCB, ensuring compact design and reliable communication with the Arduino Nano ESP32.

| <img src="https://assets.lcsc.com/images/lcsc/900x900/20230316_Infineon-Technologies-IFX9201SG_C112633_front.jpg" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Model:** IFX9201SG | **Operating Voltage:** 5.5V – 45V |
| **Logic Voltage:** 3.3V / 5V compatible | **PWM Frequency:** Up to 20 kHz |
| **Max Continuous Current:** 5A | **Max Peak Current:** 8A per channel |
| **Control Interface:** PWM + Direction pins | **Built-in Protections:** Overtemperature, Overcurrent, Undervoltage, Short-to-GND/Battery |
| 🔗 **[Buy Here](https://www.lcsc.com/product-image/C112633.html)** | **Function:** Controls drive motors |

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

### **⚙️ Impeller for downforce** <a id="impeller">

The **impeller** generates downforce to improve the robot’s grip on the track at high speeds. Powered by a **1020 coreless DC motor**, it delivers extremely high RPM with minimal weight, making it ideal for competitive line follower and robotracer builds. Its low rotor inertia ensures instant acceleration, while the compact size allows for easy integration.

| <img src="https://hyperlinerobotics.com/assets/images/products/impeller.jpg" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Type:** Coreless DC Motor | **Model:** 1020 |
| **Voltage:** 3.7–7.4V | **Shaft Diameter:** 1.0mm |
| **No-Load Speed:** ~53,000 RPM @ 3.7V	 | **	Weight:** ~4.5g |
| **Current Draw (Avg):** ~1A @ 3.7V	 | **Peak Current:** ~2.5A |
| 🔗 **[Buy Here](https://hyperlinerobotics.com/products/impeller.html)** | **Function:** Drives the downforce impeller |

---

### 📏 Distance Sensor – Pololu PWM Distance Sensor <a id="distance-sensors"></a>

The **Pololu Digital Distance Sensor (PW output, 50 cm max)** uses a short-range lidar module and reports distance as the width of a digital pulse (similar to a hobby-servo signal). It’s ideal for reliable, fast obstacle detection and gives you an actual distance reading (3 mm resolution), perfect for lap direction detection at start and for close-range wall sensing.

| <img src="https://a.pololu-files.com/picture/0J11135.1200.jpg?1910ced553e34153046a4c95021a93b3" width="300"> | **Specifications** |
|------------------------------|------------------------------|
| **Detection Range:** 50 cm (depends on surface) | **Type:** Digital pulse width (HIGH-time encodes distance) |
| **Voltage Supply:** 3.0V – 5.5V | **Current Draw:** ~30 mA enabled, ~0.4 mA disabled (EN low) |
| **Resolution:** 3 mm (≈ 4 µs per 3 mm)	 | **Update Rate:** ~50–110 Hz (period ~20–9 ms) |
| **Dimensions:** 21.6mm × 8.9mm × 3.1mm | **Weight:** 1g |
| 🔗 [Buy Here](https://www.pololu.com/product/4064) | **Use Case:** Start-line lap direction & close-range obstacle sensing for parking |

---

### **🔌 L7805CV Voltage Regulator – Power Management** <a id="voltage-regulator"> 

The **L7805CV** regulates the **11.1V Li-Po battery output** to a **stable 5V**, ensuring **safe power delivery** to the **Arduino, sensors, and camera**. It prevents **overvoltage damage** and includes **thermal & short-circuit protection** for reliability.

| <img src="https://assets.lcsc.com/images/lcsc/900x900/20230102_STMicroelectronics-L7805CV_C111887_front.jpg" width="300">  | **Specifications** |
|------------------------------|------------------------------|
| **Model:** L7805CV | **Input Voltage:** 7V – 35V |
| **Output Voltage:** 5V | **Output Current:** 1.5A |
| **Efficiency:** Linear Regulator | **Dropout Voltage:** ~2V |
| **Protection:** Short-circuit & thermal shutdown | **Mounting Type:** TO-220 Package |
| 🔗 **[Buy Here](https://www.lcsc.com/product-detail/C111887.html?s_z=n_L7805C)** | **Function:** Converts battery voltage to 5V |

---

### **🛠️ PCB Design** <a id="pcb-design"></a>
| **Electrical Schematics** | **PCB Design Cooper Traces** | **Chassis after Production** |
|---------------------------|---------------------------|---------------------------|
| <img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/electrical-schematics/Schematic_Chassis.png" width="400"> | <img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/electrical-schematics/PCB_traces.jpg" width="400"> | <img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/electrical-schematics/PCB_board.jpg" width="400"> |

**Advantages of a Custom PCB Chassis**  
✔ **Organized layout** ➜ Prevents loose connections & messy wiring  
✔ **Power stability** ➜ Ensures consistent voltage supply to all components  
✔ **Compact design** ➜ Reduces weight & optimizes space  
✔ **Reliability** ➜ Minimizes risk of failure due to poor wiring  
✔ **Resistance** ➜ Having a full PCB Chassis is more resistent compared to 3D printed parts glued or screwed together

---

### **⚡ Power Consumption Summary** <a id="power-consumption"></a>

| Item / Designators                   | Part            |                    Supply |    Typical Current |    Peak Current | Notes                               |
| ------------------------------------ | --------------- | ------------------------: | -----------------: | --------------: | ----------------------------------- |
| Arduino Nano ESP32                   | MCU board       |                       5 V |             200 mA |          500 mA | Wi-Fi/BLE spikes                    |
| Steering Servo (MG90S)               | Servo           |                       5 V |             120 mA |          500 mA | Peak at start/stall                 |
| OpenMV H7 Camera                     | Camera          |                       5 V |             300 mA |          400 mA | If on 3.3 V, power is lower         |
| BMI088                               | IMU             |                     3.3 V |             3.2 mA |            4 mA | Negligible                          |
| **1020 Coreless Impeller (via ESC)** | Ducted fan      |          **7.4 V (VBAT)** |          **0.6 A** |       **1.8 A** | Load-dependent; startup spikes high |
| Pololu 6 V 30:1 HPCB Motor           | DC motor        |                     \~6 V | \~120 mA (no-load) | \~1.6 A (stall) | Running current depends on load     |
| IFX9201SG (IFX\_A1)                  | Motor driver IC | 5 V (logic), VBAT (motor) |     \~5 mA (logic) | \~10 mA (logic) | Motor current from VBAT             |
| IRFR3411TRPBF-VB (Q1)                | N-MOSFET        |                  VBAT/5 V |           ≈0 mA DC |               — | Gate draw ≈0 DC                     |
| 78M05 (U5)                           | 5 V LDO         |                 7.4 V→5 V |     6 mA quiescent |            6 mA | Plus it sources all 5 V loads       |
| LED5                                 | XL-1608SURC-06  |                       5 V |               5 mA |           10 mA | Red, assumed 5 mA                   |
| LED1–LED4                            | XL-1608UBC-04   |                       5 V |     4×5 mA = 20 mA |         \~40 mA | Blue, assumed 5 mA each             |
| R8,R9,R10,R4–R7                      | Resistors       |                         — |             \~0 mA |               — | Included via LED rows               |
| C3,C4,C5,C6,C8                       | Capacitors      |                         — |                  0 |               — | Reactive only                       |
| RST, START (B3U-1000P)               | Tact switches   |                         — |                  0 |               — | No DC draw                          |

---

## 📝 Obstacle Management <a id="obstacle-management"></a>

### 🏁 Open Round <a id="open-round"></a>
During the **Open Round**, our robot follows a **straight trajectory using a PID controller based on gyro yaw**, ensuring stable movement. To determine turns, the **camera detects Orange and Blue lines** on the track:
- **Orange Line ➜ Right Turn**
- **Blue Line ➜ Left Turn**
- The turn is executed when the robot reaches an approximate **distance from the front black wall**.

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
        direction = 2  # Orange line first ➜ turn right
    elif blue_line:
        direction = 1  # Blue line first ➜ turn left

# Send Direction Command
uart.write(str(direction) + '\n')
```

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/other/line-detection.png" width="900">

### ⚡ Final Round <a id="final-round"></a>

In the final round, we extend our open rount algorithm by adding real‐time cube detection, following, and avoidance algorithms. The OpenMV H7 camera handles live frames processing and sends compact UART messages to the Arduino Nano ESP32. On the Arduino, incoming UART messages drive a four‐state algorithm: in **PID**, the robot adjust to hold a straight heading by using a PD on the gyro and turns 90° whenever it receives a **BLACK** signal (lap turning point in each corner). In **FOLLOW_CUBE**, the camera’s **S<corrected_servo>** message directly sets the servo angle to chase the closest visible cube; if no follow message arrives within 250-500 ms, it returns to **PID** as the cube might have been passed or lost from the view. When a proximity trigger (**RED** or **GREEN**, **R** or **G**) arrives, it switches to **AVOID_CUBE**, executes a 37° turn plus an 8 cm clear‐away while holding that heading, then enters **AFTER_CUBE** to reallign on gyro while moving back to center section of each side of the map, and to flush the leftover commands before reverting to **PID**.

---

### Arduino Side

#### States Driving Logic

This `switch` statement runs inside `void loop()` and decides what the robot does in each of the four states.

```cpp
switch (currentState) {

  case PID:
    {
      // PID straight‐drive, maintain heading using gyro PD
      double err = current_angle_gyro - gz;
      pid_error        = (err) * kp + (pid_error - pid_last_error) * kd;
      pid_last_error   = pid_error;
      steer(pid_error);
      move(robot_speed);
      break;
    }

  case FOLLOW_CUBE:
    {
      // If no follow command arrives within 250-500ms, go back to gyro PD
      if (millis() - last_follow_cube > FOLLOW_CUBE_LOST_TIME) {
        currentState = PID;
      }
      // Otherwise, steer directly toward the cube, so we do active cube following, using a PD that runs on the camera
      steer(follow_cube_angle);
      move(robot_speed);
      break;
    }

  case AVOID_CUBE:
    {
      // Perform the hard coded avoidance maneuver
      pass_cube(cube_avoid_direction);
      break;
    }

  case AFTER_CUBE:
    {
      // Return to PID after getting to be straight again, while ignoring all commands
      while (millis() - last_cube_time > 1500) {
        read_gyro_data();
        double error = current_angle_gyro - gz;
        pid_error = (error)*kp + (pid_error - pid_last_error) * kd;
        pid_last_error = pid_error;
        if (debug == true) {
          Serial.print(current_angle_gyro);
          Serial.print("     |      ");
          Serial.print(turn_direction);
          Serial.print("     |      ");
          Serial.print(gz);
          Serial.print("     |      ");
          Serial.println(robot_speed);
        }
        steer(pid_error);
        move(robot_speed);
      }
      currentState = PID;
      cameraSerial.flush();
      break;
    }
}
```

**PID:**
- Uses a PD loop on the integrated gyro yaw (`gz`) vs. desired heading (`current_angle_gyro`).
- Calls `steer(pid_error)` and `move(robot_speed)` to drive straight.

**FOLLOW_CUBE:**
- If we haven’t received a new `"S…\n"` follow message for at least 250-500 ms, assume the cube is lost or passed and switch back to **PID**.
- Otherwise, use `follow_cube_angle` (sent by the camera) to set the servo and follow the cube.

**AVOID_CUBE:**
- Calls `pass_cube(cube_avoid_direction)`, which executes a turn to left or right based on the cube color. This code sequence is hard coded.
  
**AFTER_CUBE:**
- Return to **PID** after getting to be straight again, while ignoring all commands.


 #### UART Command Parser (execute_command())
 
This function is called whenever a complete line arrives over **UART**. It selects which state to enter (**PID**, **FOLLOW_CUBE**, **AVOID_CUBE**) or adjusts gyro target angle on a **BLACK** message by adding or removing 90 degrees, based on the turn direction.

```cpp
void execute_command(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  // “S…” steering to follow cube ➜ FOLLOW_CUBE
  if (cmd.startsWith("S") && RUN_MODE == 1) {
    float val = cmd.substring(1).toFloat();
    follow_cube_angle = val;
    currentState      = FOLLOW_CUBE;
    last_follow_cube  = millis();

    if (debug) {
      Serial.print("FOLLOW_CUBE angle ➜ ");
      Serial.println(follow_cube_angle);
    }
    return;
  }

  // Map color words to one letter commands and set turn_direction on first BLUE/ORANGE
  char c = cmd.charAt(0);
  if      (cmd.indexOf("BLACK")  != -1) { c = 'B'; }
  else if (cmd.indexOf("BLUE")   != -1) { c = 'L'; lastLineDetectedTime = millis(); if (turn_direction == 0) turn_direction = -1; }
  else if (cmd.indexOf("ORANGE") != -1) { c = 'O'; lastLineDetectedTime = millis(); if (turn_direction == 0) turn_direction =  1; }
  else if (cmd.indexOf("RED")    != -1) { c = 'R'; }
  else if (cmd.indexOf("GREEN")  != -1) { c = 'G'; }
  else if (cmd.indexOf("PINK")   != -1) { c = 'P'; }
  else { return; }  // Ignore any other strings

  // “BLACK” turn 90° if in PID
  if (currentState == PID) {
    if (c == 'B' || (millis() - lastLineDetectedTime > 1800 && lastLineDetectedTime > 0)) {
      if (millis() - lastTurnTime < 1000) {
        if (debug) Serial.println("Ignoring repeated 'B' command due to cooldown.");
        return;
      }
      if (debug) Serial.println("Received 'BLACK' command. Turning 90°...");
      current_angle_gyro += turn_direction * 90;
      turn_count++;
      lastTurnTime         = millis();
      lastLineDetectedTime = 0;
      return;
    }
  }

  // “RED” or “GREEN” close to a cube and need to avoid it ➜ AVOID_CUBE
  if ((c == 'R' || c == 'G') && millis() - last_cube_time >= AVOIDANCE_DRIVE_TIME) {
    cube_avoid_direction = (c == 'R') ? 'L' : 'R';  // 'R' means turn left, 'G' turn right
    currentState         = AVOID_CUBE;
    last_cube_time       = millis();
    if (debug) {
      Serial.print("AVOID_CUBE dir ➜ ");
      Serial.println(cube_avoid_direction);
    }
    return;
  }

  if (debugcam) {
    Serial.print("Ignored cmd ➜ ");
    Serial.println(cmd);
  }
}
```

**“S…” (e.g. `"S+0.120\n"`):**
- Parses the float after `S`, sets `follow_cube_angle` to the value that was sent after S character, and switches to **FOLLOW_CUBE**.
- Records the time to `last_follow_cube` so we know how long it has been since the camera last issued a “follow” command.

**“BLACK”:**
- Only when in **PID**, add or substract 90° based on the lap direction, by adjusting `current_angle_gyro += ±90`.
- Increment `turn_count` and enforce a 1 s cooldown to avoid repeated turning signals.

**“RED” or “GREEN”:**
- When getting to close to a cube, it sets `cube_avoid_direction` based on the cube color and enter **AVOID_CUBE**.


#### Cube Avoidance Subroutine (pass_cube())
When in **AVOID_CUBE**, the robot executes a hardcoded movement, that includes a fixed turn + move forward maneuver to avoid the cube, then transitions to **AFTER_CUBE**, that alligns the robot for doing **PID** and follow the next cube.

```cpp
void pass_cube(char cube_direction) {
  read_gyro_data();
  // Convert 'R' ➜ +1 (turn left), 'G' ➜ -1 (turn right)
  int sign = (cube_direction == 'R') ? 1 : -1;

  double start_angle  = gz;
  double target_angle = start_angle - sign * AVOIDANCE_ANGLE;

  // Turn ~37° away from cube
  move_until_angle(robot_speed, target_angle);

  // Drive ~8 cm forward while holding that heading
  move_cm_gyro(8, robot_speed, target_angle);

  // Move to AFTER_CUBE case
  last_cube_time = millis();
  currentState   = AFTER_CUBE;

  // Discard any leftover UART messages before returning to PID
  flush_messages();
}
```

### Camera Side (OpenMV H7)
In the OpenMV python script, we detect the largest visible red or green blob in the bottom 40% section of the frame, while computing a PD steering error, and send either a small follow command "S±pid_error\n" or an avoid trigger "RED\n"/"GREEN\n" over UART.

**Choose Closest Cube & Follows it by using a PD algorithm**

```python
# ---- Choose Closest Cube (largest red or green) ----
candidates = []
if red_cube:   candidates.append(('R', red_cube))
if green_cube: candidates.append(('G', green_cube))

if candidates:
    # Pick the cube (red or green) with the largest area
    color_char, cube = max(candidates, key=lambda x: x[1].area())
    area = cube.area()
    col  = (255,0,0) if color_char == 'R' else (0,255,0)

    # Draw rectangle and crosshair on the selected cube
    img.draw_rectangle(cube.rect(), color=col)
    img.draw_cross(cube.cx(), cube.cy(), color=col)
    img.draw_string(cube.x(), cube.y() + cube.h() - 10,
                    str(area), color=col)

    # Compute normalized horizontal error in [-1, 1]
    error = (cube.cx() - target_x) / float(target_x)
    pid_error = kp_cube * error + kd_cube * (error - pid_last_error)
    pid_last_error = error

    if area < follow_threshold:
        # Send “S±error\n” to indicate a small servo correction (➜ FOLLOW_CUBE)
        uart.write("S{:+.3f}\n".format(error))
        if DEBUG:
            print("{} FOLLOW ➜ err:{:+.3f}, pid:{:+.3f}, area:{}".format(
                  "RED" if color_char == 'R' else "GREEN",
                  error, pid_error, area))
    else:
        # At close range, send just “RED\n” or “GREEN\n” to trigger avoidance
        uart.write(("RED\n" if color_char == 'R' else "GREEN\n"))
        if DEBUG:
            print("{} CLOSE ➜ area {} >= {}".format(
                  "RED" if color_char == 'R' else "GREEN",
                  area, follow_threshold))
```

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/other/red-cube-detection.png" width="900">

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/other/green-cube-detection.png" width="900">


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
After completing three laps, we’ll:
- **Align so we will be able to turn 90 degrees exactly on the middle section of the track.**
  
- **Determine parking-lot side.**  
   - If the laps were clockwise, the lot sits at the **beginning** of that side.  
   - If the laps were counterclockwise, the lot sits at the **end** of that side.

- **Drive a fixed distance (in cm) forward.**  
   - This moves us into the position from where we can perform our two-step parking.

- **Two-moves parking:**
   - **Move 1:** Turn the steering toward the parking space and reverse into the spot.
   - **Move 2:** Run a short PD loop using gyro to straighten perfectly within the 33 cm (1.5 x robot lenght, 33cm in our case) gap in our case.

<img src="https://github.com/andreipopescufilimon/WRO2025_Future_Engineers/blob/main/other/parking-maneuever.png" width="500">

---

## 📽️ Performance Videos <a id="performance-videos"></a>
🔗 **[Click here to watch the video on YouTube](https://youtu.be/ZELCxp2fEmI?si=_0gKM8rAUfyjFkyQ)**  

---

## 💰 Cost Analysis <a id="cost-analysis"></a>
We have optimized our robot for **performance vs. cost efficiency**. The total cost includes motors, sensors, electronics, PCB, 3D printing and custom parts.

### 📦 **Components Costs** <a id="components-costs"></a>

| Component                      | Quantity | Unit Price ($) | Total ($)  |
|--------------------------------|----------|----------------|-------------|
| **Arduino Nano ESP32**         | 1        | **21.42**      | **21.42**   |
| **Drive Motor 6V (30:1 HPCB)** | 1        | **22.45**      | **22.45**   |
| **IFX9201SG Motor Driver**     | 1        | **19.92**      | **19.92**   |
| **Steering Servo (MG90S)**     | 1        | **4.05**       | **4.05**    |
| **OpenMV H7 Camera**           | 1        | **80.00**      | **80.00**   |
| **Gyroscope (BMI088)**         | 1        | **8.50**       | **8.50**    |
| **Pololu Distance Sensor**     | 4        | **17.95**      | **71.80**   |
| **LiPo Battery (2S 300mAh)**   | 1        | **5.60**       | **5.60**    |
| **L7805CV Voltage Regulator**  | 1        | **1.50**       | **1.50**    |
| **Custom Silicone Wheels**     | 4        | **9.24**       | **36.96**   |
| **RC Differential**            | 1        | **4.00**       | **19.04**   |
| **Experimental Parts**         |          | **35.00**      | **35.00**   |
| **TOTAL COMPONENT COST**       | -        | **-**          | **325.24**  |

**Experimental Parts**

- **MPU6050**  
  A compact IMU that's easy to integrate. However, it comes with several disadvantages, such as significant drifting over short time, even when using a Kalman filter for correction.

- **Pololu 30:1 Micro Metal Gearmotor**  
  A powerful and compact motor. The main issue is that it was the no encoder version and couldn't add the encoder, making it difficult to achieve precise control.

---

### **🔌 PCB Cost (Manufactured via JLCPCB)** <a id="pcb-cost"></a>
| PCB Component                  | Quantity | Unit Price ($) | Total ($)  |
|--------------------------------|----------|--------------|--------------|
| **PCB Manufacturing (JLCPCB)** | 5        | **7.82**     | **39.10**    |
| **PCB Assembly (JLCPCB)**      | 5        | **14.55**    | **72.75**    |
| **TOTAL PCB COST**             | -        | **-**        | **111.85**   |

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
| **M3 Screws & Nuts Set**       | -        | **-**        | **4.00**      |
| **Wiring & Connectors**        | -        | **-**        | **5.00**      |
| **TOTAL OTHER MATERIALS COST** | -        | **-**        | **9.00**      |

---

### **💵 TOTAL** <a id="total-cost"></a>
| Category                      | Total Cost ($) |
|-------------------------------|----------------|
| **Components**                | **325.24**     |
| **PCB (JLCPCB + Components)** | **111.85**     |
| **3D Printing**               | **20.00**      |
| **Other Materials**           | **9.00**       |
| **TOTAL PROJECT COST**        | **466.09**     |

**Prices are approximate, based on current market prices.*

---

## 📂 Resources <a id="resources"></a>

Below is a list of **external images** used in this repository.
- **[Arduino Nano ESP32](https://store.arduino.cc/cdn/shop/files/ABX00092_01.iso_804x603.jpg?v=1727101612)**
- **[BMI088 IMU Sensor](https://files.seeedstudio.com/wiki/Grove-6-Axis_Accelerometer-Gyroscope-BMI088/img/main.jpg)**
- **[OpenMV H7 Camera](https://openmv.io/cdn/shop/products/new-cam-v4-angle-hero-web_1000x.jpg?v=1715735352)**
- **[Pololu 30:1 Gearmotor](https://a.pololu-files.com/picture/0J12418.220.jpg?8f026fe1675b1109ea574290d3d26081)**
- **[MG90S Steering Servo](https://static.optimusdigital.ro/20565-large_default/mg90s-servomotor.jpg)**
- **[L7805CV Voltage Regulator](https://assets.lcsc.com/images/lcsc/900x900/20230102_STMicroelectronics-L7805CV_C111887_front.jpg)**
- **[2S 300mAh Li-Po Battery](https://img-va.myshopline.com/image/store/2000408386/1640672930478/GNB3002S60AHV-(4)_1800x.jpeg?w=1000&h=1000)**
- **[IFX9201SG Motor Driver](https://assets.lcsc.com/images/lcsc/900x900/20230316_Infineon-Technologies-IFX9201SG_C112633_front.jpg)**
- **[Pololu Distance Sensor](https://a.pololu-files.com/picture/0J11135.1200.jpg?1910ced553e34153046a4c95021a93b3)**
  
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

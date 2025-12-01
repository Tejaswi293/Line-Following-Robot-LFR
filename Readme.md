**Advanced Line Follower & SARM Arena Automation**

Kindly Check this drive link for more information: https://drive.google.com/drive/folders/1VdFwt0SDXIHUjQNV9Ow_vTC6IU96wmun?usp=drive_link


**Overview**

This project showcases the design and implementation of an Advanced Line Following Robot (ALFR) integrated with a Single Arm Robot (SARM) for obstacle handling, automated gate triggering, fluid dispensing, and system feedback using relay-based actuators, TFT/LCD display, IR sensors, and relay-controlled pump modules.

The system integrates both physical hardware implementation and conceptual ROS2/CoppeliaSim logic such as gate-triggered events, sensor-based state transitions, PID-based steering, fluid dispensing sequence, and event-driven LED display control.

**Features**

-Line follower with analog IR sensor array (8-channel), PID correction & junction detection

-Real-time pump activation using IR trigger with relay control

-TFT display with SPIFFS-based image rendering and system status updates

-Custom-designed Arena PCB, LAM LED PCB, and motor controller PCB

-3D printed ALFR chassis, wheel shaft couplers, SARM platform, and mechanism links

-Designed to conceptually reflect ROS2 gate triggering, node behavior, and sequential process simulation

**Design Components**
Component	Description
-ALFR	Analog PID-based line follower with Nano and L298N motor driver
-SARM	Mechanically functional, manually controlled obstacle remover
-Arena PCB	Custom ESP32-based interface for power, sensors, relays, and TFT
-Pump System	IR sensor-based fluid dispensing using relay and 12V pump
-Display	SPI-based TFT for “LAM” logo, status, or team identity
-Gate Simulation	IR sensor activation mimicking ROS2/CoppeliaSim conceptual triggers

**Technologies Used**

-ESP32, Arduino Nano

-L298N Driver, Relay Modules, DC Pump

-Analog IR Sensor Array, TFT SPI Display

-3D Printed Mechanical Components

-Conceptual ROS2 Node Behavior

**System Flow**

ALFR follows the path using analog sensor data & PD correction

Upon reaching Gate 1: IR sensor triggers pump relay for fluid dispensing

Gate 2 triggers LAM LED PCB illumination

Final Gate activates TFT screen showing image or message via SPIFFS

SARM is manually used to clear obstacles where required

Flowchart, block diagrams, and full schematics included in documentation file.

**Folder Structure Example**
📁 Hardware_Hustle_Project
│── 📁 CAD_Models
│── 📁 Arena_PCB_Schematics
│── 📁 Code
│    ├── ALFR_PID.ino
│    ├── Pump_Display_Control.ino
│── 📁 Images_and_Simulation
│── 📁 Documentation
│    ├── Final_Report.pdf
│    ├── Logbook.pdf
│── README.md

**Future Enhancements**

Implement full ROS2/CoppeliaSim simulation

Convert manual SARM to autonomous kinematic control

Integrate Load Cell, LCD, and publish MQTT data

**Contributors**

Sai Tejaswi

Shivam Kr. Jha

Swayam Bansal

Ashish Negi

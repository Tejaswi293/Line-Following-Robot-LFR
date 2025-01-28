**Line Following Robot with PID Control**


- This project implements a Line Following Robot (LFR) using Arduino, IR sensors, and motors. The robot uses PID (Proportional-Integral-Derivative) control to ensure smooth and precise movement along the line.

**Features**


- Real-time line detection using IR sensors.
- PID-based control for accurate movement.
- Smooth handling of curves and corners.
- Adjustable PWM speed for different movement actions.

**How It Works**


- The robot uses multiple IR sensors to detect the black line on a light-colored surface. Based on the sensor readings, the PID algorithm adjusts the motor speeds to keep the robot on track.

Sensors: IR sensors provide the input signals.


PID Algorithm: The PID control calculates an error value and adjusts the motors' PWM signals to correct the robot's position.

**Components Used**


- Arduino Uno or compatible microcontroller.
- 5 IR sensors.
- L298N motor driver module.
- Two DC motors with wheels.
- Power supply (e.g., batteries).
- Jumper wires and breadboard.

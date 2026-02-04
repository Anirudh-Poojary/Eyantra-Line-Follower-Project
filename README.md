LINE FOLLOWER BOT (e-Yantra 2025/26)
OVERVIEW

This project focuses on the design and implementation of an autonomous line follower robot developed as part of e-Yantra 2025/26, aligned with the agricultural automation theme. The robot is capable of line navigation, color-based object detection, and pick-and-drop operations, demonstrating reliable real-world performance.

SYSTEM DESCRIPTION

The robot uses an STM32 microcontroller programmed in C using STM32CubeIDE. It integrates multiple sensors and actuators to achieve autonomous navigation and object manipulation in a structured arena.

HARDWARE COMPONENTS

Controller: STM32 Nucleo F103RB
Motors: N20 DC motors with L298N motor driver
Sensors:

IR line sensor array for path detection
TCS3200 color sensor for object identification
Actuator:
Electromagnet with MOSFET driver for pick-and-drop operation
Indicators:
RGB LED for system status and operation indication
Power:
3-cell 12 V Li-Po battery with onboard buck converter
CONTROL AND ALGORITHMS

Line following is implemented using PID-based motion control, enabling smooth and stable navigation. The control logic supports line switching and handles transitions using a mode-based state machine with debounced detection for reliable operation.

Color identification is performed using the TCS3200 color sensor, and detected objects are picked and placed using an electromagnet-based mechanism.

SOFTWARE IMPLEMENTATION

Programming Language: C
Development Environment: STM32CubeIDE
Framework: STM32 HAL
All core logic, including sensor processing, motor control, color detection, and pick-and-drop sequencing, is implemented in main.c.

RESULTS AND VALIDATION

The robot successfully demonstrates:
Stable line-following performance using PID control
Accurate color-based object detection

Reliable pick-and-drop operation using real hardware

These results validate the effectiveness of the control logic and hardware integration.

SUMMARY

The Line Follower Bot developed for e-Yantra 2025/26 demonstrates a robust and modular embedded robotics system, combining autonomous navigation, sensor fusion, and object manipulation suitable for agricultural automation tasks.

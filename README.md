LINE FOLLOWING PICK AND PLACE ROBOT (STM32)
OVERVIEW

This project implements an autonomous line-following robot using an STM32 microcontroller. The system supports line switching, color detection, and an electromagnet-based pick-and-place mechanism for reliable navigation and object handling in a structured arena.

HARDWARE

Controller: STM32 Nucleo F103RB
Motors: N20 DC motors with L298N motor driver
Sensors: IR line sensor array, TCS3200 color sensor
Actuator: Electromagnet with MOSFET driver
Indicator: RGB LED
Power: 3-cell 12 V Li-Po battery with buck converter

CONTROL LOGIC

Line following using PID or Reinforcement Learning based control
Support for line switching
Color-based object handling using color sensor
Mode-based state control with debounced detection
LED indication after operation completion

SOFTWARE

Developed using STM32 HAL
Control logic implemented in main.c
Project built and tested using STM32CubeIDE

SUMMARY

A compact and modular STM32-based robotic system integrating line following, color sensing, and pick-and-place functionality with robust control logic.


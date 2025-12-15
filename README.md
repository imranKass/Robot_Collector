# Robot_servo_tester

Introduction

The primary objective of this project was to extend the TI-RSLK platform beyond mobility and incorporate a Pololu BKT18A servo-based robotic arm for manipulation capabilities. This required synchronizing hardware components, configuring PWM outputs, calibrating servo ranges, and designing software routines for smooth and repeatable motion. Ultimately, the system enables the robot to receive commands such as “HEIGHT 60,” “TILT 90,” or “GRIP 0,” and execute coordinated pick-and-place actions in real time. This report describes the design, implementation, and testing of robotic arm integration. Topics discussed include servo driver development, PWM signal calibration, UART/BLE command processing, and the challenges encountered in aligning software control with physical hardware behavior. The resulting system demonstrates how embedded programming, real-time timing control, and wireless communication can be combined to transform the TI-RSLK into a fully functional robotic manipulation platform.


Block Diagram


<img width="807" height="418" alt="image" src="https://github.com/user-attachments/assets/59cfbe87-4feb-4d30-8404-758d2289a022" />


Results and Video demonstration links

https://drive.google.com/file/d/1F-LqzmM3RJRbTGn6RXuEi6AHfF5_3sBz/view?usp=drive_link

Background and Methodology

This project integrates a three-degree-of-freedom robotic arm onto the TI-RSLK platform to extend its capabilities beyond mobility and enable object manipulation through wireless commands. Several embedded systems concepts were applied throughout the development process, including low-level hardware control using microcontroller peripherals, PWM signal generation for servo actuation, timer configuration and clock management, UART communication, and real-time system debugging. The servos in the robotic arm require precise 50 Hz PWM signals with calibrated pulse widths to represent different joint angles. Generating these signals using Timer A1 required an understanding of capture/compare registers (CCR modules), port multiplexing through SEL0/SEL1, and clock prescaling to achieve accurate timing. Additionally, building a wireless command interface relied on embedded communication principles, namely UART serial data parsing over a BLE module, interrupt-driven data handling, and state-based control logic.
To achieve the project goals, I followed a structured methodology beginning with the mechanical and electrical integration of the Pololu arm onto the TI-RSLK robot. Each servo was mapped to a specific MSP432 timer channel (CCR2, CCR3, CCR4) and configured for PWM output. I developed custom driver functions in C to convert software angles into the correct pulse widths based on calibrated servo ranges. Clock configuration was adjusted by dividing SMCLK to obtain a stable 3 MHz timer source, ensuring accurate 20 ms PWM periods. Once reliable motion control was established, I implemented a UART/BLE communication 
interface that allowed a smartphone to send commands such as “HEIGHT 60,” “TILT 90,” or “GRIP 0,” which the MSP432 parsed and translated into servo movements. Higher-level behaviors, including smooth-motion interpolation and automated pick-and-place sequences, were added to demonstrate coordinated and repeatable motion. System testing involved validating servo ranges, ensuring stable PWM signals, correcting pin-to-CCR mismatches, and iteratively tuning motion limits to avoid mechanical strain. Through this combination of embedded systems concepts timers, PWM, UART, clock management, and real-time debugging, I successfully transformed the TI-RSLK into a wireless-controlled pick-and-place robot system.
mmm

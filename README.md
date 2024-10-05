IMU data fusion using Quaternion EKF for Bosch sensors (BMI160 accelerometer and gyroscope and BMM150 magnetometer).
The program is implemented in ESP32-WROOM-32D, but it can be adapted to work on other MCUs.
The program is for the Bosch BMI160 accelerometer and gyroscope and BMM150 magnetometer, but it can also be used with other sensors just by replacing the sensor reading part in the .ino file.
The program includes calibration for both sensors and tilt compensation for the gyroscope values in the initialization.
The main EKF program is written in C++, but it is called and used in an Arduino code.
Data-ready interrupts from the sensors are used as inputs to the ESP32 and in the program to read new sensor data whenever they becomes available, the interrupts can be removed if needed and this will not affect the program performance.
The EKF still needs some tuning since the result is still a bit noisy. However, it can be used in small robotic applications.
It cannot be used in Drones or applications involving security risks before fixing the noisy output.
A picture is included with the program to show how the sensors are connected to the ESP32 development board on a breadboard.
In attitude control tasks, please build your controller based on the Quaternion form of the rotation and not based on the Euler angles to avoid Gimball lock.

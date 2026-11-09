1. Project Overview

  Rocket Flight Computer
  An embedded system project based on STM32F446RE microcontroller, designed to acquire inertial measurements,
  estimate device orientation, and explore fault detection and recovery mechanisms in embedded software.

  The system uses MPU6050 inertial measurement unit to collect accelerometer and gyroscope data. A complementary filter 
  combines both to estimate pitch and roll values.
  
  The project focuses not only on sensor acquisition and data processing, but also on software architecture, peripheral communication, 
  timing, and fault hunting.
  
  Current status: IMU acquisition, gyroscope calibration, complementary filtering, UART telemetry, I2C communication, 
  bus recovery and under development.

2. Objectives

  * Develop firmware for an STM32 microcontroller using STM32 HAL
  * Implement fault detection and I2C bus recovery mechanisms
  * Estimate position by using complementary filters
  * Implement I2C communication with an external measurement unit
  * Analyse raw data from the peripherals
  * Implement gyroscope bias calibration
  * Establish periodic sensor acquisition
  * Develop modular, maintainable, and reliable embedded software

3. Hardware

  STM32F446RE microcontroller
  
    - Main microcontroller development board
    
  MPU6050(GY-521)
  
    - 6-axis IMU
    
  I2C1
  
    - Sensor communication
    
  USART2
  
    - Serial telemetry
    
  ST-LINK
  
    - Programming and debugging

  Jumper wires
  
  Breadboard
  

6. Software Architecture

7. Implemented Features

8. Sensor acquisition

9. Sensor Fusion

10. Fault Detection and Recovery

11. Telemetry

12. Testing and Results

13. Build and Flash

14. Future Improvements


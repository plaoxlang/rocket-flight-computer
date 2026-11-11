# STM32 based Rocket Flight Computer

## Project Overview

Rocket Flight Computer
An embedded system project based on STM32F446RE microcontroller, designed to acquire inertial measurements,
estimate device orientation, and explore fault detection and recovery mechanisms in embedded software.

The system uses MPU6050 inertial measurement unit to collect accelerometer and gyroscope data. A complementary filter 
combines both to estimate pitch and roll values.

The project focuses not only on sensor acquisition and data processing, but also on software architecture, peripheral communication, 
timing, and fault hunting.

Current status: IMU acquisition, gyroscope calibration, complementary filtering, UART telemetry, I2C communication, 
bus recovery and under development.

## Objectives

* Develop firmware for an STM32 microcontroller using STM32 HAL
* Implement fault detection and I2C bus recovery mechanisms
* Estimate position by using complementary filters
* Implement I2C communication with an external measurement unit
* Analyse raw data from the peripherals
* Implement gyroscope bias calibration
* Establish periodic sensor acquisition
* Develop modular, maintainable, and reliable embedded software

## Hardware

STM32F446RE microcontroller

    Main microcontroller development board
  
MPU6050(GY-521)

    6-axis IMU
  
I2C1

    Sensor communication
  
USART2

    Serial telemetry
  
ST-LINK

     Programming and debugging

Jumper wires

Breadboard


## Software Architecture

<img width="1662" height="832" alt="mermaid-diagram" src="https://github.com/user-attachments/assets/50126825-d741-4ada-b3bf-d9e81efc09e2" />

## Implemented Features

### 1. IMU Data Acquisition

The MPU6050 communicates with STM32 through the I2C1 peripheral.

The firmware reads 14 consecutive bytes starting from the accelerometer output register, retrieving:
* 3-axis accelerometer measurements
* 3-axis gyroscope measurements
* Temperature

Raw measurements are converted into physical units sing corresponding sensor scales.

### 2. Gyroscope Calibration

A stationary calibration procedure estimates gyroscope bias by averaging 1000 consecutive valid measurements.

The calibration procedure includes motion detection to prevent invalid calibration results.

### 3. I2C1 Bus Recovery

The firmware implements an I2C Bus Recovery procedure.

When the connection is lost during communication, the recovery system attempts to restore the bus:
* Deinitializing the I2C peripheral
* Temporarily configuring SCL and SDA as GPIO open-drain outputs
* Sending 9 clock pulses
* Generating an I2C stop condition
* Checking whether the bus has returned to its idle state

### 4. Attitude Estimation

By implementing complementary filters, accelerometer and gyroscope values are combined to estimate roll and pitch.

The gyroscope provides responsiveness, while the accelerometer compensates for long-term integration drift.

### 5. UART Telemetry

The processed data, like attitude estimates, is sent through UART2 to the computer to be stored/screened, allowing real-time monitoring using a 
serial terminal.

## Testing and Results



## Build and Flash

**Requirements**

* STM32F446RE board
* STM32CubeIDE
* STM32CubeMX
* MPU6050 module

**Instructions**

1. Copy this repository
2. Open the project in STM32CubeIDE
3. Connect the NUCLEO-F446RE board through ST-LINK
4. Build the project
5. Flash the firmware onto the microcontroller
6. Open a serial terminal connected to USART2 at 115200 baud.

The system should initialize the MPU6050, perform gyroscope calibration, and start transmitting telemetry.

## Future Improvements

* Improving error handling and recovery policies.
* Adding automated tests for sensor processing algorithms.
* Logging and visualizing telemetry data.
* Integrating a barometric pressure sensor for altitude estimation.
* Evaluating more advanced attitude estimation algorithms.
* Investigating real-time scheduling and interrupt-driven acquisition.

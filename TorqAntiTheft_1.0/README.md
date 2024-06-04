
## Overview
This project is a car parking sensor system that helps drivers park their cars safely and easily. The system uses an HC-SR04 ultrasonic sensor to detect the distance between the car and any obstacles, and displays the distance on a 16x2 LCD screen. The system also includes a buzzer that beeps faster as the car gets closer to an obstacle.

## Hardware
- STM32F401-RE microcontroller
- HC-SR04 ultrasonic sensor
- 16x2 LCD screen
- Buzzer
- Breadboards

## Software
The software is written in C language using the STM32CubeIDE development environment. The code utilizes Timer 2 for PWM signal generation to control the buzzer, and GPIO pins to read the sensor data and display it on the LCD screen.

## Future Improvements
- Implementing a new method to control the loudness of the sensor
- Find more accurate formula to change beeping interval.

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## Function Used
The void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)() function was used to measure the time interval between echo pin in this project. You can find the implementation of the function at https://youtu.be/ti_1ZwRolU4?t=280.
To use this function in your own project, follow the instructions outlined in the video.

![WhatsApp Image 2023-03-26 at 17 22 43](https://user-images.githubusercontent.com/120037992/227782229-3eaf0668-f529-4895-809b-845cb6785aad.jpeg)

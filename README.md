# EXPLOG
This is a battery powered, handheld device for travelers and explorers. It will monitor and log your GPS location, direction, speed and acceleration while you travel and also the temperature, pressure, altitude, humidity, noise level and orientation of the place where you are. So it's an all-in-one device than can gather all the data of the places you'll explore. The saved data can be exported to a computer via USB or WiFi. 

The name is a wordplay on "exploration" and "logger".

This project is under prototyping stage. I'm using ESP32 as the main controller after failing with ESP8266 (NodeMCU) and Arduino Nano. All the sensors are in the form of breakout modules for now. Once I complete the prototyping, I plan to make a custom PCB and a 3D printed case.

If you want to collaborate on this project then you can write code for each module or sensors used in this project. You need to have the modules with you. Currently being used hardwares are,

1. ESP32 Development Board
2. DPS310/BMP180/BMP280 Pressure Sensor
3. LSM303DLHC 3-Axes Accelerometer and Compass
4. ISL1208 RTC
5. Mediatek MT3329 GPS Module
6. Nokia 5110 LCD
7. microSD Card Reader

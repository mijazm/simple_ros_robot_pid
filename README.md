# ROS Robot with PID Control

This package implements a PID Controller for controlling a differential drive rover.
The low level i/o is handled by an Arduino Mega 2560 over rosserial.
The rover is controlled using a joystick.

The ROS code does three things:  
1. Use gamepad to generate a twist message on /cmd_vel topic.  
2. Implements a simple PID controller which reads the wheel encoder ticks and publishes the computed control effort as a twist message to the /pid_control topic.  
3. Calculates odomentry using wheel encoder ticks and publishes it under /odom topic.  

Arduino code can be found in the /firmware folder. Once you open that folder separately in VSCode, you will be recommended to install PlatformIO, if its not already installed. PIO will take care of the rest of the dependencies for the arduino code. reads the encoder values and publishes them to /lwheel and /rwheel topic. It also receives the control message on /pid_control from Raspberry Pi.
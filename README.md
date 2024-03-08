# MineSweeper
The MineSweeper is a capstone engineering project. It is an attempt at helping humanitarian demining organizations build a robot that can automatically detect and map metal detectors, reducing the need for human labour.

## ROS 
Robot Operating Software (ROS) Noetic is used on a Raspberry Pi 4B+ to control the robot. On the [Master](https://github.com/moelhadary/Mine-Sweeper/tree/master) branch, all the ongoing progress with ROS can be seen. ROS is used to create nodes for pwm generation to BLDC motors, endstoppers, controlling servo motors and stepper motors, I2C communication between Pi and ESP32 (used to read feedback and metal detector input) amongst others. Custom messages have also been created. <br><br>

## URDF
On the 'main' branch is the URDF design of the robot. This design was used to make a gazebo simulation of the navigation.
<div align="center">
  <img src="/URDF.png" alt="URDF Model"><br>
  <em>Fig: URDF Model of Robot</em><br><br>
</div> 

The project is set to be finished on April 11, 2024. 

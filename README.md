# MineSweeper
The MineSweeper is a capstone engineering project. It is an attempt at helping humanitarian demining organizations build a robot that can automatically detect and map metal detectors, reducing the need for human labour.

Robot Operating Software (ROS) Noetic is used on a Raspberry Pi 4B+ to control the robot. In the branch 'master', the catkin_ws folder is the workspace from the raspberry pi where all the scripts and ros files are present. ROS is used to read input from an ESP32 using I2C protocol, send pwm signals to BLDC wheels, control the navigation, and all moving parts. 

On the 'main' branch is the URDF design of the robot. This design was used to make a gazebo simulation of the navigation.
<div align="center">
  <img src="/URDF.jpg" alt="URDF Model"><br>
  <em>Fig: URDF Model of Robot</em><br><br>
</div>

The project is set to be finished in April 11, 2024. 

# Creathus AMR Robot - Autonomous Robotic Platform

Creathus AMR Robot is an autonomous robotic platform developed by [Creathus](https://creathus.org.br/) for classroom disinfection environments. Runner Robot is a differential drive mobile platform to perform different tasks in an indoor environments. Runner Robot is based on Robot Operating System (ROS) - see more in [ROS Wiki](https://www.ros.org/).

## ROS Buildfarm Development Branches

Runner Package | Noetic Devel
------------ | ------------- | ------------
drwatson_ros | [![Build Status](https://travis-ci.org/cesarhcq/drwatson.svg?branch=cesar-working)](https://travis-ci.org/github/cesarhcq/drwatson) | [![Build Status](https://travis-ci.org/cesarhcq/drwatson.svg?branch=cesar-working)](https://travis-ci.org/github/cesarhcq/drwatson)

## Install dependencies and follow the installation instructions.

- [x] ROS Noetic-devel: [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu).
- [x] Rosserial: [Package for Arduino - Real Robot](http://wiki.ros.org/rosserial).
- [x] Sick Scan: [Sick Scan Repository](http://wiki.ros.org/sick_scan).


### The robot model is based on Autonomous Robot Platform with Differential drive. (TODO) 

 If you need to add more sensors in your Robot, follow this great tutorial provided by: [Gazebo Sensors](http://gazebosim.org/tutorials/?tut=add_laser). Please, do not forget to add the .dae or .stl extension of the sensors.

## Steps to clone this repository

Create a simple ROS Workspace - if you don't have yet. Following the installation instructions to install in your Notebook.

```
mkdir -p ~/amr_creathus_ws/src && cd ~/amr_creathus_ws

catkin init

cd ~/amr_creathus_ws/src/ 

git clone https://github.com/bruiglesias/mobile-robot-creathus-2023/tree/develop

cd ~/amr_creathus_ws/

catkin_make

```

## Start a simple test of the WUVC mobile Robot.

```
cd ~/amr_creathus_ws/

source devel/setup.bash
```

1. The first lauch is possible to verify a simple world without any obstacles, and the second world is possible to verify a modified world with objectacles

```
roslaunch robot_description display-robot.launch
```

2. Teleoperating the robot

Onpen another terminal

```
roslaunch robot_description robot_description.launch
```

#### Dependencies

- [x] Rosserial: [Package for Arduino - Real Robot](http://wiki.ros.org/rosserial).


Plug the Arduino USB cable in the Raspberry Pi 3. Now, open the Arduino IDE to verify what's the USB port connected. If you are not installed Arduino IDE in the Raspberry Pi 3, you can follow this instructions.

```
sudo apt-get update
```
```
sudo apt-get install arduino arduino-core
```

Now, install the ROSSERIAL

```
sudo apt-get install ros-noetic-rosserial-arduino
```
```
sudo apt-get install ros-noetic-rosserial
```

After IDE Arduino installed, you'll need to install the **ros_lib library**

The link between ROS and Arduino is through the ros_lib library. This library will be used as any other Arduino library. To install the ros_lib library, type the following commands in the Ubuntu terminal:


#### Test Arduino

First you must release permission from arduino ports. At the end of setup, restart the computer.

```
sudo usermod -a -G dialout $USER
```

1. Open Arduino in terminal

```
arduino
```

2. Test libraries of Arduino

```
cd <sketchbook>/libraries
```

```
rosrun rosserial_arduino make_libraries.py .
```

3. Test node of Arduino in ROS manually

```
roscore
```

Open another terminal, and run the command:

```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
```

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

If you want to see the behavior of the low level pid control, use the tool `rqt_multiplot` and upload the file `~/amr_creathus_ws/src/runner/rqt_multiplot.xml`.

```
rosrun rosrun rqt_multiplot rqt_multiplot
```

```
rosrun base_controller base_controller
```

4. Run robot teleoperation automatically

```
roslaunch arduino_controller teleop.launch
```

5. Test robot with Arduino and Encoder automatically

```
roslaunch arduino_controller test_encoder.launch
```

6. Rviz visualization

```
rosrun rviz rviz -d ~/amr_creathus_ws/src/runner/arduino_controller/rviz/rviz_test_arduino.rviz
```

Close the Arduino IDE and open again. Go to **sketchbook** in the Arduino IDE, and you will see the *ROS_LIB*

Verify the *serial_port* connected. In our case is:

> /dev/ttyACM0

### MPU9250 - electric schematic

![MPU9250 - electric schematic](/images/mpu-eletric.png)

1. Implement ROS driver for several 9-DOF IMUs

```
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws

catkin init

cd ~/catkin_ws/src/ 

git clone git@github.com:cesarhcq/i2c_imu.git

cd ~/catkin_ws/

catkin_make
```

Publishes [sensor_msgs::IMU](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html) messages at 10Hz to `/imu/data` topic. 

Specifically populates `angular_velocity` & `linear_acceleration`

2. Include in file of your roslaunch:

```
<!-- Gyro from MPU9250 -->
<include file="$(find i2c_imu)/launch/i2c_imu_auto.launch"/>
```

3. Test control system along with wheel and gyro odometry

```
roslaunch roslaunch base_controller controller_base.launch
```

```
rosrun rviz rviz -d ~/guntherBot_ws/src/GuntherBot/arduino_controller/rviz/rviz_arduino_encoder_gyro.rviz
```

![guntherBOT](/images/guntherBOT_IMU.png)


### WiFi connection between Robot and PC

The Robot has a WiFi access point. 

Assuming that:

* Runner (IP: 10.42.0.1)

* PC (IP: 10.42.0.98)

1. In the PC or Laptop, open a terminal and type the following command:

```ssh -X ubuntu@10.42.0.1```

```password:ubuntu```

2. In the same terminal of the SSH, type:

```export ROS_MASTER_URI=http://10.42.0.1:11311```

```export ROS_IP=10.42.0.1```

Now, you can execute the launch file with roscore information

3. Open a new terminal again in the PC or Laptop and type:

**Does not need to use ssh again!**

```export ROS_MASTER_URI=http://10.42.0.1:11311```

```export ROS_IP=10.42.0.98```

```rosrun rviz rviz```


### Config ros.h

Original

```
#if defined(ESP8266) or defined(ESP32) or defined(ROSSERIAL_ARDUINO_TCP)
  #include "ArduinoTcpHardware.h"
#else
  #include "ArduinoHardware.h"
#endif
```

Modified

```
#if defined(ROSSERIAL_ARDUINO_TCP)
  #include "ArduinoTcpHardware.h"
#else
  #include "ArduinoHardware.h"
#endif
```


```
https://github.com/akshaykulkarni07/esp32_imu

https://arduino.stackexchange.com/questions/63221/remove-gravity-from-accelerometer-of-mpu-6050

https://forum.arduino.cc/t/removing-gravity-from-mpu6050-data/977167

https://github.com/alanprodam/hybrid_detection/blob/dev-2.0/scripts/fusion_filter_kalman.py

https://github.com/akshaykulkarni07/esp32_imu/blob/master/imu_esp_ros/imu_esp_ros.ino

https://github.com/alanprodam/guntherBot/blob/master/base_controller/launch/controller_base.launch
```

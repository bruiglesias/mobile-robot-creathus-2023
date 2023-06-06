# Creathus AMR Robot - Autonomous Robotic Platform

Creathus AMR Robot is an autonomous robotic platform developed by [Creathus](https://creathus.org.br/) for classroom disinfection environments. Runner Robot is a differential drive mobile platform to perform different tasks in an indoor environments. Runner Robot is based on Robot Operating System (ROS) - see more in [ROS Wiki](https://www.ros.org/).

## ROS Buildfarm Development Branches

Runner Package | Noetic Devel
------------ | ------------- | ------------
drwatson_ros | [![Build Status](https://travis-ci.org/cesarhcq/drwatson.svg?branch=cesar-working)](https://travis-ci.org/github/cesarhcq/drwatson) | [![Build Status](https://travis-ci.org/cesarhcq/drwatson.svg?branch=cesar-working)](https://travis-ci.org/github/cesarhcq/drwatson)

## Install dependencies and follow the installation instructions.

- [x] ROS Noetic-devel: [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu).
- [x] ROS Joystick Drivers Stack: [Joystick Driver](https://github.com/ros-drivers/joystick_drivers).
- [x] Rosserial: [Package for Arduino - Real Robot](http://wiki.ros.org/rosserial).
- [x] Sick Scan: [Sick Scan Repository](http://wiki.ros.org/sick_scan).


## Install Lidar Sick TiM 571

The laser is configured with a static IP: 192.168.0.1. Its not recommended to use the same IP. Then, you need to configure a new IP using [SOPAS Engineering Tool](https://www.sick.com/br/pt/sopas-engineering-tool-2020/p/p367244). Follow the instructions below to configure as correct way:

* Download SOPAS with Windows 7 or superior;
  
  * After installation, connect the Sick 571 LiDAR as recommended in the manufacturer manual. Plug the Ethernet cable on the PC and voltage supply (9-28VDC).

* Now, open the SOPAS software and find a new devices. Probably you'll find LiDAR TiM 571. Select "change IP" and you can modify to any IP adress as desired. 
  
  * Click in the 3 vertical points and select "connect"

## :penguin: Linux Machine Configuring.

* After configuration of the LiDAR, open a terminal and make this:

```
cd ~/amr_creathus_ws/src/ 

git clone https://github.com/SICKAG/sick_scan.git

cd ~/amr_creathus_ws/

catkin_make

source devel/setup.bash
```

* Modify the ```sick_tim_5xx.launch file``` and set the ```arg name="hostname" default="put here the IP configured in you sick laser provided by SOPAS"```.

In order to test the Sick LiDAR try this:
```
roslaunch wuvc_sensors sick_tim571.launch
```

If the ```launch file``` is not working, you'll need to configure manually via DHCP. Open a network box and click in "edit connections". Select Ethernet connection and edit it. Select IPV4 settings and complete the address, netmask and gateway.

* Address: 192.168.1.1
* Netmask: 255.255.255.0
* Gateway: 0.0.0.0

Done! Try to run the ```launch file``` again.

## Encoder Connections

The encoder model which we are using is [Encoder model](https://www.filipeflop.com/produto/sensor-de-velocidade-encoder/). We define this encoder as interrupt input (20, 21). 

## Motor controller - VNH5019 pololu

The motor controller used in this project is the [Pololu Dual VNH5019](https://www.pololu.com/product/2507). It is necessary to install the libraries for Arduino board. You can make the download in the [Pololu Github](https://github.com/pololu/dual-vnh5019-motor-shield). In order to unzip the library you need to find the ```skecthbook``` folder, probably in ```$ /home/user/sketchbook/libraries```. Now, restart or open the Arduino IDE to test the example code provided by ```Demo.ino```.



### The robot model is based on Arlo Platform with Differential drive. (TODO) 



 If you need to add more sensors in your Robot, follow this great tutorial provided by: [Gazebo Sensors](http://gazebosim.org/tutorials/?tut=add_laser). Please, do not forget to add the .dae or .stl extension of the sensors.

## Steps to clone this repository

Create a simple ROS Workspace - if you don't have yet. Following the installation instructions to install in your Notebook.

```
mkdir -p ~/amr_creathus_ws/src && cd ~/amr_creathus_ws

catkin init

cd ~/amr_creathus_ws/src/ 

git clone https://gitlab.com/zionstec1/runner.git

git clone git@github.com:ros-drivers/joystick_drivers.git

cd ~/amr_creathus_ws/

catkin_make

```

## Start a simple simulation of the WUVC mobile Robot.

```
cd ~/amr_creathus_ws/

source devel/setup.bash
```

1. The first lauch is possible to verify a simple world without any obstacles, and the second world is possible to verify a modified world with objectacles

```
roslaunch robot_gazebo first.launch
```

```
roslaunch robot_gazebo second.launch
```

2. Teleoperating the robot GuntherBot

Onpen another terminal

```
roslaunch robot_description robot_description.launch
```

## Real Robot - Integration Raspberry Pi 3, Arduino and ROS

After simulation, you need to do the experiments in real world. It is very important to create a environment with objects and obstacles.

### Communication between Arduino and Raspberry Pi 3

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


### How to run camera ros package

First, you must connect a usb camera to raspberry. Verification of the usb ports is indicated to identify the possible camera.

```
ls /dev/video*
```

```
roslaunch base_controller camera_robot.launch
```

```
rosrun rviz rviz -d ~/guntherBot_ws/src/GuntherBot/base_controller/rviz/rviz_camera.rviz
```

### How to run Joystick Xbox Controller

## Install the following dependencies:

- [x] libbluetooth-dev package
- [x] cwiid package

```
sudo apt-get install libbluetooth-dev
```

```
sudo apt-get install libcwiid-dev
```

Now, it will need to download the following repositories listed below. For more information, see the details in the [ROS-Wiki](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick).


1. teleop_twist_joy

```
cd ~/amr_creathus_ws/src/

git clone git@github.com:ros-teleop/teleop_twist_joy.git

cd ~/amr_creathus_ws/catkin_make
```

2. joystick-drivers

```
cd ~/amr_creathus_ws/src/

git clone git@github.com:ros-drivers/joystick_drivers.git

cd ~/amr_creathus_ws/catkin_make
```

### WiFi connection between Robot and PC

The RunnerBot has a WiFi access point ```ssid: ubiquityrobot```. 

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
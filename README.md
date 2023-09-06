# Creathus AMR Robot - Autonomous Robotic Platform

Creathus AMR Robot is an autonomous robotic platform developed by [Creathus](https://creathus.org.br/) for classroom disinfection environments. Runner Robot is a differential drive mobile platform to perform different tasks in an indoor environments. Runner Robot is based on Robot Operating System (ROS) - see more in [ROS Wiki](https://www.ros.org/).

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

git clone git@github.com:bruiglesias/mobile-robot-creathus-2023.git

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
bash runMultiPlot.sh 
```

4. Run robot differential controller

```
rosrun robot_integrations DifferentialRobotController
```

5. Run robot Encoder Subscriber

```
rosrun robot_integrations EncoderSubscriber
```

6. Rviz visualization

```
bash runRviz.sh 
```

Close the Arduino IDE and open again. Go to **sketchbook** in the Arduino IDE, and you will see the *ROS_LIB*

Verify the *serial_port* connected. In our case is:

> /dev/ttyUSB2

### MPU6050 - electric schematic

![MPU6050 - electric schematic](/images/schematic-diagram.png)

[Ref: ESP32 + MPU6050](https://github.com/akshaykulkarni07/esp32_imu)

1. Implement ROS driver:

Publishes [sensor_msgs::IMU](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html) messages at 100Hz to `/data/encoder`, `/data/encoder/filtered`, `/data/tick_encoder` topic. 

Specifically populates `angular_velocity` & `linear_acceleration`

2. Rosrun just rosserial_python:

```
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB2 _baud:=57600
```

3. Include in file of your roslaunch:

```
    <!-- DATA IMU ESP32 -->
    <node name="imu_data" pkg="rosserial_python" type="serial_node.py" output= "screen">
        <param name="port" value="/dev/ttyACM0"/>  
        <param name="baud" value="57600"/>
    </node>
```

3. Test control system along with wheel and gyro odometry in launch:

```
roslaunch robot_description display-robot.launch
```

```
bash runRviz.sh
```

### Connections between ESP32 DevKit v1 and MPU6050 
| ESP32 DevKit v1  | MPU6050 IMU sensor |
| ------------- | ------------- |
| 3V3  | VCC  |
| GND  | GND  |
| D21  | SDA  |
| D22  | SCL  |


### WiFi connection between Robot and PC

The Robot has a WiFi access point. 

Assuming that:

* Robot (IP: 192.168.167.119)

* PC (IP: 192.168.167.165)

1. In the PC or Laptop, open a terminal and type the following command:

```ssh creathus@192.168.167.119```

```password:creathus```

2. In the same terminal of the SSH, type in .bashrc:

```
export ROS_IP=192.168.167.165

export ROS_HOSTNAME=192.168.167.119

export ROS_MASTER_URI=http://192.168.167.119:11311
```

Now, you can execute the launch file with roscore information

3. Open a new terminal again in the PC or Laptop and type:

**Does not need to use ssh again!**

```
export ROS_IP=192.168.167.165

export ROS_HOSTNAME=192.168.167.119

export ROS_MASTER_URI=http://192.168.167.119:11311
```

4. Setting the file runRviz.sh and change the ip `export ROS_MASTER_URI=http://192.168.167.119:11311 && rosrun rviz rviz`:

```
cd ~/amr_creathus_ws/src/mobile-robot-creathus-2023

bash runRviz.sh 
```

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

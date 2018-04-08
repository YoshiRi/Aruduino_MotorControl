# Aruduino_MotorControl

## Requirement & Setup

- Hardware 

PC with ROS and Aruduino IDE.

The japanese document for installing Arduino IDE, check and follow this [page](http://ishi.main.jp/ros/ros_arduino.html).



- ROS package for arduino


```
sudo apt-get install ros-<distro>-rosserial-arduino  ros-<distro>-rosserial
rosrun rosserial_arduino make_libraries.py <your arduino libraries path>
```

- Arduino PID package

Confirm you have ros-lib in your arduino libraries path.

- Arduino Encoder package
by Paul Stoffregen



# Files

## PublishPose

Include following [package](https://github.com/Tellicious/ArduPID-Library/blob/master/ArduPID/examples/Standard_PID/Standard_PID.ino).

 


## How to run 

### Setup ros and arduino serial communication
Check your arduino device in Ubuntu/Windows. 
For my environment, it looks like:

```
% Assume you already has started roscore
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```

My program defines baudrate to be 115200 with the following code. (Normally it should be 57600.)

```c++
nh.getHardware()->setBaud(115200);
```

### teleop_twist_keyboard
Use [this package](http://wiki.ros.org/teleop_twist_keyboard).

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

```

# Robotic Navigation step

## Odometry from LIDAR

### 1. not succeed
use this [package](https://github.com/MAPIRlab/rf2o_laser_odometry)


Before install this, install MRPT libary for robotic calculations.

```
sudo add-apt-repository ppa:joseluisblancoc/mrpt
sudo apt-get update
sudo apt-get install libmrpt-dev mrpt-apps
```
?? Some fatal error happened.

### 2. under test

Clone this [package](https://github.com/ccny-ros-pkg/scan_tools)

```
%in the ~/catkin_ws/src$
git clone https://github.com/ccny-ros-pkg/scan_tools 
sudo apt-get install ros-kinetic-csm
```

Put [this file](https://pastebin.com/ebtxMGAD) to the /src folder and /include. Just follow this [guy](https://github.com/AndLydakis/Sek_Slam/tree/master/laser_scan_matcher).

Original comment is [here](https://answers.ros.org/question/12489/obtaining-nav_msgsodometry-from-a-laser_scan-eg-with-laser_scan_matcher/).

???

### running gmapping

Set the odom publisher to publish the odometry.


See the tf tree.

```
rosrun tf view_frames
evince frames.pdf
```


# memo

## Arduino Connecting issue


With 5ms roop and baudrate 57600 it communication frequency is almost 100Hz.

```
rostopic hz /current_vel 
subscribed to [/current_vel]
average rate: 104.742
        min: 0.004s max: 0.013s std dev: 0.00206s window: 101
```

If you use baudrate 115200, it become 200Hz.

```
rostopic hz /current_vel 
subscribed to [/current_vel]
average rate: 209.468
        min: 0.001s max: 0.022s std dev: 0.00203s window: 201
```

### Problem

rosserial python stops with errors.

```
[WARN] [1523175088.478695]: Serial Port read failure: device reports readiness to read but returned no data (device disconnected or multiple access on port?)
```

Higher baudrate could cause the communication error.

## Share the data with multiple ROS hardware

First, open hosts file to recognize the host and client ROS computer each other.

```
sudo gedit /etc/hosts
```

```
<host ip address> host
<client ip address> client
```


Then, for the host computer,

```
export ROS_MASTER_URI=http://host:11311
export ROS_HOSTNAME=host
```

after that for the client computer,

```
export ROS_MASTER_URI=http://host:11311
export ROS_HOSTNAME=client
```

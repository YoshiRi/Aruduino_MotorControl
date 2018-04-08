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


# memo

## Connecting Issues

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

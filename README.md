# ipa_tablet_height
Tablet height control with roboclaw in ROS platform


Roboclaw motor and encoder control with a specific service (included in the file) and to go to a desired height in ROS platform


## Usage
Just clone the repo into your catkin workspace. It contains the ROS package and the motor controller driver.  Remmeber to make sure ROS has permisions to use the dev port you give it.

```bash
cd <workspace>/src
git clone https://github.com/Jphartogi/ipa_tablet_height
cd <workspace>
catkin_make
source devel/setup.bash
roslaunch roboclaw_node roboclaw.launch
```
to call a service and set to a specific height 

```bash
rosservice call /tabletheight "enc:x"
``` 
you can input the value of x in cm


## Parameters
The launch file can be configure at the command line with arguments, by changing the value in the launch file or through the rosparam server.

|Parameter|Default|Definition|
|-----|----------|-------|
|dev|/dev/ttyACM0|Dev that is the Roboclaw|
|baud|115200|Baud rate the Roboclaw is configured for|
|address|128|The address the Roboclaw is set to, 128 is 0x80|
|max_speed|20|Max speed of the tablet|
|min_speed|20|Minimum speed of the tablet|
|ticks_per_meter|4342.2|The number of encoder ticks per meter of movement|
|height_tolerance|0.1|How accurate the height should be|


# author
Joshua Phartogi https://github.com/Jphartogi

# external source
for the driver of roboclaw in ROS
https://github.com/sonyccd/roboclaw_ros

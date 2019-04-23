# ipa_tablet_height
Tablet height control with roboclaw in ROS platform


Roboclaw motor and encoder control with a specific service and to go to a desired height in ROS platform

# How to run
``` bash
roslaunch roboclaw_node roboclaw.launch
```
to call a service 

```bash
rosservice call /tabletheight "enc:x"
``` 
you can input the value of x in cm

# author
Joshua Phartogi https://github.com/Jphartogi

# external source
for the driver of roboclaw in ROS
https://github.com/sonyccd/roboclaw_ros

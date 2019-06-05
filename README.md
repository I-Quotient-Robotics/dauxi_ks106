# ks106_uart
Ks106 is a ultrasonic transducer, it supposed TTL/485/I2C communication protocol.In this package ,I used 485 to read the data that the transducer detect.    
## Environment  
System: Ubuntu 18.04  
ROS: ROS moledic  
## Version
Ks106_Link: v1.0.0  
## Install  
1. Turn to your workspace and clone the package.
```
git clone https://github.com/bigMH/ks106_uart.git
cd ..
catkin_make
```
2. Turn to the package, copy ks106_port.rules to /etc/udev/rules.d/
```
sudo cp ks106_port.rules /etc/udev/rules.d/
```
3. Run the launch.Most importance:first,Power the ks106,and then run the launch file.
```
roslaunch ks106_uart Ks106_Link_bringup.launch
```
# ks106_uart
Ks106 is a ultrasonic transducer, it supposed TTL/485/I2C communication protocol.In this package ,I used 485 to read the data that the transducer detect.  
When the node working,it will be return a distance data 40 times/s.And it will be publish to ROS.

## Environment  
System: Ubuntu 18.04  
ROS: ROS moledic  
sensor:ks106  
![ks106](https://img-blog.csdnimg.cn/20190605175449766.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L015X1ByZWNpc2lvbg==,size_16,color_FFFFFF,t_70)
## Topic and Message
topic pub could be changed in launch file.The default topic is "/ks106_uart/Ks106_uart".  
Message is Range in the sensor_msgs.  

## Parameters
Some parameters could be changed in launch file.  
1. ks106_pub_topic:Publish in ROS used this parameter as topic.Default:/ks106_uart/Ks106_uart  
2. uart_port:The port used to communicate with ks106.It has been renamed in rules.d and can't change.  
3. baudrate:Baudrate.Default:9600.  
4. frequency:The frequency that ks106 return data.Default:40HZ.  
5. frame_id_1-4:The frame_id in the message.1-4 corresponding the detector con1-con4.Default:test_1-4.  

## Device setting
Ks106 connected to pc with USB serial port module.Make sure the 485-A / 485-B connected with corresponding port. 
The port ks106 uesd has been remap to /dev/ls106_link.   
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="ks106_link" 
```
Once you have change the USB port remap, you can change the launch file about the serial_port value.  
```
<param name = "uart_port" type = "string" value = "/dev/ks106_link"/>
```

## Install  
1. Turn to your workspace and clone the package.  
```
git clone https://github.com/bigMH/ks106_uart.git
```
2. Check and install the dependence package.  
```
rosdep check iqr_ks106_uart
rosdep install iqr_ks106_uart --ignore-src
```
3. Turn to the package, copy ks106_port.rules to /etc/udev/rules.d/  
```
sudo cp ks106_port.rules /etc/udev/rules.d/
```
4. Run the launch.Suggest:first run the launch file and then power the ks106.  
```
roslaunch ks106_uart Ks106_Link_bringup.launch
```

## Deficiency
1. Ks106 just woek on "Single sending Single receiving" model.  
2. Detection range is 0.14m to 3m.Can't change.  
3. The sensor address and other settings can't be changed.Maybe it cna be change in future.  
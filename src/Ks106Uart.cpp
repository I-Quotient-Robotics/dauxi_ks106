#include "Ks106Uart.h"

//init class:Ks106_Uart
DX::Ks106_Uart::Ks106_Uart(ros::NodeHandle& nh) {
  
  add_ = 0xe8;
  reg_ = 0x02;
  ks106_con_ = 0;
  nh.param("frequency", freq_, 20);
  nh.param("baudrate", baudrate_, 9600);
  nh.param<std::string>("uart_port", port_, "/dev/ttyUSB0");
  nh.param<std::string>("frame_id_1", frame_id1, "frame_id_1");
  nh.param<std::string>("frame_id_2", frame_id2, "frame_id_2");
  nh.param<std::string>("frame_id_3", frame_id3, "frame_id_3");
  nh.param<std::string>("frame_id_4", frame_id4, "frame_id_4");
  nh.param<std::string>("ks106_pub_topic", topic_pub_, "Ks106_uart");
  ks106_pub_ = nh.advertise<sensor_msgs::Range>(topic_pub_,10);
}

DX::Ks106_Uart::~Ks106_Uart() {

}

//initialize the uart port
bool DX::Ks106_Uart::UartInit() {

  try{
    ser_.setPort(port_);
    ser_.setBaudrate(baudrate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser_.setTimeout(to);
    ser_.open();                           // try to open the port
  }
  catch(serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open port. Please try again.");
    return false;
  }
  if(ser_.isOpen()) {
    ROS_INFO_STREAM("The port initialize succeed.");
    ser_.flushInput();
    return true;
  }
  else
    return false;
}

//accept ks106 initialize data 
bool DX::Ks106_Uart::Ks106Init() {

  int count;
  while(ser_.available()<47) {}
  ser_.read(UartData_,ser_.available());
  count = UartData_.size();
  //*************************************************************
  for (int i = 0; i < count; ++i)           
  {
    ROS_INFO("data: 0x%02x",UartData_[i]);
  }
  ROS_INFO("size:%ld",UartData_.size());
  //**************************************************************/
  sleep(5);
  ser_.flushInput();
  UartData_.erase(UartData_.begin(),UartData_.begin()+count);
  return true;
}

//read back data function 
int DX::Ks106_Uart::WriteDetect() {
  //defined necessary variable
  ros::Time t;
  double t1;
  uint8_t command_model1[5] = {0x30,0x32,0x34,0x36,0x37};
  uint8_t command,write;
  const uint8_t* cmd = &command;
  //ser_.flushInput();
  //write address
  command = add_;
  write = ser_.write(cmd,1);
  t = ros::Time::now();
  t1 = t.toSec();
  while(ros::Time::now().toSec()-t1<0.001){}
  //write register
  command = reg_;
  write += ser_.write(cmd,1);
  t = ros::Time::now();
  t1 = t.toSec();
  while(ros::Time::now().toSec()-t1<0.001){}
  //write detect command
  command = command_model1[2] + 0x08 * ks106_con_;
  write += ser_.write(cmd,1);
  return 0;
}

bool DX::Ks106_Uart::ReadAndCheck() {

  ros::Time t;
  double t1;
  t = ros::Time::now();
  t1 = t.toSec();
   //read the back data
  while(ser_.available()<2) {
    //wait 100ms without data back,the sensor is dead
    if (ros::Time::now().toSec()-t1>0.100){
      ROS_ERROR("The sensor was dead, please power the sensor.");
      return false;
    }
  }
  ser_.read(UartData_,ser_.available());
  //back data 0xeeee, check the sensor power.
  if (UartData_[0]==0xee && UartData_[1]==0xee) {  
    UartData_.erase(UartData_.begin(),UartData_.begin()+UartData_.size());
    ROS_ERROR("The sensor might lose power,please check the power.");
    return false;
  }
  distance_ = (short(UartData_[0]<< 8 | UartData_[1])) * 0.001;
  //test output
  ROS_INFO("UartData H:%02x UartData L:%02x distance_:%f",UartData_[0],UartData_[1],distance_);
  //delate old data
  UartData_.erase(UartData_.begin(),UartData_.begin()+2);
  return true;
}

//publish the back data with Range mseeage
int DX::Ks106_Uart::PubDistance() {
  if (distance_<=0 || distance_>=2.5) {
    ROS_INFO("Dangerous");
    return -1;
  }
  sensor_msgs::Range ran;
  ran.header.stamp = ros::Time::now();
  switch(ks106_con_) {
    case 0:
    ran.header.frame_id = frame_id1;
    break;
    case 1:
    ran.header.frame_id = frame_id2;
    break;
    case 2:
    ran.header.frame_id = frame_id3;
    break;
    case 3:
    ran.header.frame_id = frame_id4;
    break;
  }
  ran.radiation_type = 0;
  ran.field_of_view = 115.0 / 180.0 * 3.14159;
  ran.min_range = 0.14;
  ran.max_range = 2.50;
  ran.range = distance_;
  ks106_pub_.publish(ran);
  ks106_con_++;
  if (ks106_con_==4)
    ks106_con_ = 0;
  return 0;
}

int DX::Ks106_Uart::Frequency() {
  return freq_;
}

//write command function ,just for test
int DX::Ks106_Uart::WriteCommand() {

  ros::Time t;
  double t1;
  uint8_t command_line2[3] = {0xe8,0x02,0x9c};
  uint8_t command_model2[4] = {0x9c,0x95,0x98,0x75};
  command_line2[0] = add_;
  uint8_t command2;
  const uint8_t* cmd2 = &command2;
  ROS_INFO("Ready to change the model.");
  for (int i = 0; i < 4; i++) {
    command_line2[2] = command_model2[i];
    for (int j = 0; j < 3; j++) {
      command2 = command_line2[j];
      ser_.write(cmd2,1);
      //delay 1ms
      t = ros::Time::now();
      t1 = t.toSec();
      while(ros::Time::now().toSec()-t1<0.001){}
    }
    while(ros::Time::now().toSec()-t1<0.010){}
  }
  sleep(5);
  return 0;
}


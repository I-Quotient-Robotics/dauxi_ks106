#include "ks106_uart.h"

//init class:Ks106_Uart
iqr::Ks106Uart::Ks106Uart(ros::NodeHandle& nh) {
  
  add_ = 0xe8;
  reg_ = 0x02;
  ks106_con_ = 0;
  nh.param("frequency", freq_, 20);
  nh.param("baudrate", baudrate_, 9600);
  nh.param<std::string>("port", port_, "/dev/ttyUSB0");
  nh.param<std::string>("us1_frame_id", frame_id1, "us1_frame_id");
  nh.param<std::string>("us2_frame_id", frame_id2, "us2_frame_id");
  nh.param<std::string>("us3_frame_id", frame_id3, "us3_frame_id");
  nh.param<std::string>("us4_frame_id", frame_id4, "us4_frame_id");
  nh.param<std::string>("ks106_pub_topic", topic_pub_, "ks106_uart");
  ks106_pub_ = nh.advertise<sensor_msgs::Range>(topic_pub_,10);
}

iqr::Ks106Uart::~Ks106Uart() {

}

//initialize the uart port
bool iqr::Ks106Uart::UartInit() {

  try {
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
bool iqr::Ks106Uart::Ks106Init() {

  int count;
  while(ser_.available()<47) {}
  ser_.read(UartData_,ser_.available());
  count = UartData_.size();
  /*************************************************************
  for (int i = 0; i < count; ++i)           
  {
    ROS_INFO("data: 0x%02x",UartData_[i]);
  }
  ROS_INFO("size:%ld",UartData_.size());
  **************************************************************/
  ROS_INFO("The ks106 is working.");
  sleep(5);
  ser_.flushInput();
  UartData_.erase(UartData_.begin(),UartData_.begin()+count);
  return true;
}

bool iqr::Ks106Uart::WriteData(uint8_t command, double t2) {
  ros::Time t;
  double t1;
  int writeflag;
  const uint8_t* cmd = &command;
  writeflag = ser_.write(cmd,1);
  t = ros::Time::now();
  t1 = t.toSec();
  while(ros::Time::now().toSec()-t1<t2) {}
  if (writeflag == 1)
    return true;
  else
    return false;
}
//read back data function 
int iqr::Ks106Uart::WriteDetect() {
  //defined necessary variable
  ros::Time t;
  double t1;
  uint8_t command_model1[5] = {0x30,0x32,0x34,0x36,0x37};
  ser_.flushInput();
  // write address
  WriteData(add_, 0.001);
  // write register
  WriteData(reg_, 0.001);
  // write detect command
  WriteData(command_model1[2] + 0x08 * ks106_con_, 0.0);
  t = ros::Time::now();
  t1 = t.toSec();
   //read the back data
  while(ser_.available()<2) {
    //wait 100ms without data back,the sensor is dead
    if (ros::Time::now().toSec()-t1>0.100) {
      ROS_ERROR("The sensor was dead, please power the sensor.");
      return -1;
    }
  }
  ser_.read(UartData_,ser_.available());
  //back data 0xeeee, check the sensor power.
  if (UartData_[0]==0xee && UartData_[1]==0xee) {  
    UartData_.erase(UartData_.begin(),UartData_.begin()+UartData_.size());
    ROS_ERROR("The sensor might lose power,please check the power.");
    return -2;
  }
  return 0;
}

bool iqr::Ks106Uart::ReadAndCheck() {

  try {
    if (WriteDetect() == 0) {
      distance_ = (short(UartData_[0]<< 8 | UartData_[1])) * 0.001;
      //ROS_INFO("UartData H:%02x UartData L:%02x ks106_us%d:%f",UartData_[0], UartData_[1], ks106_con_+1, distance_);
      UartData_.erase(UartData_.begin(),UartData_.begin()+2);
    }
    else
      return false;
  }
  catch(serial::SerialException& e) {
    while(UartInit() == false)
      sleep(1);          
    }
  catch(serial::IOException& e) {
    while(UartInit() == false)
      sleep(1);
    }
  return true;
}
   
//publish the back data with Range mseeage
int iqr::Ks106Uart::PubDistance() {
  
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
  if (distance_<=0 || distance_>=2.5) {
    ROS_INFO("Dangerous");
    ran.range = -1.0;
  }
  else
    ran.range = distance_;
  ks106_pub_.publish(ran);
  ks106_con_++;
  if (ks106_con_==4)
    ks106_con_ = 0;
  return 0;
}

int iqr::Ks106Uart::Frequency() {
  return freq_;
}

//write command function ,just for test
int iqr::Ks106Uart::WriteCommand() {

  ros::Time t;
  double t1;
  uint8_t command_line2[3] = {0xe8,0x02,0x9c};
  uint8_t command_model2[4] = {0x9c,0x95,0x98,0x75};
  command_line2[0] = add_;
  ROS_INFO("Ready to change the model.");
  for (int i = 0; i < 4; i++) {
    command_line2[2] = command_model2[i];
    for (int j = 0; j < 3; j++) {
      WriteData(command_line2[j], 0.010);
    }
  }
  sleep(5);
  return 0;
}


#include "dauxi_ks106.h"

//init class:Ks106_Uart
IQR::Ks106Uart::Ks106Uart(ros::NodeHandle& nh) {
  
  add_ = 0xe8;
  reg_ = 0x02;
  ks106_con_ = 0;
  nodename_ = ros::this_node::getName();
  nh.param("frequency", freq_, 10);
  if (freq_>12 || freq_<=0) {
    freq_ = 10;
  }
  nh.param("baudrate", baudrate_, 9600);
  // nh.param("detect_model", detect_model_, 2);
  nh.param<std::string>("port", port_, "/dev/dauxi_ks106");
  nh.param<std::string>("us1_frame_id", frame_id1_, "ks106_us1");
  nh.param<std::string>("us2_frame_id", frame_id2_, "ks106_us2");
  nh.param<std::string>("us3_frame_id", frame_id3_, "ks106_us3");
  nh.param<std::string>("us4_frame_id", frame_id4_, "ks106_us4");
  nh.param<std::string>("topic", topic_pub_, "us");
  ks106_pub_ = nh.advertise<sensor_msgs::Range>(topic_pub_,10);
}

IQR::Ks106Uart::~Ks106Uart() {

}

//initialize the uart port
bool IQR::Ks106Uart::UartInit() {

  try {
    ser_.setPort(port_);
    ser_.setBaudrate(baudrate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser_.setTimeout(to);
    ser_.open();                           // try to open the port
  }
  catch(serial::IOException& e) {
    ROS_ERROR_STREAM(nodename_ << ": Unable to open port. Please try again.");
    return false;
  }
  if(ser_.isOpen()) {
    ROS_INFO_STREAM(nodename_ << ": The port initialize succeed.");
    ser_.flushInput();
    return true;
  }
  else
    return false;
}

//accept ks106 initialize data 
bool IQR::Ks106Uart::Ks106Init() {

  int count; while(ser_.available()<47) {}
  ser_.read(UartData_,ser_.available()); 
  count = UartData_.size();
/************************************************************* 
  for (int i = 0;i < count; ++i) {
    ROS_INFO("data: 0x%02x",UartData_[i]); 
  }
  ROS_INFO("size:%ld",UartData_.size());
**************************************************************/
  ROS_INFO_STREAM(nodename_ << ": The ks106 is working.");
  sleep(5); 
  ser_.flushInput();
  UartData_.erase(UartData_.begin(),UartData_.begin()+count); 
  return true;
}

bool IQR::Ks106Uart::WriteData(uint8_t command, double t2) {
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
int IQR::Ks106Uart::WriteDetect() {
  ros::Time t;
  double t1;
  ser_.flushInput();
  WriteData(add_, 0.001);
  WriteData(reg_, 0.001);
  WriteData(detect_cmd_model1[2] + 0x08 * ks106_con_, 0.0);
  t = ros::Time::now();
  t1 = t.toSec();
  while(ser_.available()<2) {
    if (ros::Time::now().toSec()-t1>0.100) {                 //wait 100ms without data back,the sensor is dead
      ROS_ERROR_STREAM(nodename_ << ": The sensor was dead, please power the sensor.");
      return -1;
    }
  }
  ser_.read(UartData_,ser_.available());
  if (UartData_[0]==0xee && UartData_[1]==0xee) {            //back data 0xeeee, check the sensor power.
    UartData_.erase(UartData_.begin(),UartData_.begin()+UartData_.size());
    ROS_ERROR_STREAM(nodename_ << ": The sensor might lose power,please check the power.");
    return -2;
  }
  return 0;
}

bool IQR::Ks106Uart::ReadAndCheck() {

  try {
    if (WriteDetect() == 0) {
      distance_ = (short(UartData_[0]<< 8 | UartData_[1]));
      //ROS_INFO("UartData H:%02x UartData L:%02x ks106_us%d:%f",UartData_[0], UartData_[1], ks106_con_+1, distance_);
      UartData_.erase(UartData_.begin(),UartData_.begin()+2);
    }
    else
      return false;
  }
  catch(serial::SerialException& e) {
    PubDistance(false);
    while(UartInit() == false) {
      sleep(1);
    }          
  }
  catch(serial::IOException& e) {
    PubDistance(false);
    while(UartInit() == false) {
      sleep(1);
    }
  }
  return true;
}

//publish the back data with Range mseeage
int IQR::Ks106Uart::PubDistance(bool flag) {
  sensor_msgs::Range ran;
  ran.min_range = 0.14;
  ran.max_range = 2.50;
  ran.radiation_type = 0;
  ran.header.stamp = ros::Time::now();
  ran.field_of_view = 115.0 / 180.0 * 3.14159;
  if (flag == false) {
    ran.range = -1.0;
    ran.header.frame_id = frame_id1_;
    ks106_pub_.publish(ran);
    ran.header.frame_id = frame_id2_;
    ks106_pub_.publish(ran);
    ran.header.frame_id = frame_id3_;
    ks106_pub_.publish(ran);
    ran.header.frame_id = frame_id4_;
    ks106_pub_.publish(ran);
    return 0;
  }
  if (distance_<=0) {
    ran.range = -1.0;
  }
  ran.range = distance_;
  switch(ks106_con_) {
    case 0:
    ran.header.frame_id = frame_id1_;
    break;
    case 1:
    ran.header.frame_id = frame_id2_;
    break;
    case 2:
    ran.header.frame_id = frame_id3_;
    break;
    case 3:
    ran.header.frame_id = frame_id4_;
    break;
  }
  ks106_pub_.publish(ran);
  ks106_con_++;
  if (ks106_con_== 4)
    ks106_con_ = 0;
  return 0;
}

int IQR::Ks106Uart::Frequency() {
  return 4*freq_;
}

int IQR::Ks106Uart::Ks106Run() {

  bool check_flag = ReadAndCheck();
  PubDistance(check_flag);
  if(!check_flag) {
    Ks106Init();
  }
  return 0;
}

//write command function ,just for test
int IQR::Ks106Uart::WriteCommand() {

  ros::Time t;
  double t1;
  uint8_t command_line2[3] = {0xe8,0x02,0x9c};
  command_line2[0] = add_;
  ROS_INFO_STREAM(nodename_ << ": Ready to change the model.");
  for (int i = 0; i < 4; i++) {
    command_line2[2] = command_model2[i];
    for (int j = 0; j < 3; j++) {
      WriteData(command_line2[j], 0.010);
    }
  }
  sleep(5);
  return 0;
}
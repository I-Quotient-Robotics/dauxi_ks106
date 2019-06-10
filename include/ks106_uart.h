#ifndef KS106UART_H
#define KS106UART_H

#include <vector>
#include <time.h>

#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Range.h"

namespace iqr {
  class Ks106Uart {
  private:
    int ks106_con_,freq_;
    float distance_;
    uint8_t add_,reg_;
    
    std::string port_,frame_id1,frame_id2,frame_id3,frame_id4;
    serial::Serial ser_;
    int baudrate_,index_;
    std::vector<uint8_t> UartData_;
    std::string topic_pub_;
    ros::Publisher ks106_pub_;
  public:
    Ks106Uart(ros::NodeHandle&);
    ~Ks106Uart();
    
    bool UartInit();
    bool Ks106Init();
    bool WriteData(uint8_t, double);
    int WriteDetect();
    bool ReadAndCheck();
    int PubDistance();
    int WriteCommand();
    int Frequency();
  };
}

#endif
#include "ros/ros.h"
#include "ks106_uart.h"

int main(int argc,char** argv) {

  ros::init(argc,argv,"ks106_node");
  ros::NodeHandle n("~");
  IQR::Ks106Uart ks106(n);
  while(ks106.UartInit() == false) {
    sleep(1);          //declare the uart port
  }
  if(!ks106.ReadAndCheck())
    ks106.Ks106Init();
  // ks106.WriteCommand();
  ros::Rate loop_rate(ks106.Frequency());
  while(ros::ok()) {
    ks106.Ks106Run();
    loop_rate.sleep();
  }
  return 0;
}
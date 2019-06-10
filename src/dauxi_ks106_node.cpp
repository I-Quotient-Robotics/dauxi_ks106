#include "ros/ros.h"
#include "dauxi_ks106.h"

int main(int argc,char** argv) {

  ros::init(argc,argv,"ks106_node");
  ros::NodeHandle n("~");
  IQR::Ks106Uart ks106(n);
  while(ks106.UartInit() == false) {
    sleep(1);          //declare the uart port
  }
  //add check code ,set ks106 with launch setting
  ros::Rate loop_rate(ks106.Frequency());
  while(ros::ok()) {
    ks106.Ks106Run();
    loop_rate.sleep();
  }
  return 0;
}
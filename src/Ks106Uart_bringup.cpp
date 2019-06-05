#include "ros/ros.h"
#include "Ks106Uart.h"

int main(int argc,char** argv) {

  ros::init(argc,argv,"ks106_node");
  ros::NodeHandle n("~");
  DX::Ks106_Uart ks106(n);
  while(ks106.UartInit() == false) {
    sleep(1);          //declare the uart port
  }
  ks106.WriteDetect();
  if(!ks106.ReadAndCheck())
    ks106.Ks106Init();
  //ks106.WriteCommand();
  ros::Rate loop_rate(ks106.Frequency());
  while(ros::ok()) {
    try {
      ks106.WriteDetect();
      if(!ks106.ReadAndCheck()){
        ks106.Ks106Init();
        continue;
      }
      ks106.PubDistance();
      loop_rate.sleep();
    }
    catch(serial::SerialException& e){
      while(ks106.UartInit() == false)
        sleep(1);          
    }
  }
  return 0;
}
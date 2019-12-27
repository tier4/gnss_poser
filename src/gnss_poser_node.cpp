#include <ros/ros.h>

#include "gnss_poser/gnss_poser_core.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "gnss_poser");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  GNSSPoser::GNSSPoser node(nh, private_nh);

  ros::spin();

  return 0;
}
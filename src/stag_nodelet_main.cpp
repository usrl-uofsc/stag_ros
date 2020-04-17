#include "stag_ros/stag_nodelet.h"
#include <nodelet/loader.h>
int main(int argc, char** argv) {
  ros::init(argc, argv, "stag_nodelet");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "stag_ros/stag_nodelet", remap, nargv);
  ros::spin();

  return 0;
}
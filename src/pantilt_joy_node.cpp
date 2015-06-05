#include "h4r_pantilt_adapter/PanTiltJoy.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pan_tilt_joy_node");
  pan_tilt_adapter::PanTiltJoy node;
  node.run();
}

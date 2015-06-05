#include "h4r_pantilt_adapter/PanTiltAdapter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pan_tilt_adapter_node");
  pan_tilt_adapter::PanTiltAdapter node;
  node.run();
}


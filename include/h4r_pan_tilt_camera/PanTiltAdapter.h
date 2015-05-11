#ifndef PANTILTADAPTER_H_
#define PANTILTADAPTER_H_

#include <ros/ros.h>
#include <string>
#include <serial/serial.h>
#include <sersyncproto.h>
#include <boost/bind.hpp>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "protocol_definition.h"

namespace pan_tilt_adapter {

using namespace std;

class PanTiltAdapter {
private:
	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::Subscriber sub_quat;
	tf2_ros::TransformBroadcaster tf_broadcaster;

	uint8_t sersync_cmds[];
	uint8_t sersync_header[];
	uint8_t sersync_payload_lens[];

	int tilt;
	int pan;

	serial::Timeout timeout;


	serial::Serial serial_interface;
	string port;

	int servo_rate;
	int tilt_target,
		pan_target;

	geometry_msgs::TransformStamped tf_tilt;
	geometry_msgs::TransformStamped tf_pan;


	void sendbyte(uint8_t byte);
	void QuaternionCallback(const geometry_msgs::Quaternion::ConstPtr& msg);

public:
	PanTiltAdapter();
	virtual ~PanTiltAdapter();
	void run();
};

} /* namespace pan_tilt_adapter */

#endif /* PANTILTADAPTER_H_ */

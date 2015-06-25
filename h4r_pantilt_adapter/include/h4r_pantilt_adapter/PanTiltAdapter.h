#ifndef PANTILTADAPTER_H_
#define PANTILTADAPTER_H_

#include <inttypes.h>
#include <ros/ros.h>
#include <string>
#include <serial/serial.h>
#include <sersyncproto.h>
#include <boost/bind.hpp>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <cfloat>
#include <tf/tf.h>
#include "protocol_definition.h"

namespace pan_tilt_adapter {

using namespace std;

class PanTiltAdapter {
private:
	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::Subscriber sub_quat;
	ros::Publisher pub_joint;

	int tilt;
	int pan;

	serial::Timeout timeout;


	serial::Serial serial_interface;
	string port;

	int servo_rate;
	int tilt_target,
		pan_target;

    sensor_msgs::JointState joint_state;


	void sendbyte(uint8_t byte);
	void QuaternionCallback(const geometry_msgs::Quaternion::ConstPtr& msg);

public:
	PanTiltAdapter();
	virtual ~PanTiltAdapter();
	void run();
};

} /* namespace pan_tilt_adapter */

#endif /* PANTILTADAPTER_H_ */

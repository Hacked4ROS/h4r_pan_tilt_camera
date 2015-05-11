#ifndef PANTILTJOY_H_
#define PANTILTJOY_H_

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Joy.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace pan_tilt_adapter {

class PanTiltJoy {
private:

	ros::NodeHandle n;
	ros::NodeHandle nh;

	ros::Subscriber sub_joy;
	ros::Publisher pub_quat;

	int loop_rate;

	bool pan_invert;
	bool tilt_invert;

	bool pan_min;
	bool pan_max;

	bool tilt_min;
	bool tilt_max;


	int tilt, pan;

	int axis_pan,
		axis_tilt;

	int button_pan_up,
	    button_pan_down,
	    button_tilt_up,
		button_tilt_down;

	bool button_pan_pressed_up,
		 button_pan_pressed_down,
	     button_tilt_pressed_up,
		 button_tilt_pressed_down;

	int axis_tilt_last,
		axis_pan_last;

	int b_rate;

	boost::mutex mutex_targets, mutex_buttons;


	void workerButton();
	void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg);

public:
	PanTiltJoy();
	virtual ~PanTiltJoy();
	void run();
};

} /* namespace pan_tilt_adapter */

#endif /* H4R_PAN_TILT_ADAPTER_SRC_PANTILTJOY_H_ */

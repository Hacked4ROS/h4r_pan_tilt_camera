/*
 * PanTiltJoy.cpp
 *
 *  Created on: May 11, 2015
 *      Author: cyborg-x1
 */

#include "h4r_pan_tilt_adapter/PanTiltJoy.h"

namespace pan_tilt_adapter {

PanTiltJoy::PanTiltJoy()
:n()
,nh("~")
,sub_joy(n.subscribe("joy", 1000, &PanTiltJoy::JoyCallback, this ))
//,pub_quat(n.advertise("cmd_dir",1000,1))
{
	nh.param<int>("loop_rate", loop_rate, 10);
	nh.param<int>("axis_pan", axis_pan, -1);
	nh.param<int>("axis_tilt", axis_tilt, -1);
	nh.param<int>("button_pan_up", button_pan_up, -1);
	nh.param<int>("button_pan_down", button_pan_down, -1);
	nh.param<int>("button_tilt_up", button_tilt_up, -1);
	nh.param<int>("button_tilt_down", button_tilt_down, -1);
	nh.param<int>("button_rate",b_rate,100);
}
PanTiltJoy::~PanTiltJoy() {
	// TODO Auto-generated destructor stub
}

void PanTiltJoy::JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
		if(button_tilt_up >= 0 && button_pan_up >= 0
		   &&button_tilt_down >= 0 && button_pan_up >= 0
		   && button_tilt_up < msg->buttons.size()
		   && button_tilt_down < msg->buttons.size()
		   && button_pan_up < msg->buttons.size()
		   && button_pan_down < msg->buttons.size())
		{
			mutex_buttons.lock();
			button_pan_pressed_up=msg->buttons[button_pan_up];
			button_pan_pressed_down=msg->buttons[button_pan_down];
			button_tilt_pressed_up=msg->buttons[button_tilt_up];
			button_tilt_pressed_down=msg->buttons[button_tilt_down];
			mutex_buttons.unlock();
		}
		else if(axis_tilt >= 0 && axis_pan >= 0
				&& axis_tilt < msg->axes.size()
				&& axis_pan  < msg->axes.size())
		{
			mutex_targets.lock();
			if(   axis_pan_last!=msg->axes[axis_pan]
			   || axis_tilt_last!=msg->axes[axis_tilt]){
				  //pan_target=(msg->axes[axis_pan]+1.0)*90.0;
				  //tilt_target=(msg->axes[axis_tilt]+1.0)*90.0;
			}
			mutex_targets.unlock();
		}
}




void PanTiltJoy::workerButton()
{

	ros::Rate button_rate(b_rate);
	while(ros::ok())
	{
		mutex_buttons.lock();
		mutex_targets.lock();
		if(button_tilt_pressed_up)
		{
			//tilt_target++;
		}

		if(button_tilt_pressed_down)
		{
			//tilt_target--;
		}

		if(button_pan_pressed_up)
		{
			//pan_target++;
		}

		if(button_pan_pressed_down)
		{
			//pan_target--;
		}
        mutex_targets.unlock();
		mutex_buttons.unlock();
		button_rate.sleep();
	}
}



void PanTiltJoy::run()
{
	ros::Rate lrate(loop_rate);

	boost::thread receive(boost::bind(&PanTiltJoy::workerButton,this));

	geometry_msgs::Quaternion q;

	while(ros::ok())
	{
		pub_quat.publish(q);
		ros::spinOnce();
		lrate.sleep();
	}
}


} /* namespace pan_tilt_adapter */

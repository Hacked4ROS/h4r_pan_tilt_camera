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
,sub_joy(n.subscribe<sensor_msgs::Joy>("joy", 1000, &PanTiltJoy::JoyCallback, this ))
,pub_quat(n.advertise<geometry_msgs::Quaternion>("cmd_dir",1000))
,button_pan_down_pressed(false)
,button_pan_up_pressed(false)
,button_tilt_up_pressed(false)
,button_tilt_down_pressed(false)
{
	nh.param<int>("loop_rate", loop_rate, 30);
	nh.param<int>("axis_pan", axis_pan, -1);
	nh.param<int>("axis_tilt", axis_tilt, -1);
	nh.param<int>("button_pan_up", button_pan_up, -1);
	nh.param<int>("button_pan_down", button_pan_down, -1);
	nh.param<int>("button_tilt_up", button_tilt_up, -1);
	nh.param<int>("button_tilt_down", button_tilt_down, -1);
	nh.param<int>("button_rate",b_rate,60);
	nh.param<int>("pan_min",pan_min,0);
	nh.param<int>("pan_max",pan_max,180);
	nh.param<int>("tilt_min",tilt_min,0);
	nh.param<int>("tilt_max",tilt_max,180);
	nh.param<bool>("pan_invert",pan_invert,false);
	nh.param<bool>("tilt_invert",tilt_invert,false);
	nh.param<int>("pan_init",pan,90);
	nh.param<int>("tilt_init",tilt,90);
}
PanTiltJoy::~PanTiltJoy() {}

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
			button_pan_up_pressed=msg->buttons[button_pan_up];
			button_pan_down_pressed=msg->buttons[button_pan_down];
			button_tilt_up_pressed=msg->buttons[button_tilt_up];
			button_tilt_down_pressed=msg->buttons[button_tilt_down];
			mutex_buttons.unlock();
		}
		else if(axis_tilt >= 0 && axis_pan >= 0
				&& axis_tilt < msg->axes.size()
				&& axis_pan  < msg->axes.size())
		{
			mutexPanTilt.lock();
				double p=msg->axes[axis_pan];
				double t=msg->axes[axis_tilt];

				if(pan_invert)p*=-1;
				if(tilt_invert)t*=-1;

				pan=(p+1.0)*90.0;
				tilt=(t+1.0)*90.0;

				publish();
			mutexPanTilt.unlock();
		}
		else
		{
			ROS_ERROR("Setting for joystick control of pan tilt joy node not accepted!");
			exit(1);
		}
}




void PanTiltJoy::workerButton()
{

	ros::Rate button_rate(b_rate);
	while(ros::ok())
	{
		mutex_buttons.lock();
		mutexPanTilt.lock();
		if(button_tilt_up_pressed)
		{
			if(tilt<180) tilt++;
		}

		if(button_tilt_down_pressed)
		{
			if(tilt>0) tilt--;
		}

		if(button_pan_up_pressed)
		{
			if(pan<180) pan++;
		}

		if(button_pan_down_pressed)
		{
			if(pan>0) pan--;
		}
		publish();
        mutexPanTilt.unlock();
        mutex_buttons.unlock();
		button_rate.sleep();
	}
}

void PanTiltJoy::publish()
{
	geometry_msgs::Quaternion q;
	tf::quaternionTFToMsg(tf::createQuaternionFromRPY(tilt/180.0*M_PI,0,pan/180.0*M_PI),q);
	pub_quat.publish(q);
}

void PanTiltJoy::run()
{
	boost::thread receive;
	if(button_tilt_up >= 0 && button_pan_up >= 0
	   &&button_tilt_down >= 0 && button_pan_up >= 0)
	{
		receive=boost::thread(boost::bind(&PanTiltJoy::workerButton,this));
	}
	ros::spin();
	receive.join();
}


} /* namespace pan_tilt_adapter */

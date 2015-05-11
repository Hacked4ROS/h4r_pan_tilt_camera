/*
 * PanTiltAdapter.cpp
 *
 *  Created on: May 7, 2015
 *      Author: cyborg-x1
 */

#include "PanTiltAdapter.h"

namespace pan_tilt_adapter {

PanTiltAdapter::PanTiltAdapter()
:n(),
nh("~"),
sub_quat(n.subscribe("cmd_dir", 1000, &PanTiltAdapter::QuaternionCallback, this )),
tf_broadcaster(),
sersync_cmds(CMD_ARRAY_INIT),
sersync_header(HEADER_ARRAY_INIT),
sersync_payload_lens(PAYLOAD_LEN_ARRAY_INIT),
tilt(RESET_PAN_VALUE),
pan(RESET_TILT_VALUE),
timeout(serial::Timeout::simpleTimeout(1000))
{
	nh.param<string>("interface",port,"/dev/ttyACM0");
	nh.param<int>("servo_rate", servo_rate, 500);
	nh.param<int>("pan_start",pan_target ,RESET_PAN_VALUE);
	nh.param<int>("tilt_start",tilt_target ,RESET_TILT_VALUE);

	if(pan_target<0 || pan_target>180)
	{
		ROS_ERROR("pan_start invalid must be 0<=pan<=180!");
		exit(1);
	}

	if(tilt_target<0 || tilt_target>180)
	{
		ROS_ERROR("tilt_start invalid must be 0<=tilt<=180!");
		exit(1);
	}

	//Get the namespace for tf transform id
	string ns=nh.getNamespace();

	//Replace / with _
	//TODO

	//Delete leading underscores
	while(ns[0]=="_")
	{
		ns.erase(ns.begin());
	}

	tf_pan.header.frame_id=ns+"pan";
	tf_tilt.header.frame_id=ns+"tilt";

    tf_pan.transform.rotation=
    		tf::createQuaternionMsgFromRollPitchYaw(0, 0, pan*M_PI/180.0);
	tf_tilt.transform.rotation=
			tf::createQuaternionMsgFromRollPitchYaw(tilt*M_PI/180.0, 0, 0);

	//Open Serial interface
	serial_interface.setBaudrate(BAUD);
	serial_interface.setPort(port);
	serial_interface.setTimeout(timeout);
	serial_interface.open();
	if(serial_interface.isOpen())
	{
		ROS_INFO("Serial opened!");
	}
	else
	{
		ROS_ERROR("Serial could not be opened!");
		exit(1);
	}
}

PanTiltAdapter::~PanTiltAdapter(){}


void PanTiltAdapter::sendbyte(uint8_t byte)
{
	serial_interface.write(&byte, 1);
}

void PanTiltAdapter::QuaternionCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
	tf::Quaternion q;
	tf::quaternionMsgToTF(*msg,q);
}

void PanTiltAdapter::run()
{

	payload_t payloads;
	INIT_SERSYNCPROTO_DATA(serial_setup,
			 sersync_cmds,
			 sersync_payload_lens,
			 sersync_header,
			 &payloads);

	ros::Rate rate(servo_rate);

	while(ros::ok() && serial_interface.isOpen())
	{
		if(pan!=pan_target)
		{
		  if(pan<pan_target)
		  {
			  pan++;
		  }
		  else
		  {
			  pan--;
		  }
		  payloads.cam_pan=pan;
		  if(pan_target>180.0)pan_target=180;
		  if(pan_target<0)pan_target=0;
		  sersyncproto_send(&serial_setup,
				  	  	  	CAM_TRANSMIT_PAN,
							(uint8_t*)&payloads,
							boost::bind(&PanTiltAdapter::sendbyte, this));
		  tf_pan.transform.rotation=
				  tf::createQuaternionMsgFromRollPitchYaw(0, 0, pan*M_PI/180.0);
		}

		if(tilt!=tilt_target)
		{
		  if(tilt<tilt_target)
		  {
			  tilt++;
		  }
		  else
		  {
			  tilt--;
		  }
		  payloads.cam_tilt=tilt;
		  if(tilt_target>180.0)tilt_target=180;
		  if(tilt_target<0)tilt_target=0;
		  sersyncproto_send(&serial_setup,
				            CAM_TRANSMIT_TILT,
							(uint8_t*)&payloads,
							boost::bind(&PanTiltAdapter::sendbyte, this));
		  tf_tilt.transform.rotation=
				  tf::createQuaternionMsgFromRollPitchYaw(tilt*M_PI/180.0, 0, 0);
		}


		//TF output
		tf_pan.header.stamp=ros::Time::now();
		tf_tilt.header.stamp=ros::Time::now();
		tf_broadcaster.sendTransform(tf_pan);
		tf_broadcaster.sendTransform(tf_tilt);
		tf_pan.header.seq++;
		tf_tilt.header.seq++;

		ros::spinOnce();
		rate.sleep();
	}
}



} /* namespace pan_tilt_adapter */



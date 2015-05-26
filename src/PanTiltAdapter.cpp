/*
 * PanTiltAdapter.cpp
 *
 *  Created on: May 7, 2015
 *      Author: cyborg-x1
 */
#include "h4r_pan_tilt_adapter/PanTiltAdapter.h"


static const uint8_t sersync_cmds[]=CMD_ARRAY_INIT;
static const uint8_t sersync_header[]=HEADER_ARRAY_INIT;
static const uint8_t sersync_payload_lens[]=PAYLOAD_LEN_ARRAY_INIT;

namespace pan_tilt_adapter {

PanTiltAdapter::PanTiltAdapter()
:n(),
nh("~"),
sub_quat(n.subscribe("cmd_dir", 1000, &PanTiltAdapter::QuaternionCallback, this )),
tf_broadcaster(),
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
	while(ns[0]=='_')
	{
		ns.erase(ns.begin());
	}



	tf_pan.header.stamp=ros::Time::now();
	tf_tilt.header.stamp=ros::Time::now();


	tf_pan.header.frame_id=ns+"_pan";
	tf_tilt.header.frame_id=ns+"_tilt";

	tf_tilt.child_frame_id=ns+"_pan";
	tf_pan.child_frame_id=ns+"_foot";


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
	double roll, pitch, yaw;

	tf::Quaternion q;
	tf::quaternionMsgToTF(*msg,q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);



	if(isnan(roll) || isnan(yaw) || isnan(pitch))
	{
		ROS_WARN("roll, pitch or yaw not a number, ignore!");
		return;
	}

	yaw=yaw*180.0/M_PI;
	roll=roll*180.0/M_PI;


	if(yaw<0)
	{
		yaw=0;
		ROS_WARN("pan(yaw) < 0 degree assuming, 0 degree");
	}
	if(yaw>180)
	{
		yaw=180;
		ROS_WARN("pan(yaw) > 180 degree assuming 180 degree");
	}

	if(roll<0)
	{
		roll=0;
		ROS_WARN("tilt(roll) < 0 degree assuming, 0 degree");
	}
	if(roll>180)
	{
		roll=180;
		ROS_WARN("tilt(roll) > 180 degree, assuming 180 degree");
	}

	pan_target=yaw;
	tilt_target=roll;
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


		  boost::function<void (uint8_t)> f2( boost::bind(&PanTiltAdapter::sendbyte, this, _1 ) );

		  sersyncproto_send(&serial_setup,
				  	  	  	CAM_TRANSMIT_PAN,
							(uint8_t*)&payloads,
							f2);
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
							boost::bind(&PanTiltAdapter::sendbyte, this, _1));
		}

		tf_pan.transform.rotation=
			  tf::createQuaternionMsgFromRollPitchYaw(0, 0, pan*M_PI/180.0);
		tf_tilt.transform.rotation=
			  tf::createQuaternionMsgFromRollPitchYaw(tilt*M_PI/180.0, 0, 0);

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



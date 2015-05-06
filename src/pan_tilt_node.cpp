#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"
#include "sersyncproto.h"
#include "protocol_definition.h"
#include <stdio.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

serial::Serial serial_interface;
double base_width=0.44;
int max_speed=4000;
payload_t payloads;
boost::mutex mutex_targets, mutex_buttons;

int tilt=90, pan=90;
int tilt_target=90,
	pan_target=90;
int baudrate,
	rate,
	axis_pan,
	axis_tilt;

int button_pan_up,
    button_pan_down,
    button_tilt_up,
	button_tilt_down;

bool button_pan_pressed_up=false,
	 button_pan_pressed_down=false,
     button_tilt_pressed_up=false,
	 button_tilt_pressed_down=false;

int axis_tilt_last=0,
	axis_pan_last=0;

int brate;

sersyncproto_data_t *proto_data;

void sendbyte(uint8_t byte)
{
	serial_interface.write(&byte, 1);
}
payload_t outputbuf;

void workerButton()
{
	ros::Rate button_rate(brate);
	while(ros::ok())
	{
		mutex_buttons.lock();
		mutex_targets.lock();
		if(button_tilt_pressed_up)
		{
			tilt_target++;
		}

		if(button_tilt_pressed_down)
		{
			tilt_target--;
		}

		if(button_pan_pressed_up)
		{
			pan_target++;
		}

		if(button_pan_pressed_down)
		{
			pan_target--;
		}
        mutex_targets.unlock();
		mutex_buttons.unlock();
		button_rate.sleep();
	}
}

void JoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
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
			  pan_target=(msg->axes[axis_pan]+1.0)*90.0;
			  tilt_target=(msg->axes[axis_tilt]+1.0)*90.0;
		}
		mutex_targets.unlock();
	}
}

void VectorCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	mutex_targets.lock();
		tilt_target=msg->x;
		pan_target=msg->y;
	mutex_targets.unlock();
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "pan_tilt_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  std::string port;
  tf2_ros::TransformBroadcaster tf_broadcaster;

  geometry_msgs::TransformStamped transform_pan, transform_tilt;
  transform_pan.header.frame_id=nh.getNamespace()+"foot";
  transform_tilt.header.frame_id=nh.getNamespace()+"pan";
  transform_pan.child_frame_id=nh.getNamespace()+"pan";
  transform_tilt.child_frame_id=nh.getNamespace()+"tilt";

  transform_tilt.transform.rotation.w=1;
  transform_tilt.transform.rotation.x=0;
  transform_tilt.transform.rotation.y=0;
  transform_tilt.transform.rotation.z=0;

  transform_pan.transform.rotation.w=1;
  transform_pan.transform.rotation.x=0;
  transform_pan.transform.rotation.y=0;
  transform_pan.transform.rotation.z=0;



  transform_tilt.transform.translation.x=0;
  transform_tilt.transform.translation.y=0;
  transform_tilt.transform.translation.z=0;

  transform_pan.transform.translation.x=0;
  transform_pan.transform.translation.y=0;
  transform_pan.transform.translation.z=0;


  //TODO remove and use center transform...
  transform_tilt.transform.translation.z=0.05;

  int srate;

  nh.param<std::string>("interface",port,"/dev/ttyACM0");
  nh.param<int>("baudrate", baudrate, BAUD);

  nh.param<int>("axis_pan", axis_pan, -1);
  nh.param<int>("axis_tilt", axis_tilt, -1);
  nh.param<int>("button_pan_up", button_pan_up, -1);
  nh.param<int>("button_pan_down", button_pan_down, -1);
  nh.param<int>("button_tilt_up", button_tilt_up, -1);
  nh.param<int>("button_tilt_down", button_tilt_down, -1);
  nh.param<int>("servo_rate", srate, 500);
  nh.param<int>("button_rate",brate,100);

  ros::Subscriber sub_deg = n.subscribe("deg", 1000, VectorCallback);
  ros::Subscriber sub_joy = nh.subscribe("joy", 1000, JoyCallback);

  ros::Rate servo_rate(srate);


  //SerSyncProto Setup

  uint8_t cmds[]=CMD_ARRAY_INIT;
  uint8_t payload_lens[]=PAYLOAD_LEN_ARRAY_INIT;
  uint8_t header[]=HEADER_ARRAY_INIT;
  payload_t payloads;
  INIT_SERSYNCPROTO_DATA(serial_setup,cmds,payload_lens,header,&payloads);
  proto_data=&serial_setup;
  serial::Timeout timeout=serial::Timeout::simpleTimeout(1000);

  serial_interface.setBaudrate(baudrate);
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



  boost::thread receive(workerButton);
  uint8_t d=0;

  while(ros::ok() && serial_interface.isOpen())
  {

	  mutex_targets.lock();




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
	      sersyncproto_send(proto_data, CAM_TRANSMIT_PAN, (uint8_t*)&payloads, sendbyte);
		  transform_pan.transform.rotation=tf::createQuaternionMsgFromRollPitchYaw(0, 0, pan*M_PI/180.0);
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
		  sersyncproto_send(proto_data, CAM_TRANSMIT_TILT, (uint8_t*)&payloads, sendbyte);

		  transform_tilt.transform.rotation=tf::createQuaternionMsgFromRollPitchYaw(tilt*M_PI/180.0, 0, 0);

		  //tf::QuaternicreateQuaternionFromRPY(double roll,double pitch,double yaw)



	  }






	  transform_pan.header.stamp=ros::Time::now();
	  transform_tilt.header.stamp=ros::Time::now();

	  tf_broadcaster.sendTransform(transform_pan);
	  tf_broadcaster.sendTransform(transform_tilt);

	  transform_pan.header.seq++;
	  transform_tilt.header.seq++;



	  mutex_targets.unlock();
	  ros::spinOnce();
      servo_rate.sleep();

  }

  ROS_INFO("Waiting for receive thread...");
  receive.join();
}

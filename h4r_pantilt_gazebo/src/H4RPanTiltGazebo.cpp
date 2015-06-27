/*
 * H4RPanTiltGazebo.cpp
 *
 *  Created on: Jun 24, 2015
 *      Author: cyborg-x1
 */

#include <h4r_pantilt_gazebo/H4RPanTiltGazebo.h>

namespace gazebo
{


#define RESET_PAN_VALUE 90
#define RESET_TILT_VALUE 90
enum
{
	PAN,
	TILT
};

H4RPanTiltGazebo::H4RPanTiltGazebo(){};
H4RPanTiltGazebo::~H4RPanTiltGazebo(){};

void H4RPanTiltGazebo::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "DiffDrive" ) );

    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_dir" );
    gazebo_ros_->getParameter<std::string> ( base_frame_, "baseFrame", "pantilt_base_frame" );
    gazebo_ros_->getParameter<double> ( joint_torque_, "servoTorque", 5.0 );
    gazebo_ros_->getParameter<double> ( servo_rate_, "servoRate", 5.0 );
    gazebo_ros_->getParameter<int> ( pan_target_, "panStart", 90 );
    gazebo_ros_->getParameter<int> ( tilt_target_, "tiltStart", 90 );

    joints_.resize ( 2 );
    joints_[PAN] = gazebo_ros_->getJoint ( parent, "panJoint", "pantilt_pan_joint" );
    joints_[TILT] = gazebo_ros_->getJoint ( parent, "tiltJoint", "pantilt_tilt_joint" );
    joints_[PAN]->SetMaxForce ( 0, joint_torque_ );
    joints_[TILT]->SetMaxForce ( 0, joint_torque_ );

    //Init joint message names
    joint_state_.name.resize ( joints_.size() );
    joint_state_.position.resize ( joints_.size() );
    for ( int i = 0; i < 2; i++ ) {
        physics::JointPtr joint = joints_[i];
        math::Angle angle = joint->GetAngle ( 0 );
        joint_state_.name[i] = joint->GetName();
        joint_state_.position[i] = angle.Radian () ;
    }
    pub_joint_.publish ( joint_state_ );





    alive_ = true;

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Quaternion>(command_topic_, 1,
                boost::bind(&H4RPanTiltGazebo::cmdDirCallback,this, _1),
                ros::VoidPtr(), &queue_);

    sub_quat_=gazebo_ros_->node()->subscribe(so);
    pub_joint_=gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states",1000);
    callback_queue_thread_ =
        boost::thread ( boost::bind ( &H4RPanTiltGazebo::QueueThread, this ) );
    // Initialize update rate stuff
    if ( servo_rate_ > 0.0 ) servo_rate_ = 1.0 / servo_rate_;
    else servo_rate_ = 0.0;
    last_update_ = parent->GetWorld()->GetSimTime();
}


void H4RPanTiltGazebo::cmdDirCallback ( const geometry_msgs::Quaternion::ConstPtr& msg )
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

	pan_target_=yaw;
	tilt_target_=roll;
}

void H4RPanTiltGazebo::QueueThread()
{
    static const double timeout = 0.01;
    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void H4RPanTiltGazebo::moveJoints()
{
  joints_[PAN]->SetAngle ( 0, (pan_*M_PI)/180.0  );
  joints_[TILT]->SetAngle ( 0,(tilt_*M_PI)/180.0 );
}

void H4RPanTiltGazebo::UpdateChild()
{
	double secondslast_update = ( parent->GetWorld()->GetSimTime()
			                             - last_update_ ).Double();
    if ( secondslast_update > servo_rate_ ) //Do we have to update the position?
    {
    	if(pan_!=pan_target_)
		{
		  if(pan_<pan_target_)
		  {
			  pan_++;
		  }
		  else
		  {
			  pan_--;
		  }
		  if(pan_target_>180.0)pan_target_=180;
		  if(pan_target_<0)pan_target_=0;
		}

		if(tilt_!=tilt_target_)
		{
		  if(tilt_<tilt_target_)
		  {
			  tilt_++;
		  }
		  else
		  {
			  tilt_--;
		  }
		  if(tilt_target_>180.0)tilt_target_=180;
		  if(tilt_target_<0)tilt_target_=0;
		}

		joint_state_.position[0] = pan_*M_PI/180.0;
		joint_state_.position[1] = tilt_*M_PI/180.0;
		joint_state_.header.stamp= ros::Time::now();

		pub_joint_.publish(joint_state_);
		last_update_+= common::Time ( servo_rate_ );
    }
    this->moveJoints();
}

void H4RPanTiltGazebo::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

GZ_REGISTER_MODEL_PLUGIN ( H4RPanTiltGazebo )
}

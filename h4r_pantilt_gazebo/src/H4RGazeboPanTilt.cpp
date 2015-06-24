/*
 * H4RPanTiltGazebo.cpp
 *
 *  Created on: Jun 24, 2015
 *      Author: cyborg-x1
 */

#include <h4r_pantilt_gazebo/H4RPanTiltGazebo.h>

#include <sdf/sdf.hh>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{

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
    gazebo_ros_->getParameter<double> ( joint_torque_, "wheelTorque", 5.0 );
    gazebo_ros_->getParameter<double> ( servo_rate_, "servoRate", 5.0 );
    gazebo_ros_->getParameter<int> ( pan_target_, "panStart", 0 );
    gazebo_ros_->getParameter<int> ( tilt_target_, "tiltStart", 0 );

    joints_.resize ( 2 );
    joints_[PAN] = gazebo_ros_->getJoint ( parent, "panJoint", "pantilt_pan_joint" );
    joints_[TILT] = gazebo_ros_->getJoint ( parent, "tiltJoint", "pantilt_tilt_joint" );
    joints_[PAN]->SetMaxForce ( 0, joint_torque_ );
    joints_[TILT]->SetMaxForce ( 0, joint_torque_ );
}

void H4RPanTiltGazebo::UpdateChild(){}
void H4RPanTiltGazebo::FiniChild(){}

}

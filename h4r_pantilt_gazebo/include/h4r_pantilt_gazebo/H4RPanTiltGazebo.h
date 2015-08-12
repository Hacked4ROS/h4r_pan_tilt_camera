/*
 * H4RPanTiltGazebo.h
 *
 *  Created on: Jun 24, 2015
 *      Author: cyborg-x1
 */

#ifndef H4RPANTILTGAZEBO_H_
#define H4RPANTILTGAZEBO_H_

//Plugin includes
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

//Model includes
#include <sdf/sdf.hh>

//ROS basic includes
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

//TF includes
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

//Messages
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>

//Boost thread and function binding
#include <boost/thread.hpp>
#include <boost/bind.hpp>


#include <stdio.h>

namespace gazebo
{
  class H4RPanTiltGazebo : public ModelPlugin
  {
    public:
	virtual ~H4RPanTiltGazebo();

	///Model load function (the actual constructor)
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

	///Update function for the model
    void Update(const common::UpdateInfo &info);

    /**
     * Reset function for when the simulation is reset,
     * it resets the model to be centered in pan and tilt.
     */
    void Reset();

    private:

    ///Pointer to model
	physics::ModelPtr parent_;

	///Pointer to Gazebo helper class
	GazeboRosPtr gazebo_ros_;

	///Model update event creation
    event::ConnectionPtr updateConnection;

    ///Pointers to the model joints
	std::vector<physics::JointPtr> joints_;

    ///Quaternion subscriber
	ros::Subscriber sub_quat_;

	/**
	 * Callback for Quaternion message
	 * @param cmd_msg The quaternion message
	 */
	void cmdDirCallback ( const geometry_msgs::Quaternion::ConstPtr& cmd_msg );


	///Joint state publisher
	ros::Publisher pub_joint_;

	///Joint state message
	sensor_msgs::JointState joint_state_;

	///Namespace parameter
	std::string namespace_;

	///Command topic name
	std::string command_topic_;

	///Base frame name
	std::string base_frame_;

	///joint torque setting
	double joint_torque_;

	///Servo rate setting
	double servo_rate_;

	///Setpoint for pan
	int pan_target_;

	///Setpoint for tilt
	int tilt_target_;

	///current tilt state
	int tilt_;

	///current pan state
	int pan_;

    ///time of last update for generating the loop rate
    common::Time last_update_;

	///Custom Callback Queue
	ros::CallbackQueue queue_;

	///Queue callback caller (ros spin)
	boost::thread callback_queue_thread_;

	///Queue callback thread function
	void QueueThread();

	///Interrupt variable for ending
	bool alive_;

	///Update function for joint positions
	void moveJoints();
  };

}
#endif /* H4RPANTILTGAZEBO_H_ */

/*
 * H4RPanTiltGazebo.h
 *
 *  Created on: Jun 24, 2015
 *      Author: cyborg-x1
 */

#ifndef H4RPANTILTGAZEBO_H_
#define H4RPANTILTGAZEBO_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <gazebo_plugins/gazebo_ros_utils.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <sdf/sdf.hh>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <tf/tf.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <stdio.h>

namespace gazebo
{
  class H4RPanTiltGazebo : public ModelPlugin
  {
    public:
	virtual ~H4RPanTiltGazebo();


	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void Update(const common::UpdateInfo &info);
    void Reset();

    private:
    physics::ModelPtr model_;
	GazeboRosPtr gazebo_ros_;

    event::ConnectionPtr updateConnection;

	void cmdDirCallback ( const geometry_msgs::Quaternion::ConstPtr& cmd_msg );
	void moveJoints();

	ros::Subscriber sub_quat_;
	ros::Publisher pub_joint_;

	physics::ModelPtr parent_;

	std::vector<physics::JointPtr> joints_;

	std::string namespace_;
	std::string command_topic_;
	std::string base_frame_;
	sensor_msgs::JointState joint_state_;


	double joint_torque_;
	int pan_target_;
	int tilt_target_;
	int tilt_;
	int pan_;

	double servo_rate_;

    double tilt_spd_;
    double pan_spd_;

	// Custom Callback Queue
	ros::CallbackQueue queue_;
	boost::thread callback_queue_thread_;
	void QueueThread();
	bool alive_;

	common::Time last_update_;
  };

}
#endif /* H4RPANTILTGAZEBO_H_ */

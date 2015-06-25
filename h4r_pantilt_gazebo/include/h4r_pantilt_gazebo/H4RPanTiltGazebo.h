/*
 * H4RPanTiltGazebo.h
 *
 *  Created on: Jun 24, 2015
 *      Author: cyborg-x1
 */

#ifndef H4RPANTILTGAZEBO_H_
#define H4RPANTILTGAZEBO_H_


#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
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

namespace gazebo {

class H4RPanTiltGazebo : public ModelPlugin {


public:
	H4RPanTiltGazebo();
	virtual ~H4RPanTiltGazebo();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

	  protected:
		virtual void UpdateChild();
		virtual void FiniChild();

private:
	void cmdDirCallback ( const geometry_msgs::Quaternion::ConstPtr& cmd_msg );
	void publishPanTiltJointState();

	ros::Subscriber sub_quat_;
	ros::Publisher pub_joint_;


	GazeboRosPtr gazebo_ros_;
	physics::ModelPtr parent;
	event::ConnectionPtr update_connection_;

	std::vector<physics::JointPtr> joints_;
	std::string robot_namespace_;
	std::string command_topic_;
	std::string base_frame_;
	sensor_msgs::JointState joint_state_;


	double joint_torque_;
	int pan_target_;
	int tilt_target_;
	int tilt;
	int pan;


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


}/*namespace gazebo*/

#endif /* H4RPANTILTGAZEBO_H_ */

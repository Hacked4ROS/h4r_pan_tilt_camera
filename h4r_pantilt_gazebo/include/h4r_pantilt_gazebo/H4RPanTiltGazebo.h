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

#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

class H4RPanTiltGazebo : public ModelPlugin {


public:
	H4RPanTiltGazebo();
	~H4RPanTiltGazebo();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

	  protected:
		virtual void UpdateChild();
		virtual void FiniChild();

private:
	void cmdDirCallback ( const geometry_msgs::Quaternion::ConstPtr& cmd_msg );

	ros::Subscriber sub_quat_;
	ros::Publisher pub_joint_;

	boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;

	GazeboRosPtr gazebo_ros_;
	physics::ModelPtr parent;
	event::ConnectionPtr update_connection_;

	std::vector<physics::JointPtr> joints_;
	std::string robot_namespace_;
	std::string command_topic_;
	std::string base_frame_;

	double joint_torque_;
	int pan_target_;
	int tilt_target_;

	double servo_rate_;
};


}/*namespace gazebo*/

#endif /* H4RPANTILTGAZEBO_H_ */

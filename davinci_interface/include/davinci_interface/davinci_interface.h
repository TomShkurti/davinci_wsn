#ifndef DAVINCI_INT_H
#define	DAVINCI_INT_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <davinci_kinematics/davinci_kinematics.h>

namespace davinci_interface {

	const std::string OUTPUT_JOINT_NAMES[13] = {
		"joint1_position_controller"	,
		"joint2_position_controller"	,
		"joint2_1_position_controller"	,
		"joint2_2_position_controller"	,
		"joint2_3_position_controller"	,
		"joint2_4_position_controller"	,
		"joint2_5_position_controller"	,
		"joint3_position_controller"	,
		"joint4_position_controller"	,
		"joint5_position_controller"	,
		"joint6_position_controller"	,
		"joint7_position_controller"	,
		"joint7_position_controller_mimic"
	};
	const std::string INPUT_JOINT_NAMES[7] = {
		"outer_yaw"			,
		"outer_pitch"			,
		"outer_insertion"		,
		"outer_roll"			,
		"outer_wrist_pitch"		,
		"outer_wrist_yaw"		,
		"jaw"	
	};
	
	//May want to expand this later on, give it functions for single arms, etc.
	std::vector<std::vector<double> > expand_joint_list(const double input[14]);
	
	void init_joint_control(ros::NodeHandle & nh);
	
	bool publish_joints(const double input[14]);
	
	void init_joint_feedback(ros::NodeHandle & nh);
	
	bool get_fresh_robot_pos(std::vector<std::vector<double> > & output);
};

#endif	/* DAVINCI_INT_H */

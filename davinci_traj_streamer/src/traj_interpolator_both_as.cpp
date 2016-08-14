#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <davinci_kinematics/davinci_kinematics.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <davinci_traj_streamer/trajAction.h>

//==================================================THEORY OF OPERATION====================================
//	The trajectory interpolator as written previously suffered a serious issue where upon recieving an 
//action, between one quarter and one sixth of the time it would immediately get stuck in a "RECALLED" state
//and never serve any requests. Only completely restarting the node would fix this. More information on when
//the RECALLED state is supposed to be triggered during normal operation can be found here:
//http://wiki.ros.org/actionlib/DetailedDescription#Server_Description
//
//	I was able to determine that this "Total Recall" problem would manifest after successfully serving
//one request, but before the Execute callback function for the next was called- essentially, it was
//happening in the internals of the SimpleActionServer object were we could not really get access. Therefore,
//the server was restructured to include an explicit "goal" callback in a (successful) attempt to bypass
//whatever element of the simple action server was causing the issue. More information on this server
//architecture can be found here:
//http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28GoalCallbackMethod%29

//GLOBAL VARS
//Might want to move these into another pkg later on, as they are usesul in a more general case.
const std::string JOINT_NAMES[13] = {
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
//And of course, despite representing the same exact joints, the joint names
//that come out of the state message are DIFFERENT from the ones the
//controllers answer to...
const std::string FEEDBACK_NAMES[13] = {
	"outer_yaw_joint"			,
	"outer_pitch_joint"			,
	"outer_pitch_joint_1" 			,
	"outer_pitch_joint_2" 			,
	"outer_pitch_joint_3" 			,
	"outer_pitch_joint_4"			,
	"outer_pitch_joint_5" 			,
	"outer_insertion_joint"			,
	"outer_roll_joint"			,
	"outer_wrist_pitch_joint"		,
	"outer_wrist_yaw_joint"			,
	"outer_wrist_open_angle_joint"		,
	"outer_wrist_open_angle_joint_mimic"
	
};
const double DT_TRAJ = 0.01;

//Persistant things relating to the publishers and subscribers.
ros::Publisher g_publishers[2][13];
actionlib::SimpleActionServer<davinci_traj_streamer::trajAction> * g_server;
//The action server has a pointer stored globally because it's constructed in Main
//as everything is initialized, but the goal callback uses it and has no arguments.
sensor_msgs::JointState g_states;
bool g_fresh_pos;

//CALLBACKS
void CB_update(const sensor_msgs::JointState::ConstPtr& incoming);
void CB_goal();

//UTILITY FUNCS
void tl_execute(
	const davinci_traj_streamer::trajGoalConstPtr& goal,
	actionlib::SimpleActionServer<davinci_traj_streamer::trajAction> * server
);
bool exec_position(
	const trajectory_msgs::JointTrajectoryPoint & in_point,
	const trajectory_msgs::JointTrajectoryPoint & prev_point,
	double & elapsed_time
);
std::vector<std::vector<double> > expand_joint_list(const std::vector<double> & input);
std::vector<std::vector<double> > get_robot_pos();//Not currently used, kept for archival reasons until moved to another pkg.

int main(int argc, char **argv) {
	//Set up the node.
	ros::init(argc, argv, "traj_interpolator_both_as");
	ros::NodeHandle nh;
	
	//Set up the publishers.
	for(int i = 0; i < 13; i++){
		g_publishers[0][i] = nh.advertise<std_msgs::Float64>("/davinci/one_" + JOINT_NAMES[i] + "/command", 1, true); 
		g_publishers[1][i] = nh.advertise<std_msgs::Float64>("/davinci/two_" + JOINT_NAMES[i] + "/command", 1, true);
	}
	
	//Set up the subscriber
	ros::Subscriber robot_state_sub = nh.subscribe("/davinci/joint_states", 10, CB_update);
	ROS_INFO("NODE LINKS LATCHED");
	
	//Begin offering the service
	actionlib::SimpleActionServer<davinci_traj_streamer::trajAction> server(nh, "trajActionServer", false);
	g_server = & server;//Update that pointer for the goal callback.
	server.registerGoalCallback(&CB_goal);
	server.start();
	
	//Go into spin.
	while (ros::ok()){
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}	
	
	return 0;
}

//Accept goals manually to avoid the "Total Recall" bug
void CB_goal(){
	
	//I guess you could just run the whole trajectory in
	//here, but I've kept the execute function.
	tl_execute(g_server->acceptNewGoal(), g_server);
	
	//Could add logic that checks if already busy, etc.
	
	//g_server->isPreemptRequested();
	//g_server->isActive();
	//g_server->isNewGoalAvailable();
}

void CB_update(const sensor_msgs::JointState::ConstPtr& incoming){
	g_fresh_pos = true;
	g_states = *incoming;
	return;
}

//The bulk of the trajectory execution is still done in what used to be the exec callback.
void tl_execute(
	const davinci_traj_streamer::trajGoalConstPtr& goal,
	actionlib::SimpleActionServer<davinci_traj_streamer::trajAction>* server
){
	ROS_INFO("Got a trajectory request with id %u, %lu positions.", goal->traj_id, goal->trajectory.points.size());
	//Start the timer.
	double elapsed_time = 0.0;
	//Kick us out early if the trajectory does not have at least a start point and an end point.
	if(goal->trajectory.points.size() < 2){
		ROS_ERROR("Trajectory %u is too small! Aborting.", goal->traj_id);
		davinci_traj_streamer::trajResult r;
		r.return_val = 0;
		r.traj_id = goal->traj_id;
		server->setAborted();
		return;
	}
	
	//Move to each point in sequence and update the timer,
	for(int i = 1; i < goal->trajectory.points.size(); i++){
		if(!exec_position(goal->trajectory.points[i], goal->trajectory.points[i-1], elapsed_time)){
			//Abort if we discover that one of the points we are going to is malformed.
			davinci_traj_streamer::trajResult r;
			r.return_val = 0;
			r.traj_id = goal->traj_id;
			server->setAborted();
			return;
		}
	}
	//Tell the requester we did a good job.
	ROS_INFO("Trajectory %u complete.", goal->traj_id);
	davinci_traj_streamer::trajResult r;
	r.return_val = 1;
	r.traj_id = goal->traj_id;
	server->setSucceeded(r);
	
	return;
}

//Move the robot to each individual markpoint.
bool exec_position(
	const trajectory_msgs::JointTrajectoryPoint& in_point,
	const trajectory_msgs::JointTrajectoryPoint& prev_point,
	double & elapsed_time
){
	//Give us at least one tick in which to execute the trajectory
	double input_gtime = max(in_point.time_from_start.toSec(), elapsed_time + DT_TRAJ);
	
	std::vector<double> input_jnts = in_point.positions;
	std::vector<double> current_jnts = prev_point.positions;
	
	//Run some checks...
	if(input_jnts.size() != 14){
		ROS_ERROR("DaVinci trajectory points should have 14 joints, and this one has %lu!", input_jnts.size());
		return false;
	}
	if(current_jnts.size() != 14){
		ROS_ERROR("DaVinci trajectory points should have 14 joints, and the last one has %lu!", current_jnts.size());
		return false;
	}
	
	ROS_INFO("	Go to position %f, %f, %f, %f, %f, %f, %f and %f, %f, %f, %f, %f, %f, %f by %f at %f",
		input_jnts[0],
		input_jnts[1],
		input_jnts[2],
		input_jnts[3],
		input_jnts[4],
		input_jnts[5],
		input_jnts[6],
		input_jnts[7],
		input_jnts[8],
		input_jnts[9],
		input_jnts[10],
		input_jnts[11],
		input_jnts[12],
		input_jnts[13],
		input_gtime,
		elapsed_time
	);
	
	//Convert from our seven degrees of freedom to DaVinci's 13 actual joints
	std::vector<std::vector<double> > start_configuration = expand_joint_list(current_jnts);
	std::vector<std::vector<double> > end_configuration = expand_joint_list(input_jnts);
	
	double old_elapsed_time = elapsed_time;
	while(elapsed_time < input_gtime){//Each tick...
		ros::Duration wait_time(DT_TRAJ);
		elapsed_time = elapsed_time + DT_TRAJ;
		std_msgs::Float64 messages[2][13];
		double parametric_value = (elapsed_time - old_elapsed_time) / (input_gtime - old_elapsed_time);//Normalize our time.
		for(int i = 0; i < 13; i++){
			//Get new positions by treating our path as a parametric equation of time.
			//And stuff that into the messages.
			messages[0][i].data = start_configuration[0][i] + (end_configuration[0][i] - start_configuration[0][i]) * parametric_value;
			messages[1][i].data = start_configuration[1][i] + (end_configuration[1][i] - start_configuration[1][i]) * parametric_value;
		}
		//Publish the messages
		for(int i = 0; i < 2; i++){
			for(int j = 0; j < 13; j++){
				g_publishers[i][j].publish(messages[i][j]);
			}
		}
		//Wait until the next tick.
		ros::spinOnce();
		wait_time.sleep();
	}
		
	return true;
}

//Currently not used, but kept because it may be useful elsewhere.
std::vector<std::vector<double> > get_robot_pos(){
	g_fresh_pos = false;
	while(!g_fresh_pos){
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	std::vector<std::vector<double> > rp;
	rp.resize(2);
	rp[0].resize(13);
	rp[1].resize(13);
	
	//Read the robopositions
	for(int i = 0; i < 13; i++){
		Davinci_fwd_solver::get_jnt_val_by_name("one_" + FEEDBACK_NAMES[i], g_states, rp[0][i]);
		Davinci_fwd_solver::get_jnt_val_by_name("two_" + FEEDBACK_NAMES[i], g_states, rp[1][i]);
	}
	
	/*ROS_INFO("Arm two is at: ");
	for(int i  = 0; i < 13; i++){
		ROS_INFO("	%f", rp[1][i]);
	}*/
	
	return rp;
}

//Convert the seven degrees of freedom the trajectories come in into the 13 joints Da Vinci-gazebo actually controls (once for each hand).
std::vector<std::vector<double> > expand_joint_list(const std::vector<double> & input){
	std::vector<std::vector<double> > njl;
	njl.resize(2);
	njl[0].resize(13);
	njl[1].resize(13);
	
	for(int i = 0; i < 2; i++){
		int offset = 7 * i;
		njl[i][0] = input[0 + offset];//joint1_position_controller
 
		njl[i][1] = input[1 + offset];//joint2_position_controller
		njl[i][2] = input[1 + offset];//joint2_1_position_controller
		njl[i][3] = input[1 + offset];//joint2_2_position_controller
		njl[i][6] = input[1 + offset];//joint2_5_position_controller 

		njl[i][4] = -input[1 + offset];//joint2_3_position_controller 
		njl[i][5] = -input[1 + offset];//joint2_4_position_controller
		
		njl[i][7] = input[2 + offset];//joint3_position_controller

		njl[i][8] = input[3 + offset];//joint4_position_controller

		njl[i][9] = input[4 + offset];//joint5_position_controller

		njl[i][10] = input[5 + offset];//joint6_position_controller

		njl[i][11] = input[6 + offset];//joint7_position_controller
		njl[i][12] = -input[6 + offset];//joint7_position_controller_mimic
	}
    
	return njl;
}

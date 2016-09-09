#include <davinci_interface/davinci_interface.h>

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

static bool publisher_ready = false;
static bool subscriber_ready = false;

static bool fresh_pos;
static sensor_msgs::JointState states;
static ros::Subscriber robot_state_sub;

static ros::Publisher joint_publishers[2][13];

//May want to expand this later on, give it functions for single arms, etc.
std::vector<std::vector<double> > expand_joint_list(const double input[14]);

//Callbacks
void CB_update(const sensor_msgs::JointState::ConstPtr& incoming);

void davinci_interface::init_joint_control(ros::NodeHandle & nh){
	if(!publisher_ready){
		//Set up the publishers.
		for(int i = 0; i < 13; i++){
			joint_publishers[0][i] = nh.advertise<std_msgs::Float64>("/dvrk/two_" + OUTPUT_JOINT_NAMES[i] + "/command", 1, true); 
			joint_publishers[1][i] = nh.advertise<std_msgs::Float64>("/dvrk/one_" + OUTPUT_JOINT_NAMES[i] + "/command", 1, true);
		}
		publisher_ready = true;
	}
}

bool davinci_interface::publish_joints(const double input[14]){
	if(!publisher_ready){
		return false;
	}
	std::vector<std::vector<double> > expanded_joints = expand_joint_list(input);
	std_msgs::Float64 messages[2][13];
	for(int i = 0; i < 2; i++){
		for(int j = 0; j < 13; j++){
			messages[i][j].data = expanded_joints[i][j];
			joint_publishers[i][j].publish(messages[i][j]);
		}
	}
	return true;
}

void davinci_interface::init_joint_feedback(ros::NodeHandle & nh){
	if(!subscriber_ready){
		ros::Subscriber robot_state_sub = nh.subscribe("/dvrk/joint_states", 10, CB_update);
		subscriber_ready = true;
		fresh_pos = false;
	}
}

bool davinci_interface::get_fresh_robot_pos(std::vector<std::vector<double> > & output){
	if(!subscriber_ready){
		return false;
	}
	
	fresh_pos = false;
	while(!fresh_pos){
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	output.clear();
	output.resize(2);
	output[0].resize(13);
	output[1].resize(13);
	
	//Read the robopositions
	for(int i = 0; i < 13; i++){
		Davinci_fwd_solver::get_jnt_val_by_name("two_" + INPUT_JOINT_NAMES[i], states, output[0][i]);
		Davinci_fwd_solver::get_jnt_val_by_name("one_" + INPUT_JOINT_NAMES[i], states, output[1][i]);
	}
	
	/*ROS_INFO("Arm one is at: ");
	for(int i  = 0; i < 13; i++){
		ROS_INFO("	%f", rp[1][i]);
	}*/
	
	return true;
}

bool davinci_interface::get_last_robot_pos(std::vector<std::vector<double> > & output){
	if(!subscriber_ready || !fresh_pos){
		return false;
	}
	
	output.clear();
	output.resize(2);
	output[0].resize(13);
	output[1].resize(13);
	
	//Read the robopositions
	for(int i = 0; i < 13; i++){
		Davinci_fwd_solver::get_jnt_val_by_name("two_" + INPUT_JOINT_NAMES[i], states, output[0][i]);
		Davinci_fwd_solver::get_jnt_val_by_name("one_" + INPUT_JOINT_NAMES[i], states, output[1][i]);
	}
	
	fresh_pos = false;
	
	return true;
}

void CB_update(const sensor_msgs::JointState::ConstPtr& incoming){
	fresh_pos = true;
	states = *incoming;
	return;
}

std::vector<std::vector<double> > expand_joint_list(const double input[14]){
	std::vector<std::vector<double> > njl;
	njl.resize(2);
	njl[0].resize(13);
	njl[1].resize(13);
	
	for(int i = 0; i < 2; i++){
		int offset = i * 7;
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

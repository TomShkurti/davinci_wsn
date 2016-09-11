#include <davinci_interface/davinci_interface.h>

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

static bool fresh_pos_1;
static bool fresh_pos_2;
static sensor_msgs::JointState states_1;
static sensor_msgs::JointState states_2;
static ros::Subscriber robot_state_sub_1;
static ros::Subscriber robot_state_sub_2;

static ros::Publisher joint_publisher_1;
static ros::Publisher joint_publisher_2;

//Callbacks
void CB_update_1(const sensor_msgs::JointState::ConstPtr& incoming);
void CB_update_2(const sensor_msgs::JointState::ConstPtr& incoming);

void davinci_interface::init_joint_control(ros::NodeHandle & nh){
	if(!publisher_ready){
		//Set up the publishers.
		joint_publisher_1 = nh.advertise<sensor_msgs::JointState>("/dvrk/PSM1/position_joint_desired", 1, true);
		joint_publisher_2 = nh.advertise<sensor_msgs::JointState>("/dvrk/PSM2/position_joint_desired", 1, true);
		publisher_ready = true;
	}
}

bool davinci_interface::publish_joints(const double input[14]){
	if(!publisher_ready){
		return false;
	}
	
	sensor_msgs::JointState psm_1_msg;
	sensor_msgs::JointState psm_2_msg;
	for(int i = 0; i < 7; i++){
		psm_1_msg.position.push_back(input[i + 7]);
		psm_2_msg.position.push_back(input[i]);
	}
	
	joint_publisher_1.publish(psm_1_msg);
	joint_publisher_2.publish(psm_2_msg);
	
	return true;
}

void davinci_interface::init_joint_feedback(ros::NodeHandle & nh){
	if(!subscriber_ready){
		ros::Subscriber robot_state_sub_1 = nh.subscribe("/dvrk/PSM1/joint_states", 10, CB_update_1);
		ros::Subscriber robot_state_sub_2 = nh.subscribe("/dvrk/PSM2/joint_states", 10, CB_update_2);
		subscriber_ready = true;
		fresh_pos_1 = false;
		fresh_pos_2 = false;
	}
}

bool davinci_interface::get_fresh_robot_pos(std::vector<std::vector<double> > & output){
	if(!subscriber_ready){
		return false;
	}
	
	fresh_pos_1 = false;
	fresh_pos_2 = false;
	while(!fresh_pos_1 && fresh_pos_2){
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	output.clear();
	output.resize(2);
	output[0].resize(13);
	output[1].resize(13);
	
	//Read the robopositions
	for(int i = 0; i < 13; i++){
		Davinci_fwd_solver::get_jnt_val_by_name(INPUT_JOINT_NAMES[i], states_1, output[0][i]);
		Davinci_fwd_solver::get_jnt_val_by_name(INPUT_JOINT_NAMES[i], states_2, output[1][i]);
	}
	
	/*ROS_INFO("Arm one is at: ");
	for(int i  = 0; i < 13; i++){
		ROS_INFO("	%f", rp[1][i]);
	}*/
	
	return true;
}

bool davinci_interface::get_last_robot_pos(std::vector<std::vector<double> > & output){
	if(!subscriber_ready || !(fresh_pos_1 && fresh_pos_2)){
		return false;
	}
	
	output.clear();
	output.resize(2);
	output[0].resize(13);
	output[1].resize(13);
	
	//Read the robopositions
	for(int i = 0; i < 13; i++){
		Davinci_fwd_solver::get_jnt_val_by_name(INPUT_JOINT_NAMES[i], states_1, output[0][i]);
		Davinci_fwd_solver::get_jnt_val_by_name(INPUT_JOINT_NAMES[i], states_2, output[1][i]);
	}
	
	fresh_pos_1 = false;
	fresh_pos_1 = false;
	
	return true;
}

void CB_update_1(const sensor_msgs::JointState::ConstPtr& incoming){
	fresh_pos_1 = true;
	states_1 = *incoming;
	return;
}

void CB_update_2(const sensor_msgs::JointState::ConstPtr& incoming){
	fresh_pos_2 = true;
	states_2 = *incoming;
	return;
}

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <davinci_kinematics/davinci_kinematics.h>

//Takes in a status update in the simulator format
//(a singe fourteen-unit-long sensor_msgs/JointState message, containing both PSMs)
//and reformats it to work with the individual controller topics you would get from the real DaVinci.
//Don't mess with this unless you are changing something in the simulator itself.

const std::string INPUT_JOINT_NAMES[7] = {
	"outer_yaw"			,
	"outer_pitch"			,
	"outer_insertion"		,
	"outer_roll"			,
	"outer_wrist_pitch"		,
	"outer_wrist_yaw"		,
	"jaw"	
};

ros::Publisher state_publisher_p1;
ros::Publisher state_publisher_p2;

void CB_input_split(const sensor_msgs::JointState::ConstPtr& msg){
	sensor_msgs::JointState msg_out_1;
	sensor_msgs::JointState msg_out_2;
	
	msg_out_1.header = msg->header;
	msg_out_2.header = msg->header;
	
	double d;
	
	for(int i = 0; i < 7; i++){
		msg_out_1.name.push_back(INPUT_JOINT_NAMES[i]);
		msg_out_2.name.push_back(INPUT_JOINT_NAMES[i]);
		
		Davinci_fwd_solver::get_jnt_val_by_name("one_" + INPUT_JOINT_NAMES[i], *msg, d);
		msg_out_1.position.push_back(d);
		Davinci_fwd_solver::get_jnt_val_by_name("two_" + INPUT_JOINT_NAMES[i], *msg, d);
		msg_out_2.position.push_back(d);
	}
	
	state_publisher_p1.publish(msg_out_1);
	state_publisher_p2.publish(msg_out_2);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "adapter_input");
	ros::NodeHandle nh;
	
	state_publisher_p1 = nh.advertise<sensor_msgs::JointState>("/dvrk/PSM1/joint_states", 1, true);
	state_publisher_p2 = nh.advertise<sensor_msgs::JointState>("/dvrk/PSM2/joint_states", 1, true);
	
	ros::Subscriber sub = nh.subscribe("/davinci/joint_states", 1, CB_input_split);
	
	ros::spin();

	return 0;
}

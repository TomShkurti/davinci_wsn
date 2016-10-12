#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

//Takes in a joint command in the Da Vinci format
//(an unlabeled, seven-unit-long sensor_msgs/JointState message, one for each PSM)
//and reformats it to work with the individual controller topics the simulator understands.
//Don't mess with this unless you are changing something in the simulator itself.

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

//The simulation-facing publishers
ros::Publisher joint_publishers_p1 [13];
ros::Publisher joint_publishers_p2 [13];

//The callbacks to recieve davinci-like commands (i.e. outward-facing).
void CB_psm1(const sensor_msgs::JointState::ConstPtr& msg){

	std::vector<double> input = msg -> position;
	
	//Go ahead and expand these to all the 'mimic' joints.
	std_msgs::Float64 messages[13];
	
	messages[0].data = input[0];//joint1_position_controller
 
	messages[1].data = input[1];//joint2_position_controller
	messages[2].data = input[1];//joint2_1_position_controller
	messages[3].data = input[1];//joint2_2_position_controller
	messages[6].data = input[1];//joint2_5_position_controller 
	
	messages[4].data = -input[1];//joint2_3_position_controller 
	messages[5].data = -input[1];//joint2_4_position_controller
		
	messages[7].data = input[2];//joint3_position_controller

	messages[8].data = input[3];//joint4_position_controller

	messages[9].data = input[4];//joint5_position_controller

	messages[10].data = input[5];//joint6_position_controller

	messages[11].data = input[6];//joint7_position_controller
	messages[12].data = -input[6];//joint7_position_controller_mimic
	
	for(int i = 0; i < 13; i++){
		joint_publishers_p1[i].publish(messages[i]);
	}
}

//Second verse, same as the first.
void CB_psm2(const sensor_msgs::JointState::ConstPtr& msg){

	std::vector<double> input = msg -> position;
	
	std_msgs::Float64 messages[13];
	
	messages[0].data = input[0];//joint1_position_controller
 
	messages[1].data = input[1];//joint2_position_controller
	messages[2].data = input[1];//joint2_1_position_controller
	messages[3].data = input[1];//joint2_2_position_controller
	messages[6].data = input[1];//joint2_5_position_controller 
	
	messages[4].data = -input[1];//joint2_3_position_controller 
	messages[5].data = -input[1];//joint2_4_position_controller
		
	messages[7].data = input[2];//joint3_position_controller

	messages[8].data = input[3];//joint4_position_controller

	messages[9].data = input[4];//joint5_position_controller

	messages[10].data = input[5];//joint6_position_controller

	messages[11].data = input[6];//joint7_position_controller
	messages[12].data = -input[6];//joint7_position_controller_mimic
	
	for(int i = 0; i < 13; i++){
		joint_publishers_p2[i].publish(messages[i]);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "adapter_output");
	ros::NodeHandle nh;
	
	//Initialize the simulator-facing publishers
	for(int i = 0; i < 13; i++){
		joint_publishers_p1[i] = nh.advertise<std_msgs::Float64>("/davinci/one_" + OUTPUT_JOINT_NAMES[i] + "/command", 1, true);
		joint_publishers_p2[i] = nh.advertise<std_msgs::Float64>("/davinci/two_" + OUTPUT_JOINT_NAMES[i] + "/command", 1, true);
	}
	
	//Initialize the user-facing subscribers
	ros::Subscriber sub_1 = nh.subscribe("/dvrk/PSM1/set_position_joint", 1, CB_psm1);
	ros::Subscriber sub_2 = nh.subscribe("/dvrk/PSM2/set_position_joint", 1, CB_psm2);
	
	//Testing in the field, ros::spin() does not seem to have the problem that ros::spinOnce() in a while loop does
	//where it sucked up massive amounts of CPU in order to do nothing extremely quickly.
	ros::spin();

	return 0;
}

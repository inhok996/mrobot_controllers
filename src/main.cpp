#include "ros/ros.h"
#include <iostream>
#include <mrobot_controllers/supervisor/supervisor.h>
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "mrobot_controllers/srvController.h"

#define STOP_CONTROL 0
#define RUN_CONTROL 1
#define RESET_PARAM 2

sensor_msgs::JointState js;
pcl::PointCloud<pcl::PointXYZI> pc;
int global_state = STOP_CONTROL;
int current_control_num = STOP;
bool if_con_changed = false;
int max_con_num = 0; //max

void jointCallback(const sensor_msgs::JointState& msg) { js.position = msg.position; }
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) 
{
	if(msg->header.frame_id != "laser") ROS_ERROR("HLDS DRIVER SHOULD WORK!!");
	else pcl::fromROSMsg(*msg,pc);
}
bool exec_command(mrobot_controllers::srvController::Request& req,
	mrobot_controllers::srvController::Response& res);
bool init_params(ros::NodeHandle& nh,mrobot_control::Supervisor& sv);

using namespace mrobot_control;
int main(int argc,char **argv)
{
	ros::init(argc, argv, "model_control");
	ros::NodeHandle n;

	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000); 
	ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_out",1000);
	ros::Subscriber state_reader = n.subscribe("/joint_states",1000,jointCallback); 
	ros::Subscriber point_reader = n.subscribe("/cloud_points",1000,cloudCallback);
	ros::ServiceServer control_srv = n.advertiseService("/msrv",exec_command);

	ros::Rate loop_rate(10); //loop_rate 10 hz
	Supervisor* sv = new Supervisor(); //supervisor constructor
	if(!init_params(n,*sv)){ std::cout << "parameter error" << std::endl; }
	geometry_msgs::Twist twist_cmd;

	twist_cmd.linear.x = twist_cmd.linear.y = twist_cmd.linear.z = 0; //twist initialization
	twist_cmd.angular.x = twist_cmd.angular.y = twist_cmd.angular.z = 0;

	//JointStates class has vector class(need to make memory location)
	js.position.push_back(0.0);
	

	printf("======================================\n");
	printf("Lab6 mobile Robot Controller program\n");
	printf("======================================\n");
	
	while(ros::ok())
	{
		ros::spinOnce(); //call all callback functions
		if(global_state == RUN_CONTROL){
			if(if_con_changed){	
				sv->set_cont_idx(current_control_num);
			}
			sv->get_sensor_data(js.position[0],js.position[1], pc); //save sensor data into supervisor
			current_control_num = sv->execute(); //execute controllers

			twist_cmd.linear.x = sv->get_out_v();//save output v,w before publish
			twist_cmd.angular.z = sv->get_out_w();

			cloud_pub.publish(sv->get_cloud()); //pharsed points Publish for debug
			twist_pub.publish(twist_cmd); //twist command publish
		}else if(global_state == RESET_PARAM){
			sv->reset(); //reset all parameters
			init_params(n,*sv);
			global_state = STOP_CONTROL;
			ROS_INFO("reset end. global_state now STOP_CONTROL\n");
		}else if(global_state == STOP_CONTROL);
		loop_rate.sleep();
	}
	ROS_INFO("program killed\n");
	return 0;
}

bool init_params(ros::NodeHandle& nh,mrobot_control::Supervisor& sv)
{
	int numCon; //number of Controllers
	if(!nh.getParam("NumControllers",numCon)){
		ROS_INFO("param name NumControllers needed");
		return false;
	}
	max_con_num = numCon;
	params pa[MAX_NUM_CONTROLLERS];
	std::string c_key("Controller0"); //controller key
	std::string key;
	for(int i = 1; i <= numCon ; i++){
		c_key[c_key.find((i + '0') - 1)] = i + '0';
		if(nh.searchParam(c_key,key)){
			std::string c_name;
			if(!nh.getParam(key+"/controller_name",c_name)) return false;
			if(!nh.getParam(key+"/kp",pa[i-1].kp)) return false;
			if(!nh.getParam(key+"/ki",pa[i-1].ki)) return false;
			if(!nh.getParam(key+"/kd",pa[i-1].kd)) return false;
			if(!nh.getParam(key+"/numInputs",pa[i-1].argc)) return false; //essential params til this line
			if(pa[i-1].argc > MAX_NUM_INPUT) { ROS_INFO("too many inputs(max : %d)",MAX_NUM_INPUT); return false;}
			std::string argvin("/input");
			for(int j = 1; j <= pa[i-1].argc; j++){
				pa[i-1].argv[j-1] = 0; //if failed, it has default value as 0
				nh.getParam(key+(argvin + boost::to_string(j)),pa[i-1].argv[j-1]);
			}
			//print here
			std::cout <<"controller name :  " << c_name << std::endl;
			std::cout <<"kp : " << pa[i-1].kp << std::endl;
			std::cout <<"ki : " << pa[i-1].ki << std::endl;
			std::cout <<"kd : " << pa[i-1].kd << std::endl;
			std::cout <<"numInputs : " << pa[i-1].argc << std::endl;
			for(int j = 1 ; j <= pa[i-1].argc ; j++)
				std::cout <<"input" << j <<" : " << pa[i-1].argv[j-1] << std::endl;
		}
	}
	sv.setParams(pa,numCon);
	ROS_INFO("param set end\n");
	return true;
}

bool exec_command(mrobot_controllers::srvController::Request& req,
	mrobot_controllers::srvController::Response& res)
{
	int cmd = -1;
	if(req.command =="stop") cmd = STOP_CONTROL;
	else if(req.command =="run") cmd = RUN_CONTROL;
	else if(req.command =="reset") cmd = RESET_PARAM;

	if(cmd == STOP_CONTROL || cmd == RUN_CONTROL || cmd == RESET_PARAM){
		if(global_state == RUN_CONTROL && cmd == RESET_PARAM){
			res.command_result = -1; //invalid command
		}else if(global_state == RESET_PARAM && cmd == RUN_CONTROL){
			res.command_result = -1; //invalid command
		}else{
			res.command_result = global_state = cmd;
			if(req.controller >= 0 && req.controller <= max_con_num)
				current_control_num = req.controller;
			else
				res.command_result = -1;
		}
	}else{
		res.command_result = -1; //invalid command
	}
	ROS_INFO("current global_state = %d (0 : STOP_CONTROL, 1: RUN_CONTROL, 2:RESET_PARAM)",global_state);
	if(res.command_result == RUN_CONTROL){
		if_con_changed = true;
		ROS_INFO("requested controller = %d (0 : STOP, 1: GTA 2: GTG)",current_control_num);
	}
	return true;
}

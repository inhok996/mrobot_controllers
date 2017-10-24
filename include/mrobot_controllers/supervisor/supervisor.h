#pragma once
#include <cmath>
#include <stdio.h>
#include <mrobot_controllers/controllers/controllers.h>
#include <mrobot_controllers/robot/hardwareinfo.h>
#include <mrobot_controllers/robot/sensors.h>
#define STOP 0
#define GOTOANGLE 1
#define GOTOGOAL 2
#define AVOIDOBSTACLE 3
#define AVONGTG 4
#define WALLFOLLOWING 5
namespace mrobot_control
{

	class Supervisor
	{
		public:
			Supervisor(void);
			void setParams(params in_param[],int size);
			void get_encoder_data(double pos_l,double pos_r); //return: if first
			int execute(); //return : current_state
			void reset();
			void update_odometry();
			void uni_to_diff(double out_v,double out_w);
			void first_position_sample(double i_pos_l,double i_pos_r);
			void set_cont_idx(int idx) { cont_idx = idx; }
			int get_cont_idx() {return cont_idx;}
			double get_vel_l(){return vel_l;} //for ev3_model_control
			double get_vel_r(){return vel_r;} //for ev3_model_control
			double get_out_v(){return out_v;} //for turtlebot3
			double get_out_w(){return out_w;} //for turtlebot3
			sensor_msgs::PointCloud2& get_cloud() {return hlds.get_cloud();}
		private:
			controller* controllers[10];
			int cont_idx; //current controller index
			int last_cont_idx; //last controller index
			odometry odm;// odometry object
			laser_sensor hlds; //laser data
			double out_v; //unicycle output
			double out_w; //unicycle output
			double vel_r; //velocity of right wheel, output of uni_to_diff function
			double vel_l; //velocity of left wheel, output of uni_to_diff function
			double pos_l; //position of left wheel(radius)
			double pos_r; //position of right wheel(radius)
			double prev_pos_l; //last saved position left
			double prev_pos_r; //last saved position right
			double robot_r; //robot wheel radius
			double robot_l; //robot base length
			long long seqno; //sequence number
			
	};
	
}

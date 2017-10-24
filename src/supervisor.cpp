#include <mrobot_controllers/supervisor/supervisor.h>

namespace mrobot_control
{
	Supervisor::Supervisor() : prev_pos_l(0),prev_pos_r(0),vel_l(0),vel_r(0),seqno(0)
	{
		printf("Supervisor init\n");
		cont_idx = STOP; //stop controller
		printf("cont_idx = %d\n",cont_idx);

		controllers[STOP] = new stop(0);
		controllers[GOTOANGLE] = new gotoangle(1);
		controllers[GOTOGOAL] = new gotogoal(2);

		robot_r = WHEEL_RADIUS; //include/mrobot_controllers/robot/hardwareinfo.h
		robot_l = WHEEL_BASE_LENGTH; //include/mrobot_controllers/robot/hardwareinfo.h
		odm.update(0,0,0);
	}

	void Supervisor::setParams(params in_param[],int size)
	{
		for(int i = 0 ; i < size ; i++)
			controllers[i]->setParam(in_param[i]);
	}

	void Supervisor::get_encoder_data(double pos_l,double pos_r)
	{
		if(seqno == 0) { //if first, it need to save jointState data to prev_pos
			first_position_sample(pos_l,pos_r);
		}
		this->pos_l = pos_l; 
		this->pos_r = pos_r;
		update_odometry(); //update robot's odometry here
		//hlds.pharse_cloud(inCloud); //save only usable point data
		++seqno; //sequence number
	}

	int Supervisor::execute()
	{
		//printf("Supervisor::execute called\n");
		//printf("current pose = %lf, %lf, %lf\n",odm.get_x(),odm.get_y(),odm.get_theta());
		int execute_event = 0;

		printf("prev_pos_l = %lf, prev_pos_r = %lf\n",prev_pos_l,prev_pos_r);

		if(controllers[cont_idx]->execute(odm)) cont_idx = STOP;//for gotoGoal

		this->out_v = controllers[cont_idx]->get_out_v(); //get out v, w according to state
		this->out_w = controllers[cont_idx]->get_out_w();

		//for ev3, not used for turtlebot
		uni_to_diff(this->out_v,this->out_w);// initialize vel_l,vel_r with respect to o_v, o_w
		return cont_idx;
	}

	void Supervisor::reset()
	{
		prev_pos_l = 0;
		prev_pos_r = 0;
		cont_idx = STOP;
		seqno = 0;
		vel_l = 0;
		vel_r = 0;
		odm.update(0,0,0);
	}
	void Supervisor::first_position_sample(double i_pos_l,double i_pos_r)
	{
		//this function is made for saving the first position values of wheels(it can be not 0.) 
		prev_pos_l = i_pos_l;
		prev_pos_r = i_pos_r;
		printf("save jointStates info to prev_pos\n");
		printf("prev_pos_l = %lf, prev_pos_r = %lf\n",prev_pos_l,prev_pos_r);

	}

	void Supervisor::update_odometry()
	{
		//printf("update_odometry called\n");
		double delta_right_rad, delta_left_rad, m_per_rad;
		double dr, dl, dc;
		double x_new, y_new, theta_new;
		double x, y, theta;
		double rad;

		rad = 180/M_PI;

		x = odm.get_x();
		y = odm.get_y();
		theta = odm.get_theta();

		//compute odometry here
		delta_right_rad = pos_r - prev_pos_r;
		delta_left_rad = pos_l - prev_pos_l;
		m_per_rad = (2*M_PI*robot_r)*(rad/360);

		dr = m_per_rad * delta_right_rad;
		dl = m_per_rad * delta_left_rad;
		dc = (dr+dl)/2;

		x_new = x + dc*cos(theta);
		y_new = y + dc*sin(theta);
		theta_new = theta + (dr - dl)/robot_l;

		//save the wheel encoder rad for the next estimate
		prev_pos_r = pos_r;
		prev_pos_l = pos_l;

		//update your estimate of (x,y,theta)
		odm.update(x_new,y_new,theta_new);

	}

	void Supervisor::uni_to_diff(double out_v,double out_w)
	{
		vel_r = (2*out_v + out_w*robot_l)/(2*robot_r);
		vel_l = (2*out_v - out_w*robot_l)/(2*robot_r);
		//if(seqno%10 ==0)
		//	printf("vel_l = %lf, vel_r = %lf\n",vel_l,vel_r);
	}

	void odometry::update(double x_in,double y_in, double theta_in)
	{
		x = x_in;
		y = y_in;
		theta = theta_in;
	}

}

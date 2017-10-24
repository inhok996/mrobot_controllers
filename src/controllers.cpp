#include <mrobot_controllers/controllers/controllers.h>
#include <math.h> //for atan2,sin,cos
#include <stdio.h> //for printf
namespace mrobot_control
{

	controller::controller(int type): type(type) {}
	controller::controller(){}
	controller::~controller(){}
	void controller::setParam(params in){}
	int controller::execute(odometry& odm,laser_sensor& ls){}
	void controller::reset(){}

	stop::stop(){}
	stop::stop(int type)
	{
		out.o_v = 0;
		out.o_w = 0;
	}
	void stop::setParam(params in) {}
	int stop::execute(odometry& odm,laser_sensor& ls) {}
	void stop::reset() {}

	gotoangle::gotoangle(int type) : kp(0)
	{
		in.theta_g = 0;
		in.v_g = 0;
		out.o_v = 0;
		out.o_w = 0;
	}

	void gotoangle::setParam(params in)
	{
		printf("setParam called\n");
		kp = in.kp;
		this->in.v_g = in.argv[0];
		this->in.theta_g = in.argv[1];
		this->in.theta_g = this->in.theta_g * (M_PI/180); //degree to radian
	}

	int gotoangle::execute(odometry& odm, laser_sensor& ls)
	{
		//printf("gotoangle::execute called\n");
		double e_k = in.theta_g - odm.get_theta(); //calculate error
		//printf("&e_k = %p\n",&e_k);
		e_k = atan2(sin(e_k),cos(e_k)); //moderate value from -pi to pi

		double w = kp * e_k;// calculate omega

		out.o_v = in.v_g;
		out.o_w = w;
		printf("out.o_v = %lf, out.o_w = %lf\n",out.o_v,out.o_w);
		return 0;
	}

	void gotoangle::reset()
	{
		kp = 0;
		in.theta_g = 0;
		in.v_g = 0;
	}

	gotogoal::gotogoal(int type) : kp(0),ki(0),kd(0),E_k(0),e_k_1(0)
	{
		in.x_g = 0;
		in.y_g = 0;
		in.v_g = 0;
	}

	void gotogoal::setParam(params in)
	{
		//printf("gotogoal setParam called\n");
		kp = in.kp;
		ki = in.ki;
		kd = in.kd;
		this->in.v_g = in.argv[0];
		this->in.x_g = in.argv[1];
		this->in.y_g = in.argv[2];
		this->d_stop = in.argv[3];
		//printf(" kp %lf ki %lf kd %lf, %lf,%lf,%lf,\n",kp,ki,kd,in.v_g,in.x_g,in.y_g); 
	}

	int gotogoal::execute(odometry& odm,laser_sensor& ls)
	{
		//printf("gotoangle::execute called\n");
		double u_x, u_y; //reference x , y
		double theta_g; //theta goal (theta between goal and ev3)
		double e_k,e_P,e_I,e_D; //PID errors
		double w; //omega

		u_x = in.x_g - odm.get_x();
		u_y = in.y_g - odm.get_y();
		theta_g = atan2(u_y,u_x);
		e_k = theta_g - odm.get_theta();
		e_k = atan2(sin(e_k),cos(e_k));
		//printf("e_k = %lf\n",e_k);
		e_P = e_k;
		e_I = E_k + e_k*0.1; //dt
		e_D = (e_k - e_k_1)/0.1; //dt

		w = kp*e_P + ki*e_I + kd*e_D;

		E_k = e_I;
		e_k_1 = e_k;

		out.o_v = in.v_g;
		out.o_w = w;
		//printf("u_y , u_x , theta_g = %lf, %lf, %lf\n",u_y,u_x,theta_g);
		printf("out.o_v = %lf, out.o_w = %lf\n",out.o_v,out.o_w);
		if(check_event(odm)) return 1;
		return 0;
	}

	void gotogoal::reset()
	{
		kp = ki = kd = 0;
		d_stop = E_k = e_k_1 = 0;
		in.x_g = 0;
		in.y_g = 0;
		in.v_g = 0;
	}

	bool gotogoal::check_event(odometry& odm)
	{
		bool rc = false;
		double x, y, x_g, y_g, e; //current x,y goal x,y error
		x = odm.get_x();
		y = odm.get_y();
		x_g = in.x_g;
		y_g = in.y_g;
		e = d_stop;
		//printf("x = %lf , y = %lf, x_g = %lf, y_g = %lf, e = %lf\n",x,y,x_g,y_g,e);
		if(( x >= x_g - e) && (x <= x_g + e) && (y >= y_g - e) && (y <= y_g + e))//check if arrive
			rc = true;
		return rc;
	}

	avoidobstacles::avoidobstacles(int type) : kp(0),ki(0),kd(0),E_k(0),e_k_1(0)
	{
		in.v_g = 0;
	}

	void avoidobstacles::setParam(params in)
	{
		//printf("avoidobstacles setParam called\n");
		kp = in.kp;
		ki = in.ki;
		kd = in.kd;
		this->in.v_g = in.argv[0];
	}

	int avoidobstacles::execute(odometry& odm,laser_sensor& ls)
	{
		//printf("avoidobstacles::execute called\n");
		double theta_g; //theta goal (theta between goal and ev3)
		double e_k,e_P,e_I,e_D; //PID errors
		double w; //omega
		float sensor_gains[5] = {1, 1, 0.5 ,1, 1};
		/*

		tf matrix(x,y,@) = 
		| cos@ -sin@ x |
		| sin@  cos@ y |
		|  0     0   1 |

		*/
		//first get sensor data into this object
		get_ir_points(ls);
		//second transform from sensor frame to robot frame
		get_tf_matrix(-0.1,0,0); //x = -0.1 , y = 0 , theta = 0
		transform_ir_points();

		//third, transform from robot frame to world frame
		get_tf_matrix(odm.get_x(),odm.get_y(),odm.get_theta());
		transform_ir_points();
		//Compute heading vector for obstacle avoidance
		
		//goto angle pid
		//end
		



		//double e_k = in.theta_g - odm.get_theta(); //calculate error
		//printf("&e_k = %p\n",&e_k);
		//e_k = atan2(sin(e_k),cos(e_k)); //moderate value from -pi to pi

		//double w = kp * e_k;// calculate omega

		//out.o_v = in.v_g;
		//out.o_w = w;
		//printf("out.o_v = %lf, out.o_w = %lf\n",out.o_v,out.o_w);
		return 0;
	}

	void avoidobstacles::reset()
	{
		kp = ki = kd = 0;
		E_k = e_k_1 = 0;
		in.v_g = 0;
	}

	void avoidobstacles::get_ir_points(laser_sensor& ls)
	{
		double max_dist_points[NUM_IR_PTS][2] = {{0,0}, {0,0.7},{0.5,0.5},{0.7,0},{0.5,-0.5},{0,-0.7},{0,0}};
		for(int i = 0 ; i < NUM_IR_PTS; i++){
			ls.ir_points[i].x = this->ir_points[i].x;
			ls.ir_points[i].y = this->ir_points[i].y;
			ir_points[i].dist = sqrt((double)ir_points[i].x * ir_points[i].x + ir_points[i].y * ir_points[i].y);
			if(ir_points[i].dist >= 0.7 || ir_points[i].dist == 0){ //too far or not found
				ir_points[i].dist = 0.7;
				ir_points[i].x = max_dist_points[i][0];
				ir_points[i].y = max_dist_points[i][1];
			}
		}
	}

	void avoidobstacles::get_tf_matrix(float x, float y, float theta)
	{
		//important!! theta should be radian value not degree
		tf.cos_theta = cos(theta);
		tf.sin_theta = sin(theta);
		tf.x = x; 
		tf.y = y;
	}

	void avoidobstacles::transform_ir_points()
	{
		for(int i = 0 ; i < NUM_IR_PTS; i++){
			ir_points[i].x = ir_points[i].x * tf.cos_theta - ir_points[i].y * tf.sin_theta + tf.x;
			ir_points[i].y = ir_points[i].x * tf.sin_theta + ir_points[i].y * tf.cos_theta + tf.y;
		}
	}



}

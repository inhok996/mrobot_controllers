#pragma once
#include <mrobot_controllers/robot/odometry.h>
#define MAX_NUM_INPUT 99
#define MAX_NUM_CONTROLLERS 9
namespace mrobot_control
{
	struct params{
		double kp;
		double ki;
		double kd;
		int argc;
		double argv[MAX_NUM_INPUT];
	};

	class controller
	{
		public:
			controller();
			controller(int type);
			virtual void setParam(params in);
			virtual int execute(odometry odm);
			virtual void reset();
			double get_out_v(){return out.o_v;}
			double get_out_w(){return out.o_w;}
			~controller();
			int type;
		protected:
			struct output{
				double o_v;
				double o_w;
			}out;
	};
	
	class stop : public controller
	{
		public:
			stop();
			stop(int type);
			void setParam(params in);
			int execute(odometry odm);
			void reset();
			//no input for stop controller
	};

	class gotoangle : public controller
	{
		public:
			gotoangle();
			gotoangle(int type);
			void setParam(params in);
			int execute(odometry odm);
			void reset();
		private:
			double kp; //p controller kp value
			struct ref_input{  //reference
				double theta_g; //input theta_d
				double v_g; //input velocity
			};
			ref_input in;
	};


	class gotogoal : public controller
	{
		public:
			gotogoal();
			gotogoal(int type);
			void setParam(params in);
			int execute(odometry odm);
			void reset();
			bool check_event(odometry odm);
		private:
			double kp; //p controller kp value
			double ki;
			double kd;
			double d_stop; //stop trigger sqaure

			double E_k; //sum of all errors(integral)
			double e_k_1; //previous error
			struct ref_input{ //reference
				double x_g; //goal x coordinate
				double y_g; //goal y coordinate
				double v_g; //input velocity
			};
			ref_input in;
	};

}

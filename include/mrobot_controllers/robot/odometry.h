#pragma once

namespace mrobot_control
{
	class odometry
	{
		public:
			odometry():x(0),y(0),theta(0){}
			~odometry(){}
			double get_x() {return x;}
			double get_y(){return y;}
			double get_theta(){return theta;}
			void update(double x_in,double y_in, double theta_in);
		private:
			double x;
			double y;
			double theta;
	};
}

#pragma once
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#define L_RES 0.02 //line resolution
#define NUM_IR_PTS 7 //number of ir points
namespace mrobot_control
{
	class laser_sensor
	{
		public:
			laser_sensor();
			~laser_sensor();
			void parse_cloud(pcl::PointCloud<pcl::PointXYZI>& inCloud);
			sensor_msgs::PointCloud2& get_cloud() {return outCloud;}
		private:
			pcl::PointCloud<pcl::PointXYZI> phCloud; //Pharsed Cloud
			sensor_msgs::PointCloud2 outCloud; //for checking
			pcl::PointXYZI ir_points[NUM_IR_PTS]; //7 points
	};
}

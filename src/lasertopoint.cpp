#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include <iostream>

int cnt = 0;
sensor_msgs::PointCloud2 point_data;
laser_geometry::LaserProjection projector_;

void cloudCallback(const sensor_msgs::LaserScan::ConstPtr& input)
{
	tf::TransformListener tfListener_;
	projector_.transformLaserScanToPointCloud("laser",*input,point_data,tfListener_);

}
int main(int argc,char **argv)
{
	ros::init(argc, argv, "scan_pharser");
	ros::NodeHandle nh;
	ros::Subscriber my_sub = nh.subscribe("/scan",1,cloudCallback);
	ros::Publisher my_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_points",100);
	ros::Rate loop_rate(5);

	while(ros::ok())
	{
		ros::spinOnce();
		my_pub.publish(point_data);
		loop_rate.sleep();	
	}

	return 0;
}

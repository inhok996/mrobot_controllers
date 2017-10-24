#include <mrobot_controllers/robot/sensors.h>
#include <stdio.h>

namespace mrobot_control
{
	laser_sensor::laser_sensor(void){}
	laser_sensor::~laser_sensor(void){}
	void laser_sensor::parse_cloud(pcl::PointCloud<pcl::PointXYZI>& inCloud)
	{
		phCloud.clear(); //clear phCloud
		for(int i = 0 ; i < NUM_IR_PTS; i++){
			ir_points[i].x = 0; ir_points[i].y = 0; ir_points[i].z = 0; ir_points[i].intensity = 0;
		}
		//printf("phCloud.points.size() = %d\n",phCloud.points.size());
		phCloud.header = inCloud.header;
		pcl::PointCloud<pcl::PointXYZI>::iterator pts_in = inCloud.begin();
		pcl::PointCloud<pcl::PointXYZI>::iterator pts_out = phCloud.begin();
		pcl::PointXYZI p;

		while(pts_in != inCloud.end()){ //get ir like data
			p.x = pts_in->x; p.y = pts_in->y; p.z = pts_in->z; p.intensity = pts_in->intensity;
			if(p.x >= -L_RES && p.x <= L_RES){		// LINE RESOLUTION = 0.02 (meter)
				if(p.y >= 0) ir_points[1] = p;
				else ir_points[5] = p;
				pts_out = phCloud.insert(pts_out,p);	//for checking in rviz
			}else if(p.y - p.x >= -L_RES && p.y - p.x <= L_RES){ // y = x
				if(p.x >= 0) ir_points[2] = p;
				else ir_points[6] = p;
				pts_out = phCloud.insert(pts_out,p); 
			}else if(p.y + p.x >= -L_RES && p.y + p.x <= L_RES){ //y = -x
				if(p.x >= 0) ir_points[4] = p;
				else ir_points[0] = p;
				pts_out = phCloud.insert(pts_out,p);
			}else if(p.y >= -L_RES && p.y <= L_RES){
				if(p.x >= 0) ir_points[3] = p;
				pts_out = phCloud.insert(pts_out,p);
			}
			pts_in++;
		}
		pcl::toROSMsg(phCloud, outCloud);
	}
}

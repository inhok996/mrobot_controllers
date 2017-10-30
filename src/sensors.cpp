#include <mrobot_controllers/robot/sensors.h>
#include <stdio.h>

namespace mrobot_control
{
	laser_sensor::laser_sensor(void){}
	laser_sensor::~laser_sensor(void){}
	void laser_sensor::cloud_to_ir(pcl::PointCloud<pcl::PointXYZI>& inCloud)
	{
		//printf("laser_sensor::cloudto_ir called\n");
		phCloud.clear(); //clear phCloud
		for(int i = 0 ; i < NUM_IR_PTS; i++){
			ir_points[i].x = 0;
			ir_points[i].y = 0;
			ir_points[i].z = 0;
			ir_points[i].intensity = 0;
		}
		//printf("phCloud.points.size() = %d\n",phCloud.points.size());
		phCloud.header = inCloud.header;
		pcl::PointCloud<pcl::PointXYZI>::iterator pts_in = inCloud.begin();
		pcl::PointCloud<pcl::PointXYZI>::iterator pts_out = phCloud.begin();
		pcl::PointXYZI p;

		while(pts_in != inCloud.end()){ //get ir like data
			p.x = pts_in->x; p.y = pts_in->y; p.z = pts_in->z; p.intensity = pts_in->intensity;
			if(p.x >= -L_RES && p.x <= L_RES){  // x = 0
				if(p.y >= 0) ir_points[2] = p;
				else ir_points[10] = p;
				pts_out = phCloud.insert(pts_out,p);	//for checking in rviz
			}
			if(p.y - p.x >= -L_RES && p.y - p.x <= L_RES){ // y = x
				if(p.x >= 0) ir_points[4] = p;
				else ir_points[12] = p;
				pts_out = phCloud.insert(pts_out,p); 
			}
			if(p.y + p.x >= -L_RES && p.y + p.x <= L_RES){ //y = -x
				if(p.x >= 0) ir_points[8] = p;
				else ir_points[0] = p;
				pts_out = phCloud.insert(pts_out,p);
			}
			if(p.y >= -L_RES && p.y <= L_RES){ // y = 0
				if(p.x >= 0){ 
					ir_points[6] = p;
					pts_out = phCloud.insert(pts_out,p);
				}
			}
			if(((p.y - (0.5*p.x)) >= -L_RES) && ((p.y - (0.5*p.x)) <= L_RES)){ //y = 1/2x
				if(p.x >= 0){ 
					ir_points[5] = p;
					pts_out = phCloud.insert(pts_out,p);
				}
			}
			if(((p.y - (2.0*p.x)) >= -L_RES) && ((p.y + (2.0*p.x)) <= L_RES)){ //y = 2x
				if(p.x >= 0) ir_points[3] = p;
				else ir_points[11] = p;
				//pts_out = phCloud.insert(pts_out,p); 
			}
			if(((p.y + (0.5*p.x)) >= -L_RES) && ((p.y + (0.5*p.x)) <= L_RES)){ //y = -1/2x
				if(p.x >= 0){ 
					ir_points[7] = p;
					pts_out = phCloud.insert(pts_out,p);
				}
			}
			if(((p.y + (2.0*p.x)) >= -L_RES) &&(( p.y + (2.0*p.x)) <= L_RES)){ //y = -2x
				if(p.x >= 0) ir_points[9] = p;
				else ir_points[1] = p;
				//pts_out = phCloud.insert(pts_out,p);
			}
			pts_in++;
		}
		//for(int i = 0 ; i < 13 ; i++){
		//	printf("ir_points[%d].x = %lf,ir_points[%d] = %lf\n",i,ir_points[i].x,i,ir_points[i].y);
		//}
		pcl::toROSMsg(phCloud, outCloud);
	}
}

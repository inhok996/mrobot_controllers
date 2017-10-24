#include <mrobot_controllers/robot/sensors.h>
#include <stdio.h>

namespace mrobot_control
{
	laser_sensor::laser_sensor(void){}
	laser_sensor::~laser_sensor(void){}
	void laser_sensor::pharse_cloud(pcl::PointCloud<pcl::PointXYZI>& inCloud)
	{
		phCloud.clear(); //clear points
		//printf("phCloud.points.size() = %d\n",phCloud.points.size());
		phCloud.header = inCloud.header;
		pcl::PointCloud<pcl::PointXYZI>::iterator p = inCloud.begin();
		pcl::PointCloud<pcl::PointXYZI>::iterator p2 = phCloud.begin();
		pcl::PointXYZI temp;

		while(p != inCloud.end()){//points downsampling
			float x = p->x; 
			float y = p->y; 
			float z = p->z;
			if(y >= 0){ //get 6 points
				if(y < 0.02){ // y = 0.02
					temp.x = x; temp.y = y; temp.z = z; temp.intensity= p->intensity;
					p2 = phCloud.insert(p2,temp);
				}else if(y - x >= -0.02 && y - x <= 0.02){ // y = x
					temp.x = x; temp.y = y; temp.z = z; temp.intensity= p->intensity;
					p2 = phCloud.insert(p2,temp);
				}else if(y + x >= -0.02 && y + x <= 0.02){ //y = -x
					temp.x = x; temp.y = y; temp.z = z; temp.intensity= p->intensity;
					p2 = phCloud.insert(p2,temp);
				}else if(x >= -0.02 && x <= 0.02){
					temp.x = x; temp.y = y; temp.z = z; temp.intensity= p->intensity;
					p2 = phCloud.insert(p2,temp);
				}
			}else{
				;
			}
			p++;
		}
		pcl::toROSMsg(phCloud, outCloud);
	}
}

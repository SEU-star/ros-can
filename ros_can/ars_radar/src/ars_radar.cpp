#include"ars_radar.h"

using namespace std;
#define _NODE_NAME_ "radar_node"

bool ArsRadar::init()
{
	ros::NodeHandle nh;                                                                          // 创建节点句柄实例
	pub_cloud_    = nh.advertise<sensor_msgs::PointCloud2>("/ars_cloud",2);                      // 发布名为‘’ 话题  rviz里面显示
	sub_can_      = nh.subscribe("from_can_topic",100,&ArsRadar::canMsg_callback, this);         // 订阅从CAN分析仪里面发来的话题（里面包含各种数据帧的信息）
	
	ROS_INFO("ars radar initialization complete.");													   
	return true;
}

void ArsRadar::run()
{
	ros::spin();
}

void ArsRadar::canMsg_callback(const can_msgs::FrameArray::ConstPtr& msg)
{
	int n = msg->frames.size();                                                                 // 获取数据帧的大小
	int NumObjects = 0; 																		// 某一时刻检测到的目标的数量
	static bool published = true;																// 设置状态位置（为了避免传输时数据的丢失，设置状态位检测上一帧是否被成功发布，若么有被成功发布则强制发布）
	pcl::PointCloud<pcl::PointXYZ> cloud;                                                       // 创建一个点集
	sensor_msgs::PointCloud2 output;
	
	for(int i=0; i<n; i++)
	{
//		std::cout << std::hex << "id: " << int(msg->frames[i].id) << std::endl;

		/*
		if(0x701 == msg->frames[i].id) //		cluster
		{
			uint8_t  id       = msg->frames[i].data[0];
			float x_pos       = ((msg->frames[i].data[1] << 5) + (msg->frames[i].data[2] >> 3)) * 0.2 - 500;
			float y_pos       = (((msg->frames[i].data[2] & 0x03) << 8) + (msg->frames[i].data[3])) * 0.2 - 500;
			float x_speed     = ((msg->frames[i].data[4] << 2) + (msg->frames[i].data[5] >> 6)) * 0.25 - 128;
			float y_speed     = (((msg->frames[i].data[5] & 0x3f) << 3) + (msg->frames[i].data[6] >> 5)) * 0.25 - 64;
			uint8_t  t_status = (msg->frames[i].data[6] & 0x07);
		}
		*/

		if(0x60a == msg->frames[i].id)                                                         // 每次进来都是新的 new object array
		{
			if(published == false)                                                             // 如果上一帧没有被成功发布，则强制发布
			{
				pcl::toROSMsg(cloud, output);                                                  // 数据转换：把pcl::PointCloud格式的转换为sensor_msgs::PointCloud2格式
				output.header.frame_id = "ars_radar";                                          
				pub_cloud_.publish(output);                                                    // 发布出去
			}									
			NumObjects = msg->frames[i].data[0];											   // 获取障碍物的个数
//			std::cout << NumObjects << std::endl;
			cloud.points.reserve(NumObjects);             		                               // 根据障碍物的个数，预留大小空间
			published = false;															       // 状态位置取反
		}
	    else if(0x60b == msg->frames[i].id)                                                    //		object
		{
			uint8_t  id       = msg->frames[i].data[0];										   // 读取数据
			float xh_pos      = msg->frames[i].data[1] << 5;
			float xl_pos      = msg->frames[i].data[2] >> 3;
			float x_pos       = (xh_pos + xl_pos) * 0.2 -500;
			
			float yh_pos      = (msg->frames[i].data[2] & 0x07) << 8;
			float yl_pos      = msg->frames[i].data[3];
			float y_pos       = (yh_pos + yl_pos) * 0.2 -204.6;
			
			float xh_speed    = msg->frames[i].data[4] << 2;
			float xl_speed    = msg->frames[i].data[5] >> 6;
			float x_speed     = (xh_speed + xl_speed) * 0.25 - 128;
			
			float yh_speed    = (msg->frames[i].data[5] & 0x3f) << 3;
			float yl_speed    = msg->frames[i].data[6] >> 5;
			float y_speed     =  (yh_speed + yl_speed) * 0.25 - 64;
			
			uint8_t  t_status = (msg->frames[i].data[6] & 0x07);
		
			float distance    = sqrt((pow(x_pos,2) + pow(y_pos,2)));
			float theta       = atan2(y_pos, x_pos);
			
			printf("x_speed:%0.2f\t y_speed:%0.2f\t distance:%.2f\t theta:%.2f\n",x_speed,y_speed,distance,theta*180.0/M_PI);

			pcl::PointXYZ point;                                                              // 创建一个点来记录其：x,y坐标
			point.x = x_pos;
			point.y = y_pos;
			point.z = 1.0;
			cloud.points.push_back(point);                                                    // 放入点云里面
			if(NumObjects == cloud.points.size())                                             // 如果这一帧的所有障碍物信息都放满了，便一次性发布
			{
				pcl::toROSMsg(cloud, output); 
				output.header.frame_id = "ars_radar";
				pub_cloud_.publish(output); 												  // 转换消息类型后发布 
				published = true;                                                             // 状态位置位 
			}			
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, _NODE_NAME_);                                                       // 进主函数后立即 ros::init(argc,argv, _NODE_NAME_)
	ArsRadar app;																		     // 创建雷达对象
	if(app.init())
	{
		app.run();                                                                           // 开始运行
	}
}



























/*
 *
void RadarConfig(ros::Publisher& pub)
{
	//雷达配置
	can_msgs::Frame radarConfigMsg;
   	radarConfigMsg.id       = 0x0200;
	radarConfigMsg.len      = 0x08;
	
   	radarConfigMsg.data[0]  = 0xC8;
   	radarConfigMsg.data[1]  = 0x00;
	radarConfigMsg.data[2]  = 0x00;
	radarConfigMsg.data[3]  = 0x00;
	radarConfigMsg.data[4]  = 0x10;    //0x10: 配置radar的输出格式为cluster; 0x08: Object
	radarConfigMsg.data[5]  = 0x90;
	radarConfigMsg.data[6]  = 0x00;
	radarConfigMsg.data[7]  = 0x00;   
	
	can_msgs::FrameArray frameArray1;
	frameArray1.header.frame_id = "ch1";
	frameArray1.frames.push_back(radarConfigMsg);   
	
//	frameArray1.frames.resize(1);
//	frameArray1.frames[0] = 	radarConfigMsg;
	
	pub.publish(frameArray1);
      
    //滤波器配置
    can_msgs::Frame FilterConfigMsg;
    FilterConfigMsg.id  	= 0x0202;
    FilterConfigMsg.len     = 0x05;
    
    FilterConfigMsg.data[0] = 0x0C;   //0x0C:配置输出格式为Cluster时的滤波器设置;0x8C：object
    FilterConfigMsg.data[1] = 0x00;
    FilterConfigMsg.data[2] = 0x0A;   //滤波器最小距离配置为1m
    FilterConfigMsg.data[3] = 0x0F;
    FilterConfigMsg.data[4] = 0xA0;   //滤波器最大距离配置为400m
    
    can_msgs::FrameArray frameArray2;
	frameArray2.header.frame_id = "ch1";
	frameArray2.frames.push_back(FilterConfigMsg);
    
    pub.publish(frameArray2);
}
 *

	ros::spin();
	return 0;
	
}
 */

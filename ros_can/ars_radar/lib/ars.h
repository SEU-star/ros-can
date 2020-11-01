#ifndef ARS_H
#define ARS_H

#include <vector>
#include <string>
#include <thread>
#include <iostream>
#include <ros/ros.h>
#include <can_msgs/FrameArray.h>

#include <can_msgs/Object.h>
#include <can_msgs/ObjectArray.h>
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <unordered_map>

/*
 *
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
 *
 */

class ArsRadar
{

public:

	ArsRadar();
   ~ArsRadar(){};
   
   	bool init();
	void run();
	//目标的运动状态
	enum Cluster_DynProp
	{
		Target_Move = 0,   				//移动
		Target_Static = 1, 				//静止
		Target_Come = 2,   				//来向
		Target_May_Static = 3,			//可能静止
		Target_Unknow = 4,				//未知
		Target_Across = 5,				//横穿
		Target_Across_Static = 6,	    //横穿静止
		Target_Stop = 7					//停止
	};
	
	typedef struct _Info                //定义结构体时要给一个默认构造函数
	{
		std::string type;
		uint8_t r;
		uint8_t g;
		uint8_t b;
		_Info(const std::string& t, uint8_t _r, uint8_t _g, uint8_t _b)   // 传引用的方式可以少一次复制 
		{
			type = t;
			r = _r;
			g = _g;
			b = _b;
		}
	}Info;
	
	std::vector<Info> Infos;   // 定义一个类型为Info的向量
						
private:
	
	void canMsg_callback(const can_msgs::FrameArray::ConstPtr& msg);
	void parse_msg(const can_msgs::Frame &frame);
	void radarConfig(ros::Publisher& pub);

    ros::Subscriber sub_can_;
	ros::Publisher  pub_can_;
	ros::Publisher  pub_objects_;
	ros::Publisher  pub_cloud_;
	
	
	can_msgs::ObjectArray ars_objects_;
	can_msgs::Object  ars_object_;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;   // 智能指针

	std::unordered_map<int, size_t> map;  // 哈系列表
	                                              
	sensor_msgs::PointCloud2 output;
	pcl::PointXYZRGB point; 
	
	
//	jsk_recognition_msgs::BoundingBoxArray bbox_array_;
//	jsk_recognition_msgs::BoundingBox      bbox_;
	
};



#endif

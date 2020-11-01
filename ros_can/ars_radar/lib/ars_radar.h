#ifndef ARS_RADAR_H
#define ARS_RADAR_H

#include <vector>
#include <thread>
#include <iostream>
#include <ros/ros.h>
#include <can_msgs/FrameArray.h>
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 

/*
 *
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
 *
 */

class ArsRadar
{

public:

	ArsRadar(){};
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
   
private:
	
    void canMsg_callback(const can_msgs::FrameArray::ConstPtr& msg);
	void parse_msg(const can_msgs::Frame &frame);


    ros::Subscriber sub_can_;
	ros::Publisher  pub_can_;
	ros::Publisher  pub_objects_;
	ros::Publisher  pub_cloud_;
	
	
	can_msgs::ObjectArray ars_objects_;
	can_msgs::Object  ars_object_;
	
	pcl::PointCloud<pcl::PointXYZRGB> cloud;                                                   
	sensor_msgs::PointCloud2 output;
	pcl::PointXYZRGB point; 
	
//	jsk_recognition_msgs::BoundingBoxArray bbox_array_;
//	jsk_recognition_msgs::BoundingBox      bbox_;
	
};




























































//typedef struct Radar_Config
//{

//    /*一共8个字节*/

//    //状态信息：一个字节
//    uint8_t RadarCfg_MaxDistance_valid: 1;       //最大距离配置位是否有效
//    uint8_t RadarCfg_SensorID_valid: 1;
//    uint8_t RadarCfg_RadarPower_valid: 1;
//    uint8_t RadarCfg_OutputType_valid: 1;
//    uint8_t RadarCfg_SendQuality_valid: 1;
//    uint8_t RadarCfg_SendExtInfo_valid: 1;
//    uint8_t RadarCfg_SortIndex_valid: 1; 
//    uint8_t RadarCfg_StoreInNVM_valid: 1;

//    //两个字节
//    uint16_t RadarCfg_MaxDistance: 10;          //最大距离配置位
//    uint16_t: 6;                                //不使用

//    //一个字节
//    uint8_t: 8;                                 //不使用

//    //一个字节
//    uint8_t RadarCfg_SensorID: 3;
//    uint8_t RadarCfg_OutputType: 2;             //输出格式配置位：0x0: 无，0x1: objects，0x2: clusters
//    uint8_t RadarCfg_RadarPower: 3;

//    //一个字节
//    uint8_t RadarCfg_CtrlRelay_valid: 1;
//    uint8_t RadarCfg_CtrlRelay: 1;
//    uint8_t RadarCfg_SendQuality: 1;
//    uint8_t RadarCfg_SendExtInfo: 1;
//    uint8_t RadarCfg_SortIndex: 3;
//    uint8_t RadarCfg_StoreInNVM: 1;

//    //一个字节
//    uint8_t RadarCfg_RCS_Threshold_valid: 1;
//    uint8_t RadarCfg_RCS_Threshold: 3;
//    uint8_t: 4;                                  // 不使用

//    //一个字节
//    uint8_t: 8;

//}radarcfg_t;

//typedef struct RadarFilter_Config
//{
//	//配置Cluster/Object的滤波器为0x1模式：此模式下 FilterCfg_Min_X和FilterCfg_Max_X12位长度
//	uint8_t: 1;
//	uint8_t FilterCfg_Vaild: 1;
//	uint8_t FilterCfg_Active: 1;
//	uint8_t FilterCfg_Index: 4;
//	uint8_t FilterCfg_Type: 1;
//	
//	uint16_t FilterCfg_Min_Distance: 12;
//	uint16_t: 4;
//	uint16_t FilterCfg_Max_D: 12;
//	uint16_t: 4;

////	配置Cluster/Object的滤波器为0xA模式：此模式下 FilterCfg_Min_X和FilterCfg_Max_X13位长度
////	uint8_t: 1;
////	uint8_t FilterCfg_Vaild: 1;
////	uint8_t FilterCfg_Active: 1;
////	uint8_t FilterCfg_Index: 4;
////	uint8_t FilterCfg_Type: 1;
////	
////	uint16_t FilterCfg_Min_X: 13;
////	uint16_t: 3;
////	uint16_t FilterCfg_Max_X: 13;
////	uint16_t: 3;

//}Filter_t;


#endif

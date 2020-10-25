# 大陆雷达 ARS408-21 数据读取

## overview
    订阅CAN分析仪发布的ros消息，并将其解析，[CAN分析仪购买地址](https://item.taobao.com/item.htm?spm=a1z2k.11010449.931864.62.275f509dPWvRhX&scm=1007.13982.82927.0&id=18286496283&last_time=1589127269)


## Nodes

### Node ars_radar_node
     解析CAN分析仪发布的ros消息
### 启动方法
	 roslaunch ars_radar ars_radar.launch
	 在rviz里面选择frame为：ars_radar;并添加PointCloud2类型的显示格式;topic选为/ars_cloud 。
	 
	 

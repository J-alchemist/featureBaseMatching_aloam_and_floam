// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"

std::string tracked_frame_id = "base_link";

LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf; // 雷达数据的回调存放
lidar::Lidar lidar_param;   // 设置雷达的参数

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
   
}

double total_time =0;
int total_frame=0;

void laser_processing(){
    while(1){
        if(!pointCloudBuf.empty()){
            //read data
            // 拿到雷达数据
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            // 20221108 by guojian
            // must!!!
            std::vector<int> vec_idx;
            pcl::removeNaNFromPointCloud(*pointcloud_in, *pointcloud_in, vec_idx); 
            //----------------------
            // ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            ros::Time pointcloud_time = ros::Time::now();

            pointCloudBuf.pop();
            mutex_lock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();   // 计时
            // ! 提取边点和面点（特征点）
            laserProcessing.featureExtraction(pointcloud_in, pointcloud_edge, pointcloud_surf);  

            end = std::chrono::system_clock::now();         
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;                                                       
            total_time+=time_temp;
            //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);
                
            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = tracked_frame_id;
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = tracked_frame_id;
            pubEdgePoints.publish(edgePointsMsg);

            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = tracked_frame_id;
            pubSurfPoints.publish(surfPointsMsg);           
        }   
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    std::string points_topic = "/velodyne_points";

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/points_topic", points_topic);
    nh.getParam("odom_child_frame_id", tracked_frame_id);       // 追踪的坐标系 

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);
    laserProcessing.init(lidar_param);
    // 订阅
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(points_topic, 100, velodyneHandler); // 只负责接收雷达数据
    // 发布 
    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);   // 边特征+面特征的合集， 只需要定位，不需要建图时，可以不发布
    pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100);   // 边特征
    pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100);   // 面特征

    std::thread laser_processing_process{laser_processing};     // ! core

    ros::spin();

    return 0;
}


// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.


#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include "a_loam/common.h"
#include "a_loam/tic_toc.h"
#include "lidarFactor.hpp"

#define DISTORTION 0          // 插值 ，1：按照实际点的扫描时刻插值，0：匀速假设插值

int corner_correspondence = 0, plane_correspondence = 0;

double SCAN_PERIOD = 0.1;     // 雷达频率
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

int skipFrameNum = 5;         // 特征发布频率
bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;

// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1};        //单位四元素  旋转
double para_t[3] = {0, 0, 0};           //平移

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q); // map是eigen的内存映射，para_q映射为q_last_curr类型，q_last_curr是Quaterniond类型
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
std::mutex mBuf;

// undistort lidar point
// 将当前帧Lidar坐标系下的点云变换到上一帧Lidar坐标系下（也就是当前帧的初始位姿，起始位姿，所以函数名是TransformToStart）
void TransformToStart(PointType const *const pi, PointType *const po)
{
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    //s = 1;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
} 

// transform all lidar points to the start of the next frame
void TransformToEnd(PointType const *const pi, PointType *const po)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}

void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    mBuf.lock();
    cornerSharpBuf.push(cornerPointsSharp2);
    mBuf.unlock();
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}

//receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    //if 1, do mapping 10 Hz, if 2, do mapping 5 Hz. Suggest to use 1, it will adjust frequence automaticlly
    nh.param<int>("mapping_skip_frame", skipFrameNum, 1);   // 特征发布频率
    nh.param<double>("scan_fre", SCAN_PERIOD, 0.2);
    printf("Mapping: %d Hz \n", 10 / skipFrameNum);

    // 订阅： 特征点， 接收数据并存储
    // less_xx(次极大特征)中是包含了极大特征， 极大特征集合是他的子集
    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);   // 极大特征（边缘线特征）
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);  // 次极大特征
    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);        // 平面点特征
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);
            // 前一节点处理过后的点云，xxx2
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);  

    //发布
    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);     // 发布点云xx3
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);         // 里程计
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);           

    nav_msgs::Path laserPath;

    int frameCount = 0;
    ros::Rate rate(100);

    while (ros::ok())
    {
        ros::spinOnce();        // 触发一次回调

        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&           // 均有数据
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty())
        {
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();       // 时间戳
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();

            if (timeCornerPointsSharp != timeLaserCloudFullRes ||                               // 时间戳对齐
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                printf("unsync messeage!");  
                ROS_BREAK();        // 用于中断程序并输出本句所在文件/行数    ros/assert.h
            }

            // 取出首元素
            mBuf.lock();
            cornerPointsSharp->clear(); 
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);       //格式转换  首地址元素开始转换  取出一帧
            cornerSharpBuf.pop();    //弹出前面一个元素 实现更新    

            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();

            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();

            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();

            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            fullPointsBuf.pop();
            mBuf.unlock();

            TicToc t_whole;
            // initializing
            if (!systemInited)
            {
                // 主要用来跳过第一帧数据
                // 仅仅将cornerPointsLessSharp保存至laserCloudCornerLast
                // 将surfPointsLessFlat保存至laserCloudSurfLast，以及更新对应的kdtree
                systemInited = true;
                std::cout << "Initialization finished \n";
            } 
            else
            {   // 当前帧特征点size
                int cornerPointsSharpNum = cornerPointsSharp->points.size();   // 2x6x16
                int surfPointsFlatNum = surfPointsFlat->points.size();      // 4x6x16 

                TicToc t_opt;
                for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)     // ceres匹配  2次迭代求解
                {
                    corner_correspondence = 0;  // 角点成功匹配的点对的数量
                    plane_correspondence = 0;   // 面点成功匹配的点对的数量

                    // 【1】设置ceres求解器，构建最小二乘问题
                    ceres::LossFunction *loss_function = NULL;
                    loss_function = new ceres::HuberLoss(0.1);     // 设置核函数的鲁棒核函数huber , 残差大于0.1的点 ,则权重降低
                    // loss_function = new ceres::CauchyLoss(0.1);                   // 柯西核函数   

                    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization(); // 由于旋转不满足一般的加法,所以要用 ceres自带的local Param去解析独特的加法规则 ,顺序x,y,z,w
                    ceres::Problem::Options problem_options;
                    
                    ceres::Problem problem(problem_options);    
                    problem.AddParameterBlock(para_q, 4, q_parameterization);        // 待优化变换矩阵
                    problem.AddParameterBlock(para_t, 3);

                    pcl::PointXYZI pointSel;                //畸变校正后的点云
                    std::vector<int> pointSearchInd;        //最近邻id
                    std::vector<float> pointSearchSqDis;    //最近邻距离

                    TicToc t_data;
                    // find correspondence for corner features
                    // ! 使用当前帧的极大特征在历史帧中的大集合(即less，包含极大和次极大)中寻找匹配
                    /** 
                     * 为每个当前帧特征在历史帧特征中寻找，查找次数cornerPointsSharpNum
                     * 先在历史帧中寻找与当前帧最近邻的特征，返回距离和索引；
                     * 然后再ring值差距在NEARBY_SCAN内的，另一个历史帧特征
                     * 得到索引closestPointInd 与 minPointInd2
                     * 构建点线
                    */
                    // ! 为每一个当前帧特征；
                    // ! 在历史帧特征中寻找最近邻特征；
                    // ! 历史帧中的的两个特征点异线（ring要不一样）
                    for (int i = 0; i < cornerPointsSharpNum; ++i)      // ! 1-点线匹配
                    {
                        /*
                        现在在第k+1帧（当前帧）中发现了点边线点i，查询在第k帧中的最近邻点j（by KD-tree），
                        查询j的附近扫描线上的最近邻点l，j与l相连形成一条直线l-j，让点i与这条直线的距离最短;
                        */
                        TransformToStart( &(cornerPointsSharp->points[i]), &pointSel ); ////去运动畸变,统一到起始点
                        kdtreeCornerLast->nearestKSearch( pointSel, 1, pointSearchInd, pointSearchSqDis );  // 在kdtreeCornerLast中查找pointSel的最近邻1个点

                        int closestPointInd = -1, minPointInd2 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)    // 特征间距离小于阈值，有效
                        {
                            closestPointInd = pointSearchInd[0];    // kdtreeCornerLast中最近特征点id
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);      // 最近特征点所在线束ring

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                            // search in the direction of increasing scan line
                            // ! 线束增加方向搜索
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                            { 
                                // 查找附近线束
                                // ring值差距在NEARBY_SCAN以内的
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop 
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;
                                // 欧式距离的平方
                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;    // 更新 minPointSqDis2，在decreasing方向搜索的时候的判断条件
                                    minPointInd2 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            // ! 线束减小方向搜索
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                { 
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                        }

                        // ! 特征点i的两个最近邻点j和m都有效，构建非线性优化问题
                        if (minPointInd2 != -1) // both closestPointInd and minPointInd2 is valid
                        {
                            // 当前帧的点
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);
                            // 上一帧的a，b点构成线
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);

                            double s;
                            if (DISTORTION)     // 进行插值     
                                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;   // 分子为小数部分，当前点的时间戳占比
                            else
                                s = 1.0;
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);    // 求解代价函数
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t); // 添加 代价函数、核函数、优化变量
                            corner_correspondence++; 
                        } 
                    }

                    // find correspondence for plane features
                    for (int i = 0; i < surfPointsFlatNum; ++i)         // ! 2-点面匹配
                    {
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);      //最近邻
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];

                            // get closest point's scan ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or lower scan line
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                // if in the higher scan line
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or higher scan line
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    // find nearer point
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            if (minPointInd2 >= 0 && minPointInd3 >= 0)         // cere优化
                            {

                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                            surfPointsFlat->points[i].y,
                                                            surfPointsFlat->points[i].z);
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                                laserCloudSurfLast->points[closestPointInd].y,
                                                                laserCloudSurfLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                                laserCloudSurfLast->points[minPointInd2].y,
                                                                laserCloudSurfLast->points[minPointInd2].z);
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                                laserCloudSurfLast->points[minPointInd3].y,
                                                                laserCloudSurfLast->points[minPointInd3].z);

                                double s;
                                if (DISTORTION)
                                    s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                                else
                                    s = 1.0;
                                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                plane_correspondence++;    
                            } 
                        }
                    }

                    //printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                    printf("data association time %f ms \n", t_data.toc());

                    if ((corner_correspondence + plane_correspondence) < 10)        // 特征匹配不太上(特征相关性弱)
                    {
                        printf("less correspondence! *************************************************\n");
                    }

                    TicToc t_solver;
                    // 【2】设置求解器
                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR;   // 增量方程的求解方式
                    options.max_num_iterations = 4;                 // 优化次数
                    options.minimizer_progress_to_stdout = false;   // 输出到cout
                    ceres::Solver::Summary summary;                 // 优化过程中的信息
                    ceres::Solve(options, &problem, &summary);
                    printf("solver time %f ms \n", t_solver.toc());
                }
                printf("optimization twice time %f \n", t_opt.toc());
                //?把计算的当前帧和上一帧的变换累加,形成相对第一帧的位姿变换,也就是世界坐标系下的位姿
                // param_q和param_t关联到的q_w_curr和t_last_curr
                // 转换为当前帧相当于第一帧的位姿
                t_w_curr = t_w_curr + q_w_curr * t_last_curr;
                q_w_curr = q_w_curr * q_last_curr;
            }

            TicToc t_pub;

            // publish odometry
            nav_msgs::Odometry laserOdometry;                   //! 里程计发布
            laserOdometry.header.frame_id = "/camera_init";
            laserOdometry.child_frame_id = "/laser_odom";
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            pubLaserOdometry.publish(laserOdometry);

            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "/camera_init";
            pubLaserPath.publish(laserPath);

            // transform corner features and plane features to the scan end point
            if (0) 
            {
                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
                for (int i = 0; i < cornerPointsLessSharpNum; i++)
                {
                    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
                }

                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
                for (int i = 0; i < surfPointsLessFlatNum; i++)
                {
                    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
                }

                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }
            }

            // 当前所有特征（极大特征+次极大特征）赋予last
            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;
            laserCloudCornerLast = laserCloudTemp;      // last保存了所有特征（极大+次极）

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;

            // laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            // laserCloudSurfLastNum = laserCloudSurfLast->points.size();
            // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';
           
            // 存储特征在kd树上
            // ! kd树每次只会保存最近一次setInputCloud的数据
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);      
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

            if (frameCount % skipFrameNum == 0) // 发布
            {
                frameCount = 0;
                std::string frame_id = "/camera";       // ! 角点、面点、所有点云

                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = frame_id;
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);             

                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = frame_id;
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = frame_id;
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }
            printf("publication time %f ms \n", t_pub.toc());
            printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
            if(t_whole.toc() > 100)
                ROS_WARN("odometry process over 100ms");

            frameCount++;
        }
        rate.sleep();
    }
    return 0;
}

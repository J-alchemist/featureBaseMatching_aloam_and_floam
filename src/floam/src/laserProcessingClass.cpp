// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include "laserProcessingClass.h"

void LaserProcessingClass::init(lidar::Lidar lidar_param_in){
    
    lidar_param = lidar_param_in;

}

void LaserProcessingClass::featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf){

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, indices);

    int N_SCANS = lidar_param.num_lines;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudScans;
    for(int i=0;i<N_SCANS;i++){
        laserCloudScans.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));     // 分配每根线束的空间
    }

    for (int i = 0; i < (int) pc_in->points.size(); i++)        // ring值
    {
        int scanID=0;
        double distance = sqrt(pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y);
        if(distance<lidar_param.min_distance || distance>lidar_param.max_distance)  // 雷达远近值滤波
            continue; 
        double angle = atan(pc_in->points[i].z / distance) * 180 / M_PI;
        
        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            if (angle > 2 || angle < -24.33 || scanID > 63 || scanID < 0)
            {
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
        }
        laserCloudScans[scanID]->push_back(pc_in->points[i]); 

    }

    for(int i = 0; i < N_SCANS; i++){                   // 每跟线束 提取
        if(laserCloudScans[i]->points.size()<131){
            continue;
        }
        
        std::vector<Double2d> cloudCurvature;           // 每个点的曲率
        int total_points = laserCloudScans[i]->points.size()-10;
        for(int j = 5; j < (int)laserCloudScans[i]->points.size() - 5; j++){
            double diffX = laserCloudScans[i]->points[j - 5].x + laserCloudScans[i]->points[j - 4].x + laserCloudScans[i]->points[j - 3].x + laserCloudScans[i]->points[j - 2].x + laserCloudScans[i]->points[j - 1].x - 10 * laserCloudScans[i]->points[j].x + laserCloudScans[i]->points[j + 1].x + laserCloudScans[i]->points[j + 2].x + laserCloudScans[i]->points[j + 3].x + laserCloudScans[i]->points[j + 4].x + laserCloudScans[i]->points[j + 5].x;
            double diffY = laserCloudScans[i]->points[j - 5].y + laserCloudScans[i]->points[j - 4].y + laserCloudScans[i]->points[j - 3].y + laserCloudScans[i]->points[j - 2].y + laserCloudScans[i]->points[j - 1].y - 10 * laserCloudScans[i]->points[j].y + laserCloudScans[i]->points[j + 1].y + laserCloudScans[i]->points[j + 2].y + laserCloudScans[i]->points[j + 3].y + laserCloudScans[i]->points[j + 4].y + laserCloudScans[i]->points[j + 5].y;
            double diffZ = laserCloudScans[i]->points[j - 5].z + laserCloudScans[i]->points[j - 4].z + laserCloudScans[i]->points[j - 3].z + laserCloudScans[i]->points[j - 2].z + laserCloudScans[i]->points[j - 1].z - 10 * laserCloudScans[i]->points[j].z + laserCloudScans[i]->points[j + 1].z + laserCloudScans[i]->points[j + 2].z + laserCloudScans[i]->points[j + 3].z + laserCloudScans[i]->points[j + 4].z + laserCloudScans[i]->points[j + 5].z;
            Double2d distance(j, diffX * diffX + diffY * diffY + diffZ * diffZ);        // 每个点的索引和该点的曲率
            cloudCurvature.push_back(distance);
        }
        // 每根线束提取6次
        for(int j=0;j<6;j++){
            int sector_length = (int)(total_points/6);      // 每个扇区长度
            int sector_start = sector_length *j;
            int sector_end = sector_length *(j+1)-1;
            if (j==5){
                sector_end = total_points - 1; 
            }
            std::vector<Double2d> subCloudCurvature(cloudCurvature.begin()+sector_start,cloudCurvature.begin()+sector_end);     // 每根线的每个方向的起始结束
            
            featureExtractionFromSector(laserCloudScans[i],subCloudCurvature, pc_out_edge, pc_out_surf);        // 每根线束的6个方向提取特征
        }

    }

}


void LaserProcessingClass::featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, std::vector<Double2d>& cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf){
    // 升序排列
    std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const Double2d & a, const Double2d & b)    
    { 
        return a.value < b.value; 
    });

    int largestPickedNum = 0;
    std::vector<int> picked_points;
    int point_info_count =0;
    for (int i = cloudCurvature.size()-1; i >= 0; i--)
    {
        int ind = cloudCurvature[i].id; 
        if(std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){    // 没有找到ind返回end(), 避免特征元素重复
            if(cloudCurvature[i].value <= 0.1){
                break;
            }
            
            largestPickedNum++;
            picked_points.push_back(ind);
            
            if (largestPickedNum <= 20){            // 20个边点
                pc_out_edge->push_back(pc_in->points[ind]);
                point_info_count++;
            }else{
                break;
            }
            // 把特征点附近(前后5个点, 两两之间的距离)的距离近的点也标记了(不能作为特征)
            for(int k=1;k<=5;k++){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                } 
                picked_points.push_back(ind+k);
            }
            for(int k=-1;k>=-5;k--){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }

        }
    }

    //find flat points
    // point_info_count =0;
    // int smallestPickedNum = 0;
    
    // for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
    // {
    //     int ind = cloudCurvature[i].id; 

    //     if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
    //         if(cloudCurvature[i].value > 0.1){
    //             //ROS_WARN("extracted feature not qualified, please check lidar");
    //             break;
    //         }
    //         smallestPickedNum++;
    //         picked_points.push_back(ind);
            
    //         if(smallestPickedNum <= 4){
    //             //find all points
    //             pc_surf_flat->push_back(pc_in->points[ind]);
    //             pc_surf_lessFlat->push_back(pc_in->points[ind]);
    //             point_info_count++;
    //         }
    //         else{
    //             break;
    //         }

    //         for(int k=1;k<=5;k++){
    //             double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
    //             double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
    //             double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
    //             if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
    //                 break;
    //             }
    //             picked_points.push_back(ind+k);
    //         }
    //         for(int k=-1;k>=-5;k--){
    //             double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
    //             double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
    //             double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
    //             if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
    //                 break;
    //             }
    //             picked_points.push_back(ind+k);
    //         }

    //     }
    // }
    
    for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)     // 面点
    {
        int ind = cloudCurvature[i].id; 
        if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end())        // 从未标记的点里选取
        {
            // if (cloudCurvature[i].value < 0.1)          // me add 20221013  曲率小
                pc_out_surf->push_back(pc_in->points[ind]);
        }
    }   

}
LaserProcessingClass::LaserProcessingClass(){
    
}

Double2d::Double2d(int id_in, double value_in){
    id = id_in;
    value =value_in;
};

PointsInfo::PointsInfo(int layer_in, double time_in){       // 未用
    layer = layer_in;
    time = time_in;
};

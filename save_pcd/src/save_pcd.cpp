#include <cmath>
#include <vector>
#include <string>
#include "save_pcd/common.h"
#include "save_pcd/tic_toc.h"
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include<mutex>
#include<math.h>
#include <eigen3/Eigen/Dense>
#define PI 3.1415926
using std::atan2;
using std::cos;
using std::sin;
std::mutex mBuf;


std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> Vec16;
std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> Vec32;


std::string calibrationpcddir_ = " ";

void savetotalpcd()
{

    int key=0;
    static int index = 0;
    cv::Mat img=cv::Mat::zeros(100,100,CV_8U);
    cv::imshow("waitkey",img);
    key = cv::waitKey(1);
    std::cout<<key<<std::endl;

        if (Vec16.size() == 0 || Vec32.size() == 0)
        {
            return;
        }

    if(key=='s')
    {
        std::string lidarID = "0";
        pcl::PointCloud<pcl::PointXYZ> filtercloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointclouds = Vec16[Vec16.size()-1];
        for(int i=0;i<pointclouds->points.size();i++)
        {
            double range = std::sqrt(pointclouds->points[i].x * pointclouds->points[i].x + pointclouds->points[i].y * pointclouds->points[i].y + pointclouds->points[i].z * pointclouds->points[i].z);
            if(range>0.5)
                filtercloud_.push_back(pointclouds->points[i]);
        }
    index++;
    std::stringstream sstr;
    sstr<<index<<"_"<<lidarID<<".pcd";
    std::string path = calibrationpcddir_ + sstr.str();
    pcl::io::savePCDFile(path, filtercloud_,false);


    lidarID = "1";
    pcl::PointCloud<pcl::PointXYZ> filtercloud_32;
    pointclouds = Vec32[Vec32.size()-1];
    for(int i=0;i<pointclouds->points.size();i++)
    {
        double range = std::sqrt(pointclouds->points[i].x * pointclouds->points[i].x + pointclouds->points[i].y * pointclouds->points[i].y + pointclouds->points[i].z * pointclouds->points[i].z);
        if(range>0.5)
            filtercloud_32.push_back(pointclouds->points[i]);
    }
    std::stringstream sstr32;
    sstr32<<index<<"_"<<lidarID<<".pcd";
    path = calibrationpcddir_ + sstr32.str();
    pcl::io::savePCDFile(path, filtercloud_32,false);
    }
}

void laserCloudHandler16(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{

    // std::cout<<"激光回调"<<std::endl;

    TicToc t_whole;
    TicToc t_prepare;


    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

    Vec16.push_back(laserCloudIn);
    // savetotalpcd(laserCloudIn);
}


void laserCloudHandler32(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

    Vec32.push_back(laserCloudIn);
    // savetotalpcd(laserCloudIn);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sphere");
    ros::NodeHandle nh;
    ros::param::get("~shot_save", calibrationpcddir_);
    // **************************************************************************************************************
    ros::Subscriber subLaserCloud16 = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points_16", 1, laserCloudHandler16);
    ros::Subscriber subLaserCloud32 = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points_32", 1, laserCloudHandler32);

    while(ros::ok())
    {
        ros::spinOnce();
        savetotalpcd();
    }

    return 0;
}

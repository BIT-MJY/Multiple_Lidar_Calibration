#include <cmath>
#include <vector>
#include <string>
#include "save_pcd/common.h"
#include "save_pcd/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/filters/passthrough.h>  
#include <pcl/common/common.h>
#include <vtkAreaPicker.h>
#include <string>
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
 #include <pcl/filters/statistical_outlier_removal.h> 
#include<mutex>

#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>

using std::atan2;
using std::cos;
using std::sin;

#define PI 3.1415926
using std::atan2;
using std::cos;
using std::sin;
std::mutex mBuf;


std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> Vec16;
std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> Vec32;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_show(new pcl::PointCloud<pcl::PointXYZ>);



bool finishLable = false;
bool nextLable = true;



std::string filename_pt = "  ";
std::string filename_pt_save = "  ";
std::string filesuffix;
std::string num;


void areapickingcallback(const pcl::visualization::AreaPickingEvent &event,void *userdata)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr  secloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<int> indices;
    event.getPointsIndices(indices);
    pcl::IndicesPtr ind_plane=boost::make_shared<std::vector<int>>(indices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_show);                       //导入点云数据
    extract.setIndices(ind_plane);                      //设置点云索引
    extract.setNegative(false);                  //设置为false，选择索引指向的点导出
    extract.filter(*secloud);                         //输出所选点云
    std::cout<<"Nums selected\t"<<secloud->points.size()<<std::endl;
    if (secloud->points.size()==0)
	{
        std::cout<<"您已手滑，并未选中任何点云，将重新弹出本帧点云，请再次框选"<<std::endl;
        nextLable = false;
        finishLable = true;
	    return;
	}


    // 2.0 version 决定是否使用本次标注的点云
    std::cout<<"是否使用本次框选的点云数据?: y(Yes)/n(No): "<<std::endl;
    std::string save_char;
    std::cin>>save_char;
    if (save_char == std::string("y") || save_char == std::string("yes") || save_char == std::string("Y"))
    {
            // // 先发布出去
            // sensor_msgs::PointCloud2 labeledObjCloudMsg;
            // pcl::toROSMsg(*secloud, labeledObjCloudMsg);
            // labeledObjCloudMsg.header.stamp = ros::Time::now();
            // labeledObjCloudMsg.header.frame_id = "/rslidar";    //    we need change to global fixed frame
            // pubObjects.publish(labeledObjCloudMsg);
            
            std::stringstream sstr32;
             ros::param::get("~filename_pt_save", filename_pt_save);
            std::string calibrationpcddir_ = filename_pt_save;
            calibrationpcddir_.append(num);
            calibrationpcddir_.append(filesuffix);  

            pcl::io::savePCDFile(calibrationpcddir_, *secloud,false);
            nextLable = true;

        }

    else if(save_char == std::string("n") || save_char == std::string("no") || save_char == std::string("N"))
    {
        std::cout<<"丢弃本次框选的点云数据，将重新弹出本帧点云，请再次框选"<<std::endl;
        nextLable = false;
    }
    else
    {
        std::cout<<"所以呢？？？将重新弹出本帧点云，请再次框选"<<std::endl;
        nextLable = false;
    }

     finishLable = true;
}




int main(int argc, char **argv)
{

    ros::init(argc, argv, "select_pcd");
    ros::NodeHandle nh;
    ros::param::get("~suffix", filesuffix);

    for (int k=0; k<100; k++)
    {
        ros::param::get("~filename_pt", filename_pt);
        finishLable = false;
        std::stringstream ss;
        ss << k+1;
        num = ss.str();
        filename_pt.append(num);
        filename_pt.append(filesuffix);   // ==================================================!!!!!!!
        
        std::cout<<"Extracting "<<k<<" pointcloud from "<<  filename_pt<<std::endl;

        if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename_pt, *cloud_show) == -1)
            {
            //* load the file
            PCL_ERROR ("Couldn't read PCD file \n");
            return (-1);
            }
        
            pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Carviewer"));

        // 去除NAN!!!!!!这个一定要有，否则在viewer里面点云的索引和外部点云的索引并不一致
        cloud_show->is_dense = false;
        std::vector<int> indices_nonan;
        pcl::removeNaNFromPointCloud(*cloud_show,*cloud_show, indices_nonan);

        viewer->setBackgroundColor(0,0,0);
        viewer->addPointCloud(cloud_show,"forCalib");
        viewer->setCameraPosition(5, 5, 5, 0, 0, 0, 0);
        viewer->registerAreaPickingCallback(areapickingcallback, (void*)&cloud_show);
        // while (!viewer->wasStopped ())
        while (!finishLable)
        {
            viewer->spinOnce (100);
            // boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
        // thisLable = true;
        if (!nextLable)
        {
            k -= 1;
        }
        

    }


        

    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     savetotalpcd();
    // }


    ros::spin();

    return 0;
}

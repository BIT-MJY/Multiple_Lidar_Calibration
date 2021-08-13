#include <cmath>
#include <vector>
#include <string>
#include "multi_lidar_calib/common.h"
#include "multi_lidar_calib/tic_toc.h"
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
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 #include <ceres/ceres.h>
 #include "lidarFactor.hpp"
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sstream>
#include<vector>

#include<mutex>

#define PI 3.1415926

using std::atan2;
using std::cos;
using std::sin;

double lidar_z_height = -1;

ros::Publisher  pubFittedPlane;

double parameters[7] = {0, 0, 0, 1, 0, 0, 0}; // 激光雷达间相对位姿关系 

Eigen::Map<Eigen::Quaterniond> q_to_be_optimized(parameters);
Eigen::Map<Eigen::Vector3d> t_to_be_optimized(parameters + 4);


std::string extracted_path = " ";
int pcd_num_per_lidar = 0;
void planeFitting(pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud, pcl::ModelCoefficients::Ptr & coefficients)
{   

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
     seg.setInputCloud(extracted_cloud);
    seg.segment (*inliers, *coefficients);

    // // ax + by + cz + d = 0，其中法向量为(a, b, c)
    // std::cout<<"平面参数："<<std::endl;
    // std::cout<<"a："<<coefficients->values[0]<<std::endl;
    // std::cout<<"b："<<coefficients->values[1]<<std::endl;
    // std::cout<<"c："<<coefficients->values[2]<<std::endl;
    // std::cout<<"d："<<coefficients->values[3]<<std::endl;
    double a,b,c,d;
    a =   coefficients->values[0];
    b =  coefficients->values[1];
    c =  coefficients->values[2];
    d =  coefficients->values[3];
    // double numerator = fabs(  a*x_c+b*y_c+c*z_c+d  );
    // double denominator = std::sqrt(  a*a+b*b+c*c );
    // double distanceToArea = numerator/ denominator;
    pcl::PointCloud<pcl::PointXYZ>::Ptr fittedPlaneCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0; i<inliers->indices.size();i++)
    {
        int ind = inliers->indices[i];
         fittedPlaneCloud->points.push_back(  extracted_cloud->points[ind]  );
    }

    sensor_msgs::PointCloud2 fittedPlaneCloudMsg;
    pcl::toROSMsg(*fittedPlaneCloud, fittedPlaneCloudMsg);
    fittedPlaneCloudMsg.header.stamp = ros::Time::now();
    fittedPlaneCloudMsg.header.frame_id = "/rslidar";
    pubFittedPlane.publish(fittedPlaneCloudMsg);

}



pcl::PointCloud<pcl::PointXYZ>::Ptr planeFittingReturn(pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud, pcl::ModelCoefficients::Ptr & coefficients)
{   

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
     seg.setInputCloud(extracted_cloud);
    seg.segment (*inliers, *coefficients);

    // // ax + by + cz + d = 0，其中法向量为(a, b, c)
    // std::cout<<"平面参数："<<std::endl;
    // std::cout<<"a："<<coefficients->values[0]<<std::endl;
    // std::cout<<"b："<<coefficients->values[1]<<std::endl;
    // std::cout<<"c："<<coefficients->values[2]<<std::endl;
    // std::cout<<"d："<<coefficients->values[3]<<std::endl;
    double a,b,c,d;
    a =   coefficients->values[0];
    b =  coefficients->values[1];
    c =  coefficients->values[2];
    d =  coefficients->values[3];
    // double numerator = fabs(  a*x_c+b*y_c+c*z_c+d  );
    // double denominator = std::sqrt(  a*a+b*b+c*c );
    // double distanceToArea = numerator/ denominator;
    pcl::PointCloud<pcl::PointXYZ>::Ptr fittedPlaneCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0; i<inliers->indices.size();i++)
    {
        int ind = inliers->indices[i];
         fittedPlaneCloud->points.push_back(  extracted_cloud->points[ind]  );
    }

    sensor_msgs::PointCloud2 fittedPlaneCloudMsg;
    pcl::toROSMsg(*fittedPlaneCloud, fittedPlaneCloudMsg);
    fittedPlaneCloudMsg.header.stamp = ros::Time::now();
    fittedPlaneCloudMsg.header.frame_id = "/rslidar";
    pubFittedPlane.publish(fittedPlaneCloudMsg);

    return fittedPlaneCloud;

}




int main(int argc, char **argv)
{

    ros::init(argc, argv, "multi_lidar_calibration");
	ros::NodeHandle nh;

    ros::param::get("~extracted_path", extracted_path);
    ros::param::get("~pcd_num_per_lidar", pcd_num_per_lidar);

    pubFittedPlane = nh.advertise<sensor_msgs::PointCloud2>("/fitted_plane", 100);

    for (int iterCount = 0; iterCount < 10; iterCount++)
    {

            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::LocalParameterization *q_parameterization =
                new ceres::EigenQuaternionParameterization();
            ceres::Problem::Options problem_options;

            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(parameters, 4, q_parameterization);
            problem.AddParameterBlock(parameters + 4, 3);

            for (int k=0; k<pcd_num_per_lidar; k++)
            {

                std::stringstream ss;
                std::string filename_pt0 = extracted_path;
                std::string filename_pt1 = extracted_path;
                ss << k+1;
                std::string num = ss.str();
                filename_pt0.append(num);
                filename_pt0.append("_0.pcd");   
                filename_pt1.append(num);
                filename_pt1.append("_1.pcd");  
                
                std::cout<<"Extracting "<<k<<" pointcloud from "<<  filename_pt0<<std::endl;
                std::cout<<"Extracting "<<k<<" pointcloud from "<<  filename_pt1<<std::endl;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

                if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename_pt0, *cloud0) == -1)
                    {
                    PCL_ERROR ("Couldn't read PCD file \n");
                    }
                if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename_pt1, *cloud1) == -1)
                    {
                    PCL_ERROR ("Couldn't read PCD file \n");
                    }

                
                pcl::ModelCoefficients::Ptr coefficients0 (new pcl::ModelCoefficients);
                planeFitting(cloud0, coefficients0);
                pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients);
                planeFitting(cloud1,coefficients1);

                // 两平面法线的夹角作为一个损失值
                Eigen::Vector3d norm0(coefficients0->values[0], coefficients0->values[1], coefficients0->values[2]);
                Eigen::Vector3d norm1(coefficients1->values[0], coefficients1->values[1], coefficients1->values[2]);
                ceres::CostFunction *cost_function;
                cost_function = LidarNormFactor::Create(norm0, norm1);
                problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);




                TicToc t_solver;
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 10;
                options.minimizer_progress_to_stdout = false;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-4;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);


                sleep(0.2);

            }

    }
    printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],parameters[4], parameters[5], parameters[6]);
  // ====================================================== //=========================================================
  // 接下来标平移：（之所以分开标定是因为如果平移不准，起码保证旋转是准的，平移不会影响旋转。实验发现，这样标定的话平移也挺准）
    for (int iterCount = 0; iterCount < 10; iterCount++)
    {

            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(parameters + 4, 3);

            for (int k=0; k<pcd_num_per_lidar; k++)
            {

                std::stringstream ss;
                std::string filename_pt0 = "/media/mjy/Samsung_T5/linux/DX/data/0812forcalib/pcd_select/";
                std::string filename_pt1 = "/media/mjy/Samsung_T5/linux/DX/data/0812forcalib/pcd_select/";
                ss << k+1;
                std::string num = ss.str();
                filename_pt0.append(num);
                filename_pt0.append("_0.pcd");   
                filename_pt1.append(num);
                filename_pt1.append("_1.pcd");  
                
                std::cout<<"Extracting "<<k<<" pointcloud from "<<  filename_pt0<<std::endl;
                std::cout<<"Extracting "<<k<<" pointcloud from "<<  filename_pt1<<std::endl;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0_plane(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_plane(new pcl::PointCloud<pcl::PointXYZ>);

                if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename_pt0, *cloud0) == -1)
                    {
                    PCL_ERROR ("Couldn't read PCD file \n");
                    }
                if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename_pt1, *cloud1) == -1)
                    {
                    PCL_ERROR ("Couldn't read PCD file \n");
                    }

                
                pcl::ModelCoefficients::Ptr coefficients0 (new pcl::ModelCoefficients);
                cloud0_plane = planeFittingReturn(cloud0, coefficients0);
                pcl::ModelCoefficients::Ptr coefficients1 (new pcl::ModelCoefficients);
                cloud1_plane = planeFittingReturn(cloud1,coefficients1);

                // 一共有三维损失值，分别代表：。。。。。。先加一维
                // 0：质心距离
                // 计算质心
                double center_x0 = 0;
                double center_y0 = 0;
                double center_z0 = 0;
                double center_x1 = 0;
                double center_y1 = 0;
                double center_z1 = 0;
                for (int i=0; i<cloud0_plane->points.size(); i++)
                {
                    center_x0 += cloud0_plane->points[i].x;
                    center_y0 += cloud0_plane->points[i].y;
                    center_z0 += cloud0_plane->points[i].z;
                }
                center_x0 /= cloud0_plane->points.size();
                center_y0 /= cloud0_plane->points.size();
                center_z0 /= cloud0_plane->points.size();


                for (int i=0; i<cloud1_plane->points.size(); i++)
                {
                    // 先将点用旋转转过去：
                    Eigen::Vector3d tmppoint(cloud1_plane->points[i].x,  cloud1_plane->points[i].y, cloud1_plane->points[i].z);
                    Eigen::Vector3d tmppoint_roted =  q_to_be_optimized.matrix() * tmppoint;
                    center_x1 += tmppoint_roted.x();
                    center_y1 +=tmppoint_roted.y();
                    center_z1 += tmppoint_roted.z();
                }
                center_x1 /= cloud1_plane->points.size();
                center_y1 /= cloud1_plane->points.size();
                center_z1 /= cloud1_plane->points.size();


                Eigen::Vector3d plane_center0(center_x0, center_y0, center_z0) ;
                Eigen::Vector3d plane_center1(center_x1, center_y1, center_z1) ;

                ceres::CostFunction *cost_function;
                cost_function = LidarCenterFactor::Create(plane_center0, plane_center1);
                problem.AddResidualBlock(cost_function, loss_function, parameters + 4);


                TicToc t_solver;
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 10;
                options.minimizer_progress_to_stdout = false;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-4;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);


                sleep(0.2);

            }

    }



    std::cout<<"======================最终标定结果======================"<<std::endl;
    printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],parameters[4], parameters[5], parameters[6]);
    std::cout<<q_to_be_optimized.matrix()<<std::endl;

    return 0;
}

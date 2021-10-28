# Multiple_Lidar_Calibration
Here is the source code for calibration between lidars. (Chinese Version)


# 双激光雷达联合标定

## 依赖库
* PCL 1.8
* OpenCV 3.2
* Ceres 1.14

## 数据准备
准备一个rosbag包，其中包含主雷达0、副雷达1的激光话题。激光数据包含**矩形标定板在远中近三个距离下，绕X-Y-Z轴分别正负方向倾斜一定角度的激光点云**。


## 使用方式
### 克隆及编译
```
mkdir Multiple_Lidar_Calibration
cd Multiple_Lidar_Calibration/
mkdir src
cd src/
git clone https://github.com/BIT-MJY/Multiple_Lidar_Calibration.git
cd ..
catkin_make
```
### 运行
首先在rosbag包中提取标定板上的激光点云为pcd。  
（1）修改Multiple_Lidar_Calibration/save_pcd/launch/save_pcd.launch文件中的```shot_save_```参数为全点云pcd保存目标位置。  
（2）修改Multiple_Lidar_Calibration/save_pcd/launch/save_pcd.launch文件中的激光映射。  
```
     <remap from="/rslidar_points_16" to="/rslidar_points_16" />
     <remap from="/rslidar_points_32" to="/rslidar_points_32" />
```
（3）运行下方命令，聚焦于弹出来的黑色方框，点击键盘“S”键，对激光点进行保存。注意，要保证远中近三个距离下（3），绕X-Y-Z轴分别正负方向倾斜一定角度（3x2=6)，都保存到，每个要求保存两次，即为3x6x2=36张pcd文件。  
```
rosbag play your_bag_containing_2_lidar.bag
roslaunch save_pcd save_pcd.launch 
```
在```shot_save_```文件夹下可以看到后缀为0的主雷达（在这里对应rslidar_points_16）和后缀为1的副雷达（在这里对应rslidar_points_32）的全点云pcd。  
（4）修改Multiple_Lidar_Calibration/save_pcd/launch/select_pcd.launch文件中的```filename_pt_source```与上面```shot_save_```一致。  
（5）修改Multiple_Lidar_Calibration/save_pcd/launch/select_pcd.launch文件中的```filename_pt_selected_det```为标定板点云pcd保存目标位置。  
（6）修改Multiple_Lidar_Calibration/save_pcd/launch/select_pcd.launch文件中的```suffix_```，```"_0.pcd"```表示主雷达，```"_1.pcd"```表示副雷达，这里先设置为```"_0.pcd"```。  
（7）运行下方命令，聚焦于弹出来的显示点云的黑色窗口，按键盘X键后，框选只属于标定板区域的点云，在终端弹出的提示中选择是否保存。若选择不保存，则会重新弹出这一帧点云。  
```
roslaunch save_pcd select_pcd.launch 
```
（8）循环结束后，将Multiple_Lidar_Calibration/save_pcd/launch/select_pcd.launch文件中的```suffix_```设置为```"_1.pcd"```，再次运行  
```
roslaunch save_pcd select_pcd.launch 
```
循环结束后，发现框选的所有点云均以主副雷达后缀区分的形式保存于```filename_pt_selected_det```文件夹中。  
（9）修改Multiple_Lidar_Calibration/multi_lidar_calib/launch/multi_lidar_calib.launch文件中的```extracted_path_```与```filename_pt_selected_det```一致。  
（10）修改Multiple_Lidar_Calibration/multi_lidar_calib/launch/multi_lidar_calib.launch文件中的```pcd_num_per_lidar_```为每个雷达对应的pcd文件个数（per lidar）。  
（11）运行下方命令，开展基于CERES的优化，获得标定结果。  
```
roslaunch multi_lidar_calib multi_lidar_calib.launch 
```

<img src="https://github.com/BIT-MJY/Multiple_Lidar_Calibration/blob/main/calib_results.png">


## 修改
下个版本准备加入ICP细化，详见分支[withICP](https://github.com/BIT-MJY/Multiple_Lidar_Calibration/tree/withICP)。


## 作者
北京理工大学智能车辆研究所，Junyi Ma  

多雷达数据，contact：mjy980625@163.com



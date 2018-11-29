#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>
// #include<PointCloudMapping.h>
// #include<System.h>

// class System;
// class PointCloudMapping;

int main (int argc, char **argv)  
{  
  ros::init (argc, argv, "orbslam");  
  
  ros::NodeHandle nh;  
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/orbslam2/output/pointcloud", 10);  

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;  
  pcl::PointCloud<pcl::PointXYZRGB> cloud;  
  sensor_msgs::PointCloud2 output;
  // cloud = ORB_SLAM2::System.mpPointCloudMapper->get_globalPointCloudMap();  
  // pcl::io::loadPCDFile ("/home/why/SLAM/demo/orbslam2_ws/src/ORB_SLAM2/PointCloud.pcd", cloud);  
  // pcl::toROSMsg(cloud,output);// transfer to ROS data type

  // output.header.stamp=ros::Time::now();
  // output.header.frame_id  ="camera_rgb_frame";

  ros::Rate loop_rate(1);  
  while (ros::ok())  
  {  
    pcl::io::loadPCDFile ("/home/nvidia/SLAM/project/slam_ws/PointCloud.pcd", cloud);  
    pcl::toROSMsg(cloud,output);// transfer to ROS data type
  
    output.header.stamp=ros::Time::now();
    output.header.frame_id  ="zed_center";

    pcl_pub.publish(output);  
    ros::spinOnce();  
    loop_rate.sleep();  
  }  
  return 0;  
}  
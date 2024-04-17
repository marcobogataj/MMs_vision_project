#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "voxel_filtering_n");
  ros::NodeHandle nh;
  
  ros::Rate loop_rate(0.1);

  // Create a ROS publisher for the output point cloud
  ros::Publisher pub_out = nh.advertise<sensor_msgs::PointCloud2> ("filter_output_t", 1);
  ros::Publisher pub_in = nh.advertise<sensor_msgs::PointCloud2> ("filter_input_t", 1);
  
  //initialize Pount cloud objects
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2 cloud_filtered;
    
  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file 
  reader.read ("/home/abb/Documents/zivid_captures/zivid_manual_holefilling.pcd", *cloud); 
  
  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (cloud_filtered);
  
  // Convert from PCD to ROS msg data type
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 input;
 
  while (ros::ok())
  {
     pcl_conversions::fromPCL(cloud_filtered, output);
     pcl_conversions::fromPCL(*cloud, input);

     input.header.frame_id="pcl_tf";
     input.header.stamp = ros::Time::now();
     pub_in.publish(input); //input message*/

     output.header.frame_id="pcl_tf";
     output.header.stamp = ros::Time::now();
     pub_out.publish(output); //filtered message

     ros::spinOnce();
     loop_rate.sleep();
  }

  return 0;
}

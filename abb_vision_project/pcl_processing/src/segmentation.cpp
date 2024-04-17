#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

/*Node for .pcd offline file reading, point cloud SEGMENTATION and 
publishing on different ROS topics for Rviz visualisation*/

int
main (int argc, char** argv)
{
// Initialize ROS
ros::init (argc, argv, "segmentation_n");
ros::NodeHandle nh;
  
ros::Rate loop_rate(1);

// Create a ROS publisher for the output point cloud
ros::Publisher pub_out = nh.advertise<sensor_msgs::PointCloud2> ("filter_output_t", 1);
ros::Publisher pub_in = nh.advertise<sensor_msgs::PointCloud2> ("filter_input_t", 1);
  
//initialize Pount cloud objects
pcl::PCLPointCloud2::Ptr cloud_in (new pcl::PCLPointCloud2 ());
pcl::PCLPointCloud2 cloud_out;
    
// Fill in the cloud data
pcl::PCDReader reader;

// Replace the path below with the path where you saved your file
reader.read ("/home/abb/Documents/zivid_captures/zivid_amb60hz_auto_whitebox+++.pcd", *cloud_in); 
  
/* PROCESSING */

//FILTERING
// Create the filtering object

//SEGMENTATION



  
// Convert from PCD to ROS msg data type
sensor_msgs::PointCloud2 cloud_out_msg;
sensor_msgs::PointCloud2 cloud_in_msg;
 
while (ros::ok())
{
    pcl_conversions::fromPCL(cloud_out, cloud_out_msg);
    pcl_conversions::fromPCL(*cloud_in, cloud_in_msg);

    input.header.frame_id="pcl_tf";
    input.header.stamp = ros::Time::now();
    pub_in.publish(cloud_in_msg); //non-processed input point cloud message

    output.header.frame_id="pcl_tf";
    output.header.stamp = ros::Time::now();
    pub_out.publish(cloud_out_msg); //processed output point cloud message

    ros::spinOnce();
    loop_rate.sleep();
}

return 0;
}

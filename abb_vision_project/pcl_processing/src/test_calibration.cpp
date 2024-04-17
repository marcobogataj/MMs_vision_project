#include <ros/ros.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/common/time.h>

//specific processing libraries
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>

ros::Subscriber sub;
ros::Publisher pub;

typedef pcl::PointXYZRGBA PointT;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{ 
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*input, *cloud);

  ROS_INFO("Point cloud received!");
  printf ("Input cloud: width = %d, height = %d\n", cloud->width, cloud->height);

  pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT> ());

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Define a rotation (see https://en.wikipedia.org/wiki/Rotation_matrix)
  transform (0,0) = 0.968074;
  transform (0,1) = 0.000779;
  transform (0,2) = 0.250665;

  transform (1,0) = -0.065838;
  transform (1,1) = -0.964096;
  transform (1,2) = 0.257262;

  transform (2,0) = 0.241866;
  transform (2,1) = -0.265552;
  transform (2,2) = -0.933265;

  // Define a translation 
  transform (0,3) = 0.037611496;
  transform (1,3) = -0.595347046;
  transform (2,3) = 0.677256409;

  // Print the transformation
  printf ("Point Cloud calibration: using a Matrix4f\n");
 
  // Executing the transformation
  pcl::transformPointCloud(*cloud, *output, transform);

  pub.publish(*output); //send output message

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle n;

  // capture()

  // Create a ROS subscriber for the input point cloud
  auto sub = n.subscribe("/zivid_camera/points/xyzrgba", 1, cloud_callback);

  // Create a ROS publisher for the output point cloud
  pub = n.advertise<pcl::PointCloud<PointT>>("/pcl_processing/filtered/xyzrgba", 1);

  // processing()

  // Spin
  ros::spin ();
}

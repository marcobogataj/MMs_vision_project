#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#define CHECK(cmd)                                                                                                     \
do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

void normalsCB(const pcl::PointCloud<pcl::Normal>::ConstPtr& cloud_normals)
{
    ROS_INFO("Normals received!");
    std::cout<<"Normal data size"<< cloud_normals->width*cloud_normals->height;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "test_sub_normals");
  ros::NodeHandle nh_;

  ros::Subscriber normal_subscriber_;

  // Create a ROS subscriber for the input point cloud
  normal_subscriber_ = nh_.subscribe<pcl::PointCloud<pcl::Normal>>("/zivid_camera/normals/xyz", 1, normalsCB);

  // Spin
  ros::spin();
}


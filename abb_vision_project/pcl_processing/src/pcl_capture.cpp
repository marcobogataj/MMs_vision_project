#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

const ros::Duration default_wait_duration{ 30 };

#define CHECK(cmd)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pcl_capture");

  CHECK(ros::service::waitForService("/zivid_capture/zivid_capture_suggested",default_wait_duration));

  ROS_INFO("Ready to capture & segment");
  
  while(ros::ok)
  {
    ROS_INFO("Type c to capture. Anything else to shutdown");
    std::string inputString;
    std::getline(std::cin, inputString);

    if(inputString == "c"){
      auto result = system("rosservice call /zivid_capture/zivid_capture_suggested");
    }
    else{
      ros::shutdown();
    }
  }

  // Spin
  ros::spin();
}


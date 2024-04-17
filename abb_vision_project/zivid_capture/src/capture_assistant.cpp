#include <zivid_camera/Capture.h>
#include <zivid_camera/SettingsAcquisitionConfig.h>
#include <zivid_camera/SettingsConfig.h>
#include <zivid_camera/CaptureAssistantSuggestSettings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

//include service header
#include <zivid_capture/ZividCaptureSuggested.h>

//to log
#include <fstream>


#define CHECK(cmd)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

int countAcquisitions = 0;

void write_timeData_to_log_file(const std::vector<double> &timeVector)
{   
    std::string timeStr;
    std::ofstream log_file("/home/abb/catkin_ws/src/abb_vision_project/zivid_capture/log/CaptureTimeDataLog.txt", std::ios_base::out | std::ios_base::app );
    log_file << countAcquisitions;
    for(int i=0; i<timeVector.size(); i++)
    {
    //timeStr = removeTrailingZerosFromDecimal(rounded2string(timeVector[i]));
    timeStr = std::to_string(round(timeVector[i]*1000) / 1000);

    //remove three zeroes
    timeStr.pop_back();
    timeStr.pop_back();
    timeStr.pop_back();

    log_file << "," << timeStr;
    }
    log_file << std::endl;
    countAcquisitions++;
}

std::vector<double> timeData(1);

namespace
{
const ros::Duration default_wait_duration{ 30 };
constexpr auto ca_suggest_settings_service_name = "/zivid_camera/capture_assistant/suggest_settings";

void capture_assistant_suggest_settings()
{
  zivid_camera::CaptureAssistantSuggestSettings cass;
  cass.request.max_capture_time = ros::Duration{ 1.20 };
  cass.request.ambient_light_frequency =
      zivid_camera::CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_NONE;

  ROS_INFO_STREAM("Calling " << ca_suggest_settings_service_name
                             << " with max capture time = " << cass.request.max_capture_time << " sec");
  CHECK(ros::service::call(ca_suggest_settings_service_name, cass));
}

void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}

}  // namespace

bool execute_capture(zivid_capture::ZividCaptureSuggested::Request  &req,
                     zivid_capture::ZividCaptureSuggested::Response &res)
{
  double init_secs = ros::Time::now().toSec();

  ROS_INFO("Capture...");

  capture();

  ROS_INFO("Capture...OK");

  timeData[0] = ros::Time::now().toSec() - init_secs; //get suggested settings

  write_timeData_to_log_file(timeData);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "capture_assistant");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("zivid_capture/zivid_capture_suggested", execute_capture);
  
  CHECK(ros::service::waitForService(ca_suggest_settings_service_name, default_wait_duration));

  ROS_INFO("Get suggested settings...");

  capture_assistant_suggest_settings();

  
  ROS_INFO("Set region of interest...");
  system("rosrun dynamic_reconfigure dynparam set /zivid_camera/settings region_of_interest_box_enabled true");
  system("rosrun dynamic_reconfigure dynparam set /zivid_camera/settings region_of_interest_box_extents_min -10");
  system("rosrun dynamic_reconfigure dynparam set /zivid_camera/settings region_of_interest_box_extents_max 80");
  system("rosrun dynamic_reconfigure dynparam set /zivid_camera/settings region_of_interest_box_point_a_x 144.04747");
  system("rosrun dynamic_reconfigure dynparam set /zivid_camera/settings region_of_interest_box_point_a_y 70.5990982");
  system("rosrun dynamic_reconfigure dynparam set /zivid_camera/settings region_of_interest_box_point_a_z 477.182098");
  system("rosrun dynamic_reconfigure dynparam set /zivid_camera/settings region_of_interest_box_point_b_x -151.751099");
  system("rosrun dynamic_reconfigure dynparam set /zivid_camera/settings region_of_interest_box_point_b_y -101.680817");
  system("rosrun dynamic_reconfigure dynparam set /zivid_camera/settings region_of_interest_box_point_b_z 450.587646");
  system("rosrun dynamic_reconfigure dynparam set /zivid_camera/settings region_of_interest_box_point_o_x -128.985992");
  system("rosrun dynamic_reconfigure dynparam set /zivid_camera/settings region_of_interest_box_point_o_y 70.7012939");
  system("rosrun dynamic_reconfigure dynparam set /zivid_camera/settings region_of_interest_box_point_o_z 405.896667");
  
  
  
  ROS_INFO("Ready to capture with suggested settings.");

  ros::spin();

  return 0;
}

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "zivid_optical_frame";
  transformStamped.transform.translation.x = 0.037611496;
  transformStamped.transform.translation.y = -0.595347046;
  transformStamped.transform.translation.z = 0.677256409;
  
  transformStamped.transform.rotation.x = 0.9830354;
  transformStamped.transform.rotation.y = -0.0165454;
  transformStamped.transform.rotation.z = 0.1252577;
  transformStamped.transform.rotation.w = -0.1329591;

  ros::Rate rate(10.0);
  while (node.ok()){
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    rate.sleep();
    printf("sending\n");
  }

};
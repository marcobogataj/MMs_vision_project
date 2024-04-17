#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(0.5);

  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("world", "tool0",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    double x,y,z, qw,qx,qy,qz;
    x = transformStamped.transform.translation.x;
    y = transformStamped.transform.translation.y;
    z = transformStamped.transform.translation.z;

    qx = transformStamped.transform.rotation.x;
    qy = transformStamped.transform.rotation.y;
    qz = transformStamped.transform.rotation.z;
    qw = transformStamped.transform.rotation.w;
   
    double r00,r01,r02,r10,r11,r12,r20,r21,r22;

    //First row of the rotation matrix
    r00 = 2 * (qw * qw + qx * qx) - 1;
    r01 = 2 * (qx * qy - qw * qz);
    r02 = 2 * (qx * qz + qw * qy);
     
    //Second row of the rotation matrix
    r10 = 2 * (qx * qy + qw * qz);
    r11 = 2 * (qw * qw + qy * qy) - 1;
    r12 = 2 * (qy * qz - qw * qx);
     
    //Third row of the rotation matrix
    r20 = 2 * (qx * qz - qw * qy);
    r21 = 2 * (qy * qz + qw * qx);
    r22 = 2 * (qw * qw + qz * qz) - 1;

    std::cout <<"TF: "<<r00<<" "<<r01<<" "<<r02<<" "<<x<<" "
                      <<r10<<" "<<r11<<" "<<r12<<" "<<y<<" "
                      <<r20<<" "<<r21<<" "<<r22<<" "<<z<<" "
                      <<0  <<" "<<0  <<" "<<0  <<" "<<1<<std::endl;

    rate.sleep();
  }
  return 0;
};
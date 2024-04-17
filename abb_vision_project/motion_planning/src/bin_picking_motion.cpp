/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>

//Eigen
#include <Eigen/Geometry>
#include <Eigen/Core>

const ros::Duration default_wait_duration{ 30 };

#define CHECK(cmd)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

std::vector<double> marker2vector(visualization_msgs::MarkerArray::ConstPtr& cylinder_marker)
{
    std::vector<double> v;
    v.resize(9);

    v[0]=cylinder_marker->markers[0].pose.position.x;
    v[1]=cylinder_marker->markers[0].pose.position.y;
    v[2]=cylinder_marker->markers[0].pose.position.z;

    v[3]=cylinder_marker->markers[0].pose.orientation.x;
    v[4]=cylinder_marker->markers[0].pose.orientation.y;
    v[5]=cylinder_marker->markers[0].pose.orientation.z;
    v[6]=cylinder_marker->markers[0].pose.orientation.w;
    
    v[7]=cylinder_marker->markers[0].scale.x; //diameter
    v[8]=cylinder_marker->markers[0].scale.z; //height

    return v;
}

double GripperApertureConversion(double aperture_in_mm)
{
  double joint_value;
  //min: 0rad->0.085m, max: 0.8rad->0m
  joint_value = (0.085 - aperture_in_mm)/0.085 * 0.8; 
  return joint_value;
}

geometry_msgs::Pose computeCylinderGrasp(visualization_msgs::MarkerArray::ConstPtr& cylinder_marker, double H, double D)
{
  geometry_msgs::Pose targetPose = cylinder_marker->markers[0].pose;
  tf2::Transform cylinderPose;
  tf2::fromMsg(targetPose,cylinderPose);
  tf2::Vector3 origin_x_axis(1,0,0), origin_y_axis(0,1,0), origin_z_axis(0,0,1);
  tf2::Vector3 tool0_x_axis, tool0_y_axis, tool0_z_axis;
  tf2::Vector3 cylinder_z_axis = cylinderPose.getBasis() * origin_z_axis;

  //TO DO: choose mode of grasping: vertical or horizontal
  std::string mode;

  //projection value used to choose between vertical and horizontal mode
  if (cylinder_z_axis.dot(origin_z_axis) >= 0){
      cylinder_z_axis = - cylinder_z_axis;
  }
    
  double pzvalue = cylinder_z_axis.dot(origin_z_axis) / (cylinder_z_axis.length() * origin_z_axis.length()) ; //value between -1 and 1
  double pyvalue = cylinder_z_axis.dot(origin_y_axis) / (cylinder_z_axis.length() * origin_y_axis.length()) ; //value between -1 and 1

  std::cout<<"abs pzvalue ="<<abs(pzvalue)<<std::endl;
  std::cout<<"pyvalue ="<<pyvalue<<std::endl;

  if (   (abs(pzvalue) > 0.97 && pyvalue > 0) //--> cylinder oriented opposite to robot
      || (abs(pzvalue) > 0.50 && pyvalue < 0)) //--> cylinder oriented to the robot
  { 
    mode = "vertical";
  }
  else 
  {
    mode = "horizontal";
  } 

  if (mode == "vertical")
  {
    ROS_INFO("Compute grasping in vertical mode...");
    tool0_z_axis = cylinder_z_axis; 

    tf2::Vector3 rob_to_cylinder(targetPose.position.x,targetPose.position.y,targetPose.position.z);

    //Get orthogonal component of rob_to_cylinder with respect to cylinder_z_axis
    //see https://en.wikipedia.org/wiki/Vector_projection
    tool0_x_axis = (rob_to_cylinder - rob_to_cylinder.dot(tool0_z_axis) * tool0_z_axis) * -1; 
    tool0_y_axis = tool0_x_axis.cross(tool0_z_axis) * -1;

    tool0_z_axis.normalize();

    targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(+0.1628-H/3); 
    targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(+0.1628-H/3);
    targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(+0.1628-H/3);

  }
  else
  {
    ROS_INFO("Compute grasping in horizontal mode...");
    
    if (cylinder_z_axis.dot(origin_y_axis) <= 0){
      cylinder_z_axis = - cylinder_z_axis;
    }

    tool0_x_axis = cylinder_z_axis; 

    tf2::Vector3 rob_to_cylinder(targetPose.position.x,targetPose.position.y,targetPose.position.z);

    //Get orthogonal component of rob_to_cylinder with respect to cylinder_z_axis
    //see https://en.wikipedia.org/wiki/Vector_projection
    tool0_y_axis = tool0_x_axis.cross(origin_z_axis);
    
    if (tool0_y_axis.dot(origin_x_axis) <= 0){
      tool0_y_axis = - tool0_y_axis;
    }

    tool0_z_axis = tool0_x_axis.cross(tool0_y_axis);

    tool0_z_axis.normalize();

    //change D with H for smaller diameter long height objects
    targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(+0.1628-D/4); 
    targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(+0.1628-D/4);
    targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(+0.1628-D/4);

  }

  //obtain quaternions for pose definiton
  tool0_z_axis.normalize();
  tool0_x_axis.normalize();
  tool0_y_axis.normalize();

  Eigen::Matrix3f tool0_basis;
  tool0_basis <<tool0_x_axis.getX(), tool0_y_axis.getX(), tool0_z_axis.getX(),
                tool0_x_axis.getY(), tool0_y_axis.getY(), tool0_z_axis.getY(),
                tool0_x_axis.getZ(), tool0_y_axis.getZ(), tool0_z_axis.getZ();

  Eigen::Quaternionf q(tool0_basis);

  targetPose.orientation.x = q.x();
  targetPose.orientation.y = q.y();
  targetPose.orientation.z = q.z();
  targetPose.orientation.w = q.w();

  return targetPose;
}

void stampPosition(geometry_msgs::Pose Pose)
{
  static tf2_ros::StaticTransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "grasping_pose";
  transformStamped.transform.translation.x = Pose.position.x;
  transformStamped.transform.translation.y = Pose.position.y;
  transformStamped.transform.translation.z = Pose.position.z;

  transformStamped.transform.rotation.x = Pose.orientation.x;
  transformStamped.transform.rotation.y = Pose.orientation.y;
  transformStamped.transform.rotation.z = Pose.orientation.z;
  transformStamped.transform.rotation.w = Pose.orientation.w;

  transformStamped.header.stamp = ros::Time::now();
  tfb.sendTransform(transformStamped);
  ros::spinOnce;
} 

void openGripper(trajectory_msgs::JointTrajectory& posture, double D)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(1);
  posture.joint_names[0] = "robotiq_85_left_knuckle_joint";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] =  GripperApertureConversion(D+0.006);
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture, double D)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(1);
  posture.joint_names[0] = "robotiq_85_left_knuckle_joint";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = GripperApertureConversion(D-0.002);
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group, 
          visualization_msgs::MarkerArray::ConstPtr& cylinder_marker, 
          double H, double D)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Allow collisions of panda_hand_sc with object
  grasps[0].allowed_touch_objects.push_back("robotiq_85_right_finger_tip_link");

  // Setting grasp pose

  grasps[0].grasp_pose.header.frame_id = "base_link";
  grasps[0].grasp_pose.pose = computeCylinderGrasp(cylinder_marker,H,D);
  stampPosition(grasps[0].grasp_pose.pose);

  // Setting pre-grasp approach
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
  /* Direction is set as positive x axis */
  //grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.04;
  grasps[0].pre_grasp_approach.desired_distance = 0.05;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.04;
  grasps[0].post_grasp_retreat.desired_distance = 0.05;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture, D);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture, D);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  move_group.setSupportSurfaceName("box");
  // Call pick to pick up the object using the grasps given
  move_group.pick("cylinder", grasps);
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in
  // verbose mode." This is a known issue and we are working on fixing it. |br|
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "base_link";
  place_location[0].place_pose.pose.orientation.x = 0;
  place_location[0].place_pose.pose.orientation.y = 1;
  place_location[0].place_pose.pose.orientation.y = 0; 
  place_location[0].place_pose.pose.orientation.y = -0.0047801;

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0.55213;
  place_location[0].place_pose.pose.position.y = 0;
  place_location[0].place_pose.pose.position.z = 0.3244;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.10;
  place_location[0].pre_place_approach.desired_distance = 0.11;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.10;
  place_location[0].post_place_retreat.desired_distance = 0.11;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */ //set open gripper to an object of diameter 15cm to be sure 
  openGripper(place_location[0].post_place_posture, 0.015);

  // Set support surface as box2.
  //group.setSupportSurfaceName("box2");
  // Call place to place the object using the place locations given.
  group.place("cylinder", place_location);
  // END_SUB_TUTORIAL
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bin_picking_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("irb_120");
  group.setPlanningTime(45.0);

  //Cylinder parameters
  visualization_msgs::MarkerArray::ConstPtr marker_array_msg(new visualization_msgs::MarkerArray);
  std::vector<double> v; //cylindwe params. [x,y,z,qx,qy,qz,qw,diameter,height]

  // Moveit monitor
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

  CHECK(ros::service::waitForService("/zivid_capture/zivid_capture_suggested",default_wait_duration));

  ROS_INFO("Ready to capture & pick");

  ROS_INFO("visualizing plan %s", success.val ? "":"FAILED");

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  while(ros::ok())
  {
    std::string inputString;
    ROS_INFO("Type p to pick cylinder. Anything else to shutdown");
    std::getline(std::cin, inputString);

    if(inputString == "p"){
      ROS_INFO("Start camera acquisition...");
      auto result = system("rosservice call /zivid_capture/zivid_capture_suggested");
    }
    else{
      ros::shutdown();
    }

    ROS_INFO("Wait for cylinder identification...");

    marker_array_msg = ros::topic::waitForMessage<visualization_msgs::MarkerArray>("visualization_marker_array",nh);
    v = marker2vector(marker_array_msg); //get cylinder marker parameters in a vector
    double D = v[7];
    double H = v[8];

    if (D != 0)
    {
      ROS_INFO("Cylinder idientified!");

      v = marker2vector(marker_array_msg); //get cylinder marker parameters in a vector

      std::cout<<"D: "<<D<<"mm "<<"H: "<<H<<"mm "<<std::endl;
      std::cout<<"Position [x,y,z]=["<<v[0]<<","<<v[1]<<","<<v[2]<<"]"<<std::endl;
      std::cout<<"Orientation [qx,qy,qz,qw]="<<v[3]<<","<<v[4]<<","<<v[5]<<","<<v[6]<<"]"<<std::endl;

      ROS_INFO("PICKING START!");

      pick(group, marker_array_msg,H,D);

      ros::WallDuration(1.0).sleep();

      place(group);
    }
    else{
      ROS_ERROR("No cylinder identified.");
    }
  }

  ros::waitForShutdown();
  return 0;
}


#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <fstream>

const ros::Duration default_wait_duration{ 30 };

#define CHECK(cmd)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

int countAcquisitions = 1;

void write_timeData_to_log_file(const std::vector<double> &timeVector)
{   
    std::string timeStr;
    std::ofstream log_file("/home/abb/catkin_ws/src/abb_vision_project/motion_planning/log/PickingTimeDataLog.txt", std::ios_base::out | std::ios_base::app );
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

std::vector<double> timeData(5);

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
  //in realtà i break pads tolgono 2mm per lato -> totale 4mm di perdita in apertura. 
  //Si potrebbe fare aperture_in_mm = aperture_in_mm + 4mm per ottenere l'apertura corretta
  joint_value = (0.085 - aperture_in_mm)/0.085 * 0.8; 
  return joint_value;
}

double GripperAngleConversion(double angle_in_radians)
{
  double aperture;
  //min: 0.7189rad->0.008mm, max: 0.612rad->0.0115mm 
  //aperture = (0.7189 - angle_in_radians)*0.0035/0.1069 + 0.008; //old interpolation
  
  //beta0 = 0.7826 beta1=-7.7740 cool interpolation
  double beta0 = 0.7826;
  double beta1=-7.7740;
  aperture = (angle_in_radians - beta0)/beta1; 
  return aperture;
}

struct grasp_pose {
    geometry_msgs::Pose grasp_targetPose;
    geometry_msgs::Pose pregrasp_targetPose;
};

grasp_pose computeCylinderGrasp(visualization_msgs::MarkerArray::ConstPtr& cylinder_marker, double H, double D)
{
  grasp_pose grasp;
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
      || (abs(pzvalue) > 0.87 && pyvalue <= 0)) //--> cylinder oriented to the robot
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

    grasp.grasp_targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(+0.1628+H/2); 
    grasp.grasp_targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(+0.1628+H/2);
    grasp.grasp_targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(+0.1628+H/2);

    grasp.pregrasp_targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(0.1628+H+0.03); //3cm as offset
    grasp.pregrasp_targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(0.1628+H+0.03);
    grasp.pregrasp_targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(0.1628+H+0.03);
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

    //change D with H for smaller diameter long height objects (+0.005 for offset, old was 0.006)
    grasp.grasp_targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(+0.1628-D/4+0.005); 
    grasp.grasp_targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(+0.1628-D/4+0.005);
    grasp.grasp_targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(+0.1628-D/4+0.005);

    //8cm above target (old was 4cm)
    grasp.pregrasp_targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(0.1493+0.06); //substitute 0.04 with D 
    grasp.pregrasp_targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(0.1493+0.06);
    grasp.pregrasp_targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(0.1493+0.06);
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

  //save to struct grasp_pose 
  grasp.grasp_targetPose.orientation = targetPose.orientation;
  grasp.pregrasp_targetPose.orientation = targetPose.orientation;

  return grasp;
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

bool checkArmGoal(geometry_msgs::Pose arm_pose, geometry_msgs::Pose arm_goal, double tolerance)
{
  double r_tolerance2 = pow(tolerance,2);
  double dx2 = pow(arm_goal.position.x - arm_pose.position.x,2);
  double dy2 = pow(arm_goal.position.y - arm_pose.position.y,2);
  double dz2 = pow(arm_goal.position.z - arm_pose.position.z,2);

  /*
  tf2::Quaternion q_pose(
        arm_pose.orientation.x,
        arm_pose.orientation.y,
        arm_pose.orientation.z,
        arm_pose.orientation.w);

  tf2::Matrix3x3 m_pose(q_pose);
  double roll_pose, pitch_pose, yaw_pose;
  m_pose.getRPY(roll_pose, pitch_pose, yaw_pose);

  tf2::Quaternion q_goal(
        arm_goal.orientation.x,
        arm_goal.orientation.y,
        arm_goal.orientation.z,
        arm_goal.orientation.w);

  tf2::Matrix3x3 m_goal(q_goal);
  double roll_goal, pitch_goal, yaw_goal;
  m_goal.getRPY(roll_goal, pitch_goal, yaw_goal);

  double droll = abs(roll_goal - roll_pose);
  double dpitch = abs(pitch_goal - pitch_pose);
  double dyaw = abs(yaw_goal - yaw_pose);

  std::cout<<"radius_error = "<<dx2+dy2+dz2;
  std::cout<<"dpitch = "<<dpitch;
  std::cout<<"droll = "<<droll;
  std::cout<<"dyaw = "<<dyaw;
  */

  std::cout<<"radius_error = "<<dx2+dy2+dz2;

  if ( (dx2 + dy2 + dz2) < r_tolerance2) 
  {
    ROS_INFO("Arm's goal reached!");
    return true;
  }
  return false;
  
}

/*
bool checkGripperGoal(std::vector<double> gripper_joint_state, double gripper_joint_goal, double tolerance)
{
  std::cout<<"Abs joint error = " <<abs(gripper_joint_goal - gripper_joint_state[0])<<std::endl;
  if (abs(gripper_joint_goal - gripper_joint_state[0]) < tolerance)
  {
    ROS_INFO("Grippers's goal reached!");
    return true;
  }
  return false;
}
*/

void moveGripper(moveit::planning_interface::MoveGroupInterface& gripper_group, double aperture)
{
  double joint_value = GripperApertureConversion(aperture);
  ROS_INFO("Close gripper to joint value %.4f",joint_value);
  gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint",joint_value);
  gripper_group.move();
}

void write_D_to_log_file(double aperture, double D)
{
    std::string timeStr;
    std::ofstream log_file("/home/abb/catkin_ws/src/abb_vision_project/motion_planning/log/DiameterDataLog.txt", std::ios_base::out | std::ios_base::app );
    log_file << countAcquisitions;

    log_file <<","<< aperture << "," << D;
    log_file << std::endl;
}

int main(int argc, char **argv)
{
    //creation of the node and named it
    ros::init(argc, argv, "simple_bin_picking_test");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    sleep(2.0);

    //ros::Duration timeout(20);

    //create a move_group_interface object
    static const std::string PLANNING_GROUP_ARM = "irb_120";
    static const std::string PLANNING_GROUP_GRIPPER = "robotiq_gripper";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning arm_group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface arm_group(PLANNING_GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface gripper_group(PLANNING_GROUP_GRIPPER);

    arm_group.startStateMonitor(1.0);
    gripper_group.startStateMonitor(1.0);

    //Initialize planner
    bool success = true;
    //arm_group.setPlannerId("RRTConnect");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    /* //NOT WORKING: To fix
    auto psmPtr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

    // Listen for planning scene messages on topic /XXX and apply them to the internal planning scene accordingly
    psmPtr->startSceneMonitor();

    // Listens to changes of world geometry, collision objects, and (optionally) octomaps
    psmPtr->startWorldGeometryMonitor();
    //psmPtr->startWorldGeometryMonitor("/pick_n_place/collision_object");
  
    // Listen to joint state updates as well as changes in attached collision objects and update the internal planning scene accordingly
    psmPtr->startStateMonitor(); */

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", arm_group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", arm_group.getEndEffectorLink().c_str());

    //Target Poses 
    geometry_msgs::Pose target_pose, intermediate_pose, drop_pose;

    grasp_pose grasp;
    
    /*
    //between picking and placing
    intermediate_pose.orientation.x = 0;
    intermediate_pose.orientation.y = 0.97897;
    intermediate_pose.orientation.z = 0;
    intermediate_pose.orientation.w = 0.20401;
    intermediate_pose.position.x = 0.42237;
    intermediate_pose.position.y = 0;
    intermediate_pose.position.z = 0.45176;
    */

    //for placing (old)
    /*
    drop_pose.orientation.x = 0;
    drop_pose.orientation.y = 1;
    drop_pose.orientation.z = 0;
    drop_pose.orientation.w = -0.0047801;
    drop_pose.position.x = 0.55213;
    drop_pose.position.y = 0;
    drop_pose.position.z = 0.3244;
    */

    drop_pose.orientation.x = 0;
    drop_pose.orientation.y = 1;
    drop_pose.orientation.z = 0;
    drop_pose.orientation.w = -0.002623;
    drop_pose.position.x = 0.55291;
    drop_pose.position.y = 0;
    drop_pose.position.z = 0.25075;

    //Objects for cartesian path computation
    std::vector<geometry_msgs::Pose> waypoints;
    moveit_msgs::RobotTrajectory trajectory;
    robot_trajectory::RobotTrajectory rt(arm_group.getCurrentState()->getRobotModel(),"irb_120");
    trajectory_processing::IterativeParabolicTimeParameterization itp;
    const double jump_threshold = 1.5;
    const double eef_step = 0.01;
    double fraction;

    //Gripper joint value
    std::vector<double> gripper_joint_state;
    double joint_value;

    //Declarations for markers
    visualization_msgs::MarkerArray::ConstPtr marker_array_msg(new visualization_msgs::MarkerArray);
    std::vector<double> v; //cylindwe params. [x,y,z,qx,qy,qz,qw,diameter,height]
    double D = 0;
    double H = 0;

    ros::Duration timeout(10.0);
    double init_secs, init_secs2;

    std::string inputString;

    CHECK(ros::service::waitForService("/zivid_capture/zivid_capture_suggested",default_wait_duration));

    //Initialize gripper 
    ROS_INFO("Initialize gripper...");
    moveGripper(gripper_group, 0.02);

    ROS_INFO("Ready to capture & pick");
    
    // 0 = initialize scene, 1 = wait for processed point cloud, 2 = wait user to pick, 3 = start picking, 4 = picking fail
    int manager_mode = 0;
    bool loop_mode = false;

    while(ros::ok())
    {
      switch(manager_mode) 
      {
        case 0: //initialization scene capture
          ROS_INFO("Initialize point cloud scene...");
          system("rosservice call /zivid_capture/zivid_capture_suggested");

          manager_mode = 1;
          break;
        
        case 1: //Wait for processed point cloud
          ROS_INFO("Wait for cylinder identification...");
          marker_array_msg = ros::topic::waitForMessage<visualization_msgs::MarkerArray>("processing_marker_array",nh,timeout);
          v = marker2vector(marker_array_msg); //get cylinder marker parameters in a vector
          D = v[7];
          H = v[8];

          if(D != 0)
          {
            ROS_INFO("Cylinder idientified! Scene initialized");

            // Compute grasping
            grasp = computeCylinderGrasp(marker_array_msg,H,D);

            if(loop_mode){manager_mode = 3;} //go to pick directly
            else{manager_mode = 2;} //ask user when to pick
          }
          else
          {
            ROS_ERROR("No cylinder found. Try changing the scene");
            
            //!-Go home
            arm_group.setJointValueTarget(arm_group.getNamedTargetValues("home"));
            arm_group.move();

            ROS_INFO("Type c to capture again. Anything else to shutdown");
            std::getline(std::cin, inputString);
            if(inputString != "c"){ros::shutdown();}

            manager_mode = 0;
          }

          break;

        case 2: //wait for user to start picking
          ROS_INFO("Type p to pick once, lp to pick in a loop!");
          ROS_INFO("Anything else to shutdown");
          std::getline(std::cin, inputString);
          if(inputString != "lp" && inputString != "p" ){ros::shutdown();}
          if(inputString == "lp"){loop_mode = true;} 

          manager_mode = 3;
          break;

        case 3: //bin picking process
          ROS_INFO("PICKING START!");
          init_secs = ros::Time::now().toSec();

          //1 move the arm_group arm close to the target pose
          ROS_INFO("Moving to pre-grasp position...");
          target_pose = grasp.pregrasp_targetPose;
          stampPosition(target_pose);
          arm_group.setPoseTarget(target_pose); //valutare il planning

          success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
          ROS_INFO_NAMED("Pick&Place", "Visualizing plan 1 (pre-grasp) %s", success ? "SUCCEDED" : "FAILED");

          if(!success)
          {
            manager_mode = 4;
            break;
          }

          arm_group.execute(my_plan);

          //sleep(3.0);

          //2 close gripper in the pre-grasping phase
          moveGripper(gripper_group, D+0.006);

          timeData[0] = ros::Time::now().toSec() - init_secs; //pre-grasp time
          init_secs = ros::Time::now().toSec();

          //sleep(3.0);

          //3 move the arm_group arm to the target pose
          ROS_INFO("Moving to grasp position...");
          target_pose = grasp.grasp_targetPose;
          stampPosition(target_pose);
          waypoints.push_back(target_pose);

          fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
          waypoints.clear();

          ROS_INFO_NAMED("Pick&Place", "Visualizing plan 2 (Grasping Cartesian path) (%.2f%% acheived)", fraction * 100.0);

          if (fraction < 0.65){
            manager_mode = 4;
            break;
          }

          arm_group.execute(trajectory);

          //sleep(3.0);

          //3.5 - Open gripper to move screws away
          moveGripper(gripper_group, D+0.016);

          //4 - Close gripper
          moveGripper(gripper_group, D-0.002);

          timeData[1] = ros::Time::now().toSec() - init_secs; //grasp time
          init_secs = ros::Time::now().toSec();

          //sleep(3.0);

          //5 - Post-grasp movement
          ROS_INFO("Moving to post-grasp position...");
          target_pose = grasp.pregrasp_targetPose;
          stampPosition(target_pose);
          waypoints.push_back(target_pose);

          fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
          waypoints.clear();

          ROS_INFO_NAMED("Pick&Place", "Visualizing plan 3 (Post-grasp Cartesian path) (%.2f%% acheived)", fraction * 100.0);

          if (fraction < 0.65){
            manager_mode = 4;
            break;
          }

          arm_group.execute(trajectory);

          timeData[2] = ros::Time::now().toSec() - init_secs; //post-grasp time
          init_secs = ros::Time::now().toSec();

          //sleep(3.0);
            
          //6 - Go to drop point
          ROS_INFO("Going to drop point...");

          //if(D>0.007) drop_pose.position.y = 0.067648;
          //else if(D<0.005) drop_pose.position.y = -0.067648;
          //else drop_pose.position.y = 0;

          arm_group.setPoseTarget(drop_pose);
          arm_group.asyncMove();

          
          while (arm_group.getCurrentPose().pose.position.y < -0.30) 
          {
            std::cout<<"EEF pose.position.y = "<< arm_group.getCurrentPose().pose.position.y <<std::endl;
          }
          ROS_INFO("Manipulator is out of the camera view. Capture...");
          
          init_secs2 = ros::Time::now().toSec();
          ROS_INFO("Capture..."); //capture while moving to intermediate point
          system("rosservice call /zivid_capture/zivid_capture_suggested");

          gripper_joint_state = gripper_group.getCurrentJointValues();
          write_D_to_log_file(gripper_joint_state[0],GripperAngleConversion(gripper_joint_state[0]));
            
          do  //boolean function to check if goal is satisfied from getCurrentPose()
          {
            std::cout<<"Waiting to reach arm goal..." <<std::endl;
          }
          while (!checkArmGoal(arm_group.getCurrentPose().pose, drop_pose, 0.0001)); 

          //8 - Open Gripper while moving to drop position
          ROS_INFO("Open gripper...");
          moveGripper(gripper_group, 0.02);

          timeData[3] = ros::Time::now().toSec() - init_secs; //place time
          timeData[4] = ros::Time::now().toSec() - init_secs2; //maximum time dedicated to capture and processing while moving

          write_timeData_to_log_file(timeData);

          //Go to Retrieve cylinder information from the processed point cloud  
          manager_mode = 1;
        
          break;

        case 4: //Picking failed
          ROS_ERROR("No pickable cylinder found. Going home...");

          //!-Go home
          arm_group.setJointValueTarget(arm_group.getNamedTargetValues("home"));
          arm_group.move();

          ROS_INFO("Try changing the scene...");
          
          success = false;
          loop_mode = false;

          ROS_INFO("Type c to capture again. Anything else to shutdown");
          std::getline(std::cin, inputString);
          if(inputString != "c"){ros::shutdown();}

          manager_mode = 0;
          break;
      }
    }
}
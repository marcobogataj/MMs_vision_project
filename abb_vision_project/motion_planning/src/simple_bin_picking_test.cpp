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
      || (abs(pzvalue) > 0.50 && pyvalue <= 0)) //--> cylinder oriented to the robot
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

    grasp.grasp_targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(+0.1628+H/3); 
    grasp.grasp_targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(+0.1628+H/3);
    grasp.grasp_targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(+0.1628+H/3);

    grasp.pregrasp_targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(0.1628+H); 
    grasp.pregrasp_targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(0.1628+H);
    grasp.pregrasp_targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(0.1628+H);
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

    //change D with H for smaller diameter long height objects (+0.006 for offset)
    grasp.grasp_targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(+0.1628-D/4+0.006); 
    grasp.grasp_targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(+0.1628-D/4+0.006);
    grasp.grasp_targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(+0.1628-D/4+0.006);

    //4cm above target
    grasp.pregrasp_targetPose.position.x = targetPose.position.x - tool0_z_axis.getX()*(0.1493+0.04); //substitue 0.04 with D 
    grasp.pregrasp_targetPose.position.y = targetPose.position.y - tool0_z_axis.getY()*(0.1493+0.04);
    grasp.pregrasp_targetPose.position.z = targetPose.position.z - tool0_z_axis.getZ()*(0.1493+0.04);
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

    //between picking and placing
    intermediate_pose.orientation.x = 0;
    intermediate_pose.orientation.y = 0.97897;
    intermediate_pose.orientation.z = 0;
    intermediate_pose.orientation.w = 0.20401;
    intermediate_pose.position.x = 0.42237;
    intermediate_pose.position.y = 0;
    intermediate_pose.position.z = 0.45176;

    //for placing
    drop_pose.orientation.x = 0;
    drop_pose.orientation.y = 1;
    drop_pose.orientation.z = 0;
    drop_pose.orientation.w = -0.0047801;
    drop_pose.position.x = 0.55213;
    drop_pose.position.y = 0;
    drop_pose.position.z = 0.3244;

    //Objects for cartesian path computation
    std::vector<geometry_msgs::Pose> waypoints;
    moveit_msgs::RobotTrajectory trajectory;
    robot_trajectory::RobotTrajectory rt(arm_group.getCurrentState()->getRobotModel(),"irb_120");
    trajectory_processing::IterativeParabolicTimeParameterization itp;
    const double jump_threshold = 1.5;
    const double eef_step = 0.01;
    double fraction;

    //Gripper joint value
    double joint_value;

    //Declarations for markers
    visualization_msgs::MarkerArray::ConstPtr marker_array_msg(new visualization_msgs::MarkerArray);
    std::vector<double> v; //cylindwe params. [x,y,z,qx,qy,qz,qw,diameter,height]
    double D = 0;
    double H = 0;

    //Initialize planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = true;

    ros::Duration timeout(10.0);

    std::string inputString;

    CHECK(ros::service::waitForService("/zivid_capture/zivid_capture_suggested",default_wait_duration));

    ROS_INFO("Ready to capture & pick");

    //0 Initialize gripper 
    ROS_INFO("Initialize gripper...");
    joint_value = GripperApertureConversion(0.02); 
    gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint",joint_value);
    gripper_group.move();

    do
    {
      ROS_INFO("Initialize point cloud scene...");

      auto result = system("rosservice call /zivid_capture/zivid_capture_suggested");

      ROS_INFO("Wait for cylinder identification...");

      marker_array_msg = ros::topic::waitForMessage<visualization_msgs::MarkerArray>("processing_marker_array",nh,timeout);
      v = marker2vector(marker_array_msg); //get cylinder marker parameters in a vector
      D = v[7];
      H = v[8];

      if(D != 0)
      {
        ROS_INFO("Cylinder idientified! Scene initialized");

        v = marker2vector(marker_array_msg); //get cylinder marker parameters in a vector

        //std::cout<<"D: "<<D<<"mm "<<"H: "<<H<<"mm "<<std::endl;
        //std::cout<<"Position [x,y,z]=["<<v[0]<<","<<v[1]<<","<<v[2]<<"]"<<std::endl;
        //std::cout<<"Orientation [qx,qy,qz,qw]="<<v[3]<<","<<v[4]<<","<<v[5]<<","<<v[6]<<"]"<<std::endl;
      }
      else
      {
        ROS_ERROR("No cylinder found. Try changing the scene");
        ROS_INFO("Type c to capture again. Anything else to shutdown");
        std::getline(std::cin, inputString);
        if(inputString != "c"){
          ros::shutdown();    
        }
      }

    } while (D == 0);

    while(ros::ok())
    {
        ROS_INFO("Type p to start picking! Anything else to shutdown");
        std::getline(std::cin, inputString);

        if(inputString != "p"){
          ros::shutdown();    
        }

        //IDEALLY here we already have D,H and pose of new cylinder

        if (D != 0)
        {
          ROS_INFO("PICKING START!");

          arm_group.setPlannerId("PRM");

          moveit::planning_interface::MoveGroupInterface::Plan my_plan;


          while(true) //while grasping planning does not fail
          {
            // Compute grasping
            grasp_pose grasp = computeCylinderGrasp(marker_array_msg,H,D);

            //1 move the arm_group arm close to the target pose
            ROS_INFO("Moving to pre-grasp position...");
            target_pose = grasp.pregrasp_targetPose;
            stampPosition(target_pose);
            arm_group.setPoseTarget(target_pose); //valutare il planning

            success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("Pick&Place", "Visualizing plan 1 (pre-grasp) %s", success ? "SUCCEDED" : "FAILED");

            if(!success){break;}

            arm_group.execute(my_plan);

            //ros::Duration(0.1).sleep(); 

            //2 close gripper in the pre-grasping phase
            joint_value = GripperApertureConversion(D+0.006);
            ROS_INFO("Close gripper to joint value %.4f",joint_value);
            gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint",joint_value);
            gripper_group.move();

            //3 move the arm_group arm to the target pose

            ROS_INFO("Moving to grasp position...");
            target_pose = grasp.grasp_targetPose;
            stampPosition(target_pose);
            waypoints.push_back(target_pose);

            fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            waypoints.clear();

            rt.setRobotTrajectoryMsg(*arm_group.getCurrentState(),trajectory);
            itp.computeTimeStamps(rt,0.1,0.1);
            rt.getRobotTrajectoryMsg(trajectory);

            ROS_INFO_NAMED("Pick&Place", "Visualizing plan 2 (Grasping Cartesian path) (%.2f%% acheived)", fraction * 100.0);

            if (fraction < 0.9){
              success = false;
              break;
            }

            arm_group.execute(trajectory);

            //ros::Duration(0.1).sleep();

            //4 - Close gripper
            joint_value = GripperApertureConversion(D-0.002);
            ROS_INFO("Close gripper to joint value %.4f",joint_value);
            gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint",joint_value);
            gripper_group.move();

            //ros::Duration(0.1).sleep();

            //5 - Post-grasp movement
            ROS_INFO("Moving to post-grasp position...");
            target_pose = grasp.pregrasp_targetPose;
            stampPosition(target_pose);
            waypoints.push_back(target_pose);

            fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            waypoints.clear();

            rt.setRobotTrajectoryMsg(*arm_group.getCurrentState(),trajectory);
            itp.computeTimeStamps(rt,0.1,0.1);
            rt.getRobotTrajectoryMsg(trajectory);

            ROS_INFO_NAMED("Pick&Place", "Visualizing plan 3 (Post-grasp Cartesian path) (%.2f%% acheived)", fraction * 100.0);

            if (fraction < 0.9){
              success = false;
              break;
            }

            arm_group.execute(trajectory);
            

            //6 - Go to drop point
            ROS_INFO("Going to drop point...");
            arm_group.setPoseTarget(drop_pose);
            arm_group.asyncMove();

            /*
            while (arm_group.getCurrentPose().pose.position.y < -0.30) 
            {
              std::cout<<"EEF pose.position.y = "<< arm_group.getCurrentPose().pose.position.y <<std::endl;
            }
            ROS_INFO("Manipulator is out of the camera view. Capture...");
            */

            ROS_INFO("Capture..."); //capture while moving to intermediate point
            system("rosservice call /zivid_capture/zivid_capture_suggested");
            
            do  //boolean function to check if goal is satisfied from getCurrentPose()
            {
              std::cout<<"Waiting to reach arm goal..." <<std::endl;
            }
            while (!checkArmGoal(arm_group.getCurrentPose().pose, drop_pose, 0.0001)); 

            //8 - Open Gripper while moving to drop position
            ROS_INFO("Open gripper...");
            joint_value = GripperApertureConversion(0.02); 
            gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint",joint_value);
            //gripper_group.setJointValueTarget(gripper_group.getNamedTargetValues("open_gripper"));
            gripper_group.move(); //provare se facendo asyncMove e iscrivendosi al topic del cylindro riesco a ottenere il risultato del processing

            //Retrieve cylinder information from the point cloud processed while placing the object

            marker_array_msg = ros::topic::waitForMessage<visualization_msgs::MarkerArray>("processing_marker_array", nh,timeout);

            v = marker2vector(marker_array_msg); //get cylinder marker parameters in a vector
            D = v[7];
            H = v[8];

            if(D != 0)
            {
              ROS_INFO("Cylinder idientified!");
            }
            else
            {
              success = false;
              break;
            }
  
            //ros::Duration(3.0).sleep();

            //9 - Go to home
            //ROS_INFO("Going home...");
            //arm_group.setJointValueTarget(arm_group.getNamedTargetValues("home"));
            //arm_group.move();
          }
          
          if (!success)
          {
            ROS_ERROR("PLANNING FAILED. ABORTED!");
            //! - Go to home
            ROS_INFO("Going home...");
            arm_group.setJointValueTarget(arm_group.getNamedTargetValues("home"));
            arm_group.move();

            do
            {
              ROS_ERROR("No pickable cylinder found. Try changing the scene");
              ROS_INFO("Type c to capture again. Anything else to shutdown");
              std::getline(std::cin, inputString);
              if(inputString != "c"){
                ros::shutdown(); 
              }
              else
              {
                auto result = system("rosservice call /zivid_capture/zivid_capture_suggested");

                ROS_INFO("Wait for cylinder identification...");

                marker_array_msg = ros::topic::waitForMessage<visualization_msgs::MarkerArray>("processing_marker_array",nh, timeout);
                v = marker2vector(marker_array_msg); //get cylinder marker parameters in a vector
                D = v[7];
                H = v[8];

                if(D != 0)
                {
                ROS_INFO("Cylinder idientified! Scene initialized");

                v = marker2vector(marker_array_msg); //get cylinder marker parameters in a vector
                }
              }   
            }   
            while (D == 0);
          }
          else
          {
            ROS_INFO("PICKING COMPLETED!");
          }
        }
        else
        {
          do
          {
            ROS_ERROR("No cylinder found. Try changing the scene");
            ROS_INFO("Type c to capture again. Anything else to shutdown");
            std::getline(std::cin, inputString);
            if(inputString != "c"){
              ros::shutdown(); 
            }
            else
            {
              auto result = system("rosservice call /zivid_capture/zivid_capture_suggested");

              ROS_INFO("Wait for cylinder identification...");

              marker_array_msg = ros::topic::waitForMessage<visualization_msgs::MarkerArray>("processing_marker_array",nh, timeout);
              v = marker2vector(marker_array_msg); //get cylinder marker parameters in a vector
              D = v[7];
              H = v[8];

              if(D != 0)
              {
              ROS_INFO("Cylinder idientified! Scene initialized");

              v = marker2vector(marker_array_msg); //get cylinder marker parameters in a vector
              }
            }   
          } while (D == 0);
        }
    }
    ros::shutdown();
    return 0;
}
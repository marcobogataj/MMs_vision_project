#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::vector<double> getTarget()
{
    std::string inputString;
    std::cout << "[x,y,z,qx,qy,qz,qw]:";
    std::getline(std::cin, inputString);

    std::istringstream line(inputString);
    
    char c;
    while ((line >> c) && c != '[');

    std::vector<double> v;

    double d;
    while ((line >> d)) { v.push_back(d); line >> c; if (c != ',') { break; } }

    return v;
}

int main(int argc, char **argv)
{
    //creation of the node and named it
    ros::init(argc, argv, "calibration_target");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0);

    //create a move_group_interface object
    static const std::string PLANNING_GROUP_ARM = "irb_120";
    
    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning arm_group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface arm_group(PLANNING_GROUP_ARM);


    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", arm_group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", arm_group.getEndEffectorLink().c_str());

    //Target position (ASK FOR TARGET)
    geometry_msgs::Pose target_pose;

    // visualize the planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = arm_group.plan(my_plan);

    ROS_INFO("visualizing plan %s", success.val ? "":"FAILED");

    for (int i = 0; i < 15; i++)
    {
        ROS_INFO("Insert position %d of 15", i+1);
        std::vector<double> goal = getTarget();

        target_pose.position.x = goal[0]/1000;
        target_pose.position.y = goal[1]/1000;
        target_pose.position.z = goal[2]/1000;

        target_pose.orientation.x = goal[3];
        target_pose.orientation.y = goal[4];
        target_pose.orientation.z = goal[5];
        target_pose.orientation.w = goal[6];

        arm_group.setPoseTarget(target_pose);

        // move the arm_group arm
        arm_group.move();

        ros::Duration(3.0).sleep(); 
    }


    ros::shutdown();
    return 0;

}
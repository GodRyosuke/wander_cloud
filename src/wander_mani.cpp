#include <map>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <sstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/surface/convex_hull.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>


#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/dynamics_solver/dynamics_solver.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/RobotState.h>




// // １辺 size_of_box[m] の正方形の中に 1辺 resolution_[m] のボクセルを作成する
// void createVoxel(moveit::core::RobotModelPtr robot_state_)
// {
//   float size_of_box = 1.5;

//   // 正方形の原点
//   pcl::PointXYZ origin(0.0, 0.0, 0.0);

//   // 正方形の原点をロボットアームの搭載位置にする場合は有効にする
//   geometry_msgs::Pose arm_base_pose;
//   Eigen::Affine3d base_to_arm_base = robot_state_->getGlobalLinkTransform("base_link");
//   tf::poseEigenToMsg(base_to_arm_base, arm_base_pose);
//   origin.x = arm_base_pose.position.x;
//   origin.y = arm_base_pose.position.y;
//   origin.z = arm_base_pose.position.z;

//   for (float x = origin.x - size_of_box; x <= origin.x + size_of_box; x += resolution_) {
//     for (float y = origin.y - size_of_box; y <= origin.y + size_of_box; y += resolution_) {
//       for (float z = origin.z - size_of_box; z <= origin.z + size_of_box; z += resolution_) {
//         pcl::PointXYZ point;
//         point.x = x;
//         point.y = y;
//         point.z = z;
//         cloud_->push_back(point);
//       }
//     }
//   }
//   // Create octree for binning the base poses
//   octree_.setInputCloud(cloud_);
//   octree_.addPointsFromInputCloud();
// }

void create_reachability_map()
{
    int degree_of_freedom = 6;
    float resolution = 0.05;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZ>);
    // robot_model_loader::RobotModelLoader robot_model_loader_("robot_description");
    // robot_model::RobotModelPtr robot_model_(robot_model_loader_.getModel());
    // moveit::core::RobotModelPtr robot_state_(new robot_state::RobotState(robot_model_));


}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "wander_mani");
    ros::NodeHandle private_nh("~");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string str_param;
    bool bool_param;

    private_nh.param<std::string>("string_paramator", str_param, "hello"); // 第３引数はデフォルト値
    private_nh.param<bool>("bool_paramator", bool_param, false);

    ROS_INFO("string_param: %s", str_param.c_str());
    if (bool_param == true) {
        ROS_INFO("true!!!");
    } else {
        ROS_INFO("false !!!");
    }

    create_reachability_map();


    return (0);
}
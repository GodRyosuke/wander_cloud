#include "ros/ros.h"
#include "wander_cloud/DetectCylinder.h"
#include <cstdlib>

//
// 円柱を検出するよう要請するノード
//

int main (int argc, char** argv)
{
    ros::init(argc, argv, "detect_cylinder_from_realsense");

    ros::NodeHandle nh;
    ros::ServiceClient detect_cylinder_client = nh.serviceClient<wander_cloud::DetectCylinder>("detect_cylinder");

    wander_cloud::DetectCylinder srv;
    srv.request.is_detect = true;

    if (detect_cylinder_client.call(srv)) {
        ROS_INFO("detected cylinder from realsense point cloud");
    } else {
        ROS_INFO("failed to detect cylinder from realsense point cloud");
        return 1;
    }

    return 0;
}
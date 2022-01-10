#include "ros/ros.h"
#include "wander_cloud/SaveRealsenseData.h"
#include <cstdlib>

int main (int argc, char** argv)
{
    ros::init (argc, argv, "save_realsense_data");

    ros::NodeHandle nh;
    ros::ServiceClient save_realsense_client = nh.serviceClient<wander_cloud::SaveRealsenseData>("save_cloud");

    wander_cloud::SaveRealsenseData srv;
    srv.request.request = true;

    if (save_realsense_client.call(srv)) {
        ROS_INFO("saved realsense data");
    } else {
        ROS_INFO ("failed to save realsense data");
        return 1;
    }

    return 0;
}
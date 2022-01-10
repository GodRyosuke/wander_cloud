#include <ros/ros.h>
#include <ros/package.h>
#include "testInterface.h"
// #include <librealsense2/rs.hpp>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "cloud_test_create"); // ノード宣言
    ros::NodeHandle nh;

    // testClass* test = new testClass();
    // test->mul(1, 2);
    // ROS_INFO("result: %d", test->get_result());

    // rs2::pipeline p;
    // p.start();

    // while (true) {
    //     rs2::frameset frames = p.wait_for_frames();
    //     rs2::depth_frame depth = frames.get_depth_frame();

    //     auto width = depth.get_width();
    //     auto height = depth.get_height();

    //     // 画像の中心とカメラの間の距離を求める
    //     double dist_to_center = depth.get_distance(width / 2, height / 2);

    //     std::cout << "distance between camera and object: " << dist_to_center << std::endl;
    // }


    return 0;
}
//
// realsense d435i からpoint cloudのデータを受け取って、保存するプログラム
//

// #include <ros/ros.h>
// #include <ros/package.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
#include "wander_cloud/SaveRealsenseData.h"
#include "wander_cloud/DetectCylinder.h"

#include "detect_cylinder.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>); // realsense から得たカメラの点群データ
pcl::PointCloud<pcl::PointXYZRGB>::Ptr circle(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder(new pcl::PointCloud<pcl::PointXYZRGB>);

bool is_publish = false;

void getPointFactory (const sensor_msgs::PointCloud2Ptr& input)
{
    pcl::fromROSMsg(*input, *cloud);
}

bool saveData(wander_cloud::SaveRealsenseData::Request &req, wander_cloud::SaveRealsenseData::Response &res)
{
    if (req.request == true) {
        // 渡された点群データを保存する
        std::string file_path_ = ros::package::getPath("wander_cloud") + "/data/" + "pcd_from_realsense.pcd";
        pcl::io::savePCDFile(file_path_, *cloud);
        res.response = true;
        return true;
    } else {
        res.response = false;
    }
    return true;
}

// 円柱を検出して保存する処理
bool detectCylinder(wander_cloud::DetectCylinder::Request &req, wander_cloud::DetectCylinder::Response &res)
{
    if (req.is_detect) {
        // 一旦点群データを保存
        std::string file_path_ = ros::package::getPath("wander_cloud") + "/data/" + "pcd_from_realsense.pcd";
        pcl::io::savePCDFile(file_path_, *cloud);
        // 保存したデータを再度呼び出し
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile(file_path_, *camera_cloud);
        detect_objects* detect_object_handler = new detect_objects(camera_cloud);
        detect_object_handler->detect_intersect_circle(); // 検出
        circle = detect_object_handler->get_circle();
        plane = detect_object_handler->get_plane();
        cylinder = detect_object_handler->get_cylinder();
        ROS_INFO("called detect cylinder");
        ROS_INFO("circle: %d, plane: %d, cylinder: %d", circle->points.size(), plane->points.size(), cylinder->points.size());

        file_path_ = ros::package::getPath("wander_cloud") + "/data/" + "detected_circle_from_realsense.pcd";
        pcl::io::savePCDFile(file_path_, *circle);
        file_path_ = ros::package::getPath("wander_cloud") + "/data/" + "detected_plane_from_realsense.pcd";
        pcl::io::savePCDFile(file_path_, *plane);
        file_path_ = ros::package::getPath("wander_cloud") + "/data/" + "detected_cylinder_from_realsense.pcd";
        pcl::io::savePCDFile(file_path_, *cylinder);
        ROS_INFO("saved circle, plane, cylinder cloud data");

        //
        // 描画処理
        //
        pcl::visualization::PCLVisualizer viewer("detect cylinder and plane");
        // // 入力点群
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> input_cloud_color_handler (input_cloud, 0, 191, 255); // 青
        // viewer.addPointCloud (input_cloud, input_cloud_color_handler, "input cloud");
        // 出力の円
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> output_cloud_color_handler (circle, 127, 255, 0); // 黄緑
        viewer.addPointCloud (circle, output_cloud_color_handler, "output cloud");
        // 検出された平面
        pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_plane_color_handler (plane, 102, 153, 204); // 青
        viewer.addPointCloud (plane, cloud_plane_color_handler, "cloud plane");
        // 検出された円柱
        pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_cylinder_color_handler (cylinder, 204, 153, 102); // オレンジ
        viewer.addPointCloud (cylinder, cloud_cylinder_color_handler, "cloud cylinder");
        while (!viewer.wasStopped()) {
            viewer.spinOnce();
        }        

        res.is_ok = true;
        is_publish = true;
        return true;
    } else {
        res.is_ok = false;
    }

    return true;
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "cloud_from_realsense");
    ros::NodeHandle nh;

    // カメラの点群データを取得
    ros::Subscriber point_sub = nh.subscribe("/camera/depth_registered/points", 1, getPointFactory);
    
    // カメラの点群を保存するサーバ
    ros::ServiceServer cloud_server = nh.advertiseService("save_cloud", saveData);
    ROS_INFO("ready for saving point cloud data from realsense");
    // カメラの点群を円柱と平面、その交線を認識した結果を保存するサーバ
    ros::ServiceServer detect_cylinder_server = nh.advertiseService ("detect_cylinder", detectCylinder);
    ROS_INFO("ready for detect cylinder from realsense camera cloud data");
    ros::Rate r(1);

    // rvizに渡すためのPublisher
    ros::Publisher cloud_cylinder_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_cylinder", 5);
    sensor_msgs::PointCloud2 ros_cylinder;
    pcl::toROSMsg(*cylinder, ros_cylinder);
    ros_cylinder.header.frame_id = "camera_color_frame";

    ros::Publisher cloud_plane_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_plane", 1);
    sensor_msgs::PointCloud2 ros_plane;
    pcl::toROSMsg(*plane, ros_plane);
    ros_plane.header.frame_id = "camera_color_frame";

    ros::Publisher cloud_circle_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_circle", 1);
    sensor_msgs::PointCloud2 ros_circle;
    pcl::toROSMsg(*circle, ros_circle);
    ros_circle.header.frame_id = "camera_color_frame";

    while (ros::ok()) {
        cloud_cylinder_publisher.publish(ros_cylinder);
        cloud_plane_publisher.publish(ros_plane);
        cloud_circle_publisher.publish(ros_circle);
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}

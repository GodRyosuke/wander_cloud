#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "cloud_test_create"); // ノード宣言
    ros::NodeHandle nh;

    // どんなpoint cloudを作るのかの設定
    // 発生させるpoint cloudの設定
    pcl::PointCloud<pcl::PointXYZ> cloud_pcl_;
    cloud_pcl_.width = 5000; // 外枠
    cloud_pcl_.height = 1;
    cloud_pcl_.points.resize(cloud_pcl_.width * cloud_pcl_.height);
    for (size_t i = 0; i < cloud_pcl_.points.size(); i++) { // point cloud 直方体の中身
        cloud_pcl_.points[i].x = 0.5 * rand() / (RAND_MAX + 1.0F);
        cloud_pcl_.points[i].y = 1.0 * rand() / (RAND_MAX + 1.0F);
        cloud_pcl_.points[i].z = 1.0 * rand() / (RAND_MAX + 1.0F);
    }

    ros::Publisher cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("wander_pcl_sample_data", 1);
    sensor_msgs::PointCloud2 cloud_ros_;
    pcl::toROSMsg(cloud_pcl_, cloud_ros_); // point cloud のデータをrosのメッセージ型に変換
    cloud_ros_.header.frame_id = "base_link";

    // 所定のファイルパスにPclを保存する
    std::string file_path_ = ros::package::getPath("wander_cloud") + "/data/" + "wander_pcl_sample.pcd";
    pcl::io::savePCDFile(file_path_, cloud_pcl_);

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        cloud_publisher.publish(cloud_ros_);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
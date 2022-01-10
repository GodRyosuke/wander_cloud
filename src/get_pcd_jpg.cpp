#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include "wander_cloud/SaveRealsenseData.h"
#include <time.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
cv::Mat camera_image;

void getPointFactory(const sensor_msgs::PointCloud2Ptr& input)
{
    pcl::fromROSMsg(*input, *camera_cloud);
}

void getRGBFactory(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    camera_image = cv_ptr->image;
}

bool saveData(wander_cloud::SaveRealsenseData::Request &req, wander_cloud::SaveRealsenseData::Response &res)
{
    if (req.request == true) {
        time_t this_time = time(NULL);

        std::string pcdFilePath = ros::package::getPath("wander_cloud") + "/data/pcd_from_realsense" + std::to_string(this_time) + ".pcd";
        std::string imgFilePath = ros::package::getPath("wander_cloud") + "/img/rgb_from_realsense" + std::to_string(this_time) + ".jpg";
        pcl::io::savePCDFile(pcdFilePath, *camera_cloud);
        cv::imwrite(imgFilePath, camera_image);
        res.response = true;
    } else {
        res.response = false;
    }
    return res.response;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_pcd_jpg");
    ros::NodeHandle nh;
    ros::Rate r(1);

    // カメラの点群データを取得
    ros::Subscriber point_sub = nh.subscribe("/camera/depth/color/points", 1, getPointFactory);
    // rgbデータを取得
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber get_rgb = it.subscribe(
        "/camera/color/image_raw", 1, getRGBFactory);

    // カメラの点群を保存するサーバ
    ros::ServiceServer cloud_server = nh.advertiseService("save_pcd_jpg", saveData);

    
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    ros::shutdown();
    return 0;
}
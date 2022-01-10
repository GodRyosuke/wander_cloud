// #include <ros/ros.h>
// #include <ros/package.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
#include "detect_objects.h"
#include "wander_cloud/DetectCylinder.h"

class Test {
public:
    Test();
    ~Test();
    void get_camera_cloud(const sensor_msgs::PointCloud2Ptr& input);
    void intersect_detection();

private:
    bool ellipce_detection(wander_cloud::DetectCylinder::Request &req, wander_cloud::DetectCylinder::Response &res);

    ros::NodeHandle nh;
    ros::Subscriber point_sub;
    ros::Publisher detected_cloud_publisher;
    ros::ServiceServer detect_object_server;

    bool have_detected_cloud;

    detect_objects* detect_objects_handler;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ellipce_arc;
};

Test::Test()
    :have_detected_cloud(false)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_ellipce_arc(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    camera_cloud = test_cloud;
    ellipce_arc = test_ellipce_arc;

    point_sub = nh.subscribe("/camera/depth_registered/points", 1, &Test::get_camera_cloud, this);
    detected_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("wander_cloud", 100);
    detect_object_server = nh.advertiseService("detect_ellipce_arc", &Test::ellipce_detection, this);
}

Test::~Test()
{
    ros::shutdown();
}

void Test::get_camera_cloud(const sensor_msgs::PointCloud2Ptr& input)
{
    pcl::fromROSMsg(*input, *this->camera_cloud);

    // detect_objects_handler = new detect_objects(this->camera_cloud, 0.01f);
    // this->detect_objects_handler->detect_ellipse_arc();
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr intersect_carves = this->detect_objects_handler->get_ellipse_arc_xy();

    // delete detect_objects_handler;

    ROS_INFO("subscribed camera cloud");
    sensor_msgs::PointCloud2 wander_msg;
    pcl::toROSMsg(*ellipce_arc, wander_msg);
    detected_cloud_publisher.publish(wander_msg);
}

bool Test::ellipce_detection(wander_cloud::DetectCylinder::Request &req, wander_cloud::DetectCylinder::Response &res)
{
    if (req.is_detect == true) {
        detect_objects_handler = new detect_objects(this->camera_cloud, 0.01f);
        this->detect_objects_handler->detect_ellipse_arc();
        this->ellipce_arc = this->detect_objects_handler->get_ellipse_arc_xy();
        delete detect_objects_handler;

        ROS_INFO("succeeded in detecting ellipce arc");
        // とりあえず、認識できた円弧の点群の数を表示
        ROS_INFO("point size: %d", this->ellipce_arc->points.size());
        for (int i = 0; i < this->ellipce_arc->points.size(); i++) {
            auto this_point = this->ellipce_arc->points[i];
            ROS_INFO("x: %d, y: %d, z: %d", this_point.x, this_point.y, this_point.z);
        }
        std::string file_path_ = ros::package::getPath("wander_cloud") + "/data/" + "pcd_from_realsense.pcd";
        pcl::io::savePCDFile(file_path_, *this->ellipce_arc);

        res.is_ok = true;
    }

    return res.is_ok;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_class_topic");
    Test* test = new Test();

    ros::spin();

    delete test;
    return 0;
}
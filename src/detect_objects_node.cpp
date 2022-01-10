#include "detect_objects.h"

//
// 円柱と平面との交線を導出する
//

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_objects_node");
    ros::NodeHandle private_nh("~");

    // setup
    std::string file_name = "";
    private_nh.param<std::string>("pcl_file_name", file_name, "cylinder_and_plane.pcd");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr default_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intersect_carves (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::string filepath = ros::package::getPath("wander_cloud") + "/data/" + file_name;
    pcl::io::loadPCDFile(filepath, *default_cloud);

    // 認識処理
    detect_objects* detect_handler = new detect_objects(default_cloud, 0.01f);
    detect_handler->detect_ellipse_arc();

    intersect_carves = detect_handler->get_ellipse_arc_xy();
    cloud_plane = detect_handler->get_plane_xy();
    cloud_cylinder = detect_handler->get_cylinder_xy();

    // 描画処理
    pcl::visualization::PCLVisualizer viewer("detected masses");
    // 平面
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> plane_color (cloud_plane, 102, 153, 204); // 青
    viewer.addPointCloud(cloud_plane, plane_color, "cloud_plane");
    // 円柱(点群データ)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_cylinder_color (cloud_cylinder, 204, 153, 102); // オレンジ
    viewer.addPointCloud (cloud_cylinder, cloud_cylinder_color, "cloud_cylinder");
    // 共有線(円)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> intersect_cylinder_and_plane_with_rotation_color_handler (intersect_carves, 240, 255, 255); // 薄青
    viewer.addPointCloud (intersect_carves, intersect_cylinder_and_plane_with_rotation_color_handler, "intersect color and plane with rotation");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    ros::shutdown();
    return 0;
}
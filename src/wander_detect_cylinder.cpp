#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <boost/bind.hpp>

float voxel_size(0.01);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


void downSampling(float voxel, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_input_data)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelSampler;
    voxelSampler.setInputCloud(cloud_input_data);
    voxelSampler.setLeafSize(voxel, voxel, voxel);
    voxelSampler.filter(*cloud_sampled);
    cloud_input_data.swap(cloud_sampled);
}

void detectIntersectLineAndPlane(float line[], float plane[], float* output)
{
    float d[3]; // 直線の方向ベクトル
    for (int i = 0; i < 3; i++) {
        d[i] = line[i + 3];
    }
    float b[3]; // 直線の通る点
    for (int i = 0; i < 3; i++) {
        b[i] = line[i];
    }
    float n[3];
    for (int i = 0; i < 3; i++) {
        n[i] = plane[i];
    }
    float a4 = plane[3];

    // 計算処理
    // (a4- b*n / d*n) / d*n
    float bn = 0;
    for (int i = 0; i < 3; i++) {
        bn += b[i] * n[i];
    }
    float dn = 0;
    for (int i = 0; i < 3; i++) {
        dn += d[i] * n[i];
    }
    float k = (-a4 - bn) / dn; // 係数

    // 交点を導出
    for (int i = 0; i < 3; i++) {
        output[i] = b[i] + k * d[i];
    }
}

// vecの大きさを１にする
void create_unit_vector(float* vec)
{
    float abst = 0;
    for (int i = 0; i < 3; i++) {
        abst += vec[i] * vec[i];
    }
    abst = sqrt(abst);
    for (int i = 0; i < 3; i++) {
        vec[i] /= abst;
    }
}

void detect_cylinder_and_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& intersect_circle, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_plane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cylinder)
{
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2 (new pcl::PointCloud<pcl::Normal>);


    // 点群データの読み出し
    // std::string filepath = ros::package::getPath("wander_cloud") + "/data/" + "cylinder_and_plane.pcd";
    // pcl::io::loadPCDFile(filepath, *cloud);

    // ダウンサンプリング
    downSampling(0.005, cloud);

    // 法線の計算
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setInputCloud (cloud);
    norm_est.setKSearch (50);
    norm_est.compute (*cloud_normals);

    // 平面検出
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices());
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.02);
    seg.setInputCloud (cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers_plane, *coefficients_plane);
    std::cerr << "plane coefficients: " << *coefficients_plane << std::endl;

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_plane);
    extract.setNegative(false);
    extract.filter (*cloud_plane);

    // 出力平面点群にコピー
    for (int i = 0; i < cloud_plane->points.size(); i++) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = cloud_plane->points[i].x;
        temp_point.y = cloud_plane->points[i].y;
        temp_point.z = cloud_plane->points[i].z;
        output_plane->push_back(temp_point);
    }

    extract.setNegative (true);
    extract.filter (*cloud_filtered_2);
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals_2);

    // 円柱検出
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0.04, 0.1);
    seg.setInputCloud (cloud_filtered_2);
    seg.setInputNormals (cloud_normals_2);
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    extract.setInputCloud (cloud_filtered_2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud_cylinder);

    // 出力円柱点群にコピー
    for (int i = 0; i < cloud_cylinder->points.size(); i++) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = cloud_cylinder->points[i].x;
        temp_point.y = cloud_cylinder->points[i].y;
        temp_point.z = cloud_cylinder->points[i].z;
        output_cylinder->push_back(temp_point);
    }

    // 円柱の中心点
    pcl::PointXYZ center_point;
    center_point.x = coefficients_cylinder->values[0];
    center_point.y = coefficients_cylinder->values[1];
    center_point.z = coefficients_cylinder->values[2];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_center_point (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_center_point->push_back (center_point);


    // 円柱のモデル
    // 微小な高さの円柱で円の代わりとする
    pcl::ModelCoefficients cylinder_coeff_fromdata;
    cylinder_coeff_fromdata.values.resize(7);
    for (int i = 0; i < 7; i++) {
        cylinder_coeff_fromdata.values[i] = coefficients_cylinder->values[i];
    }

    // 平面のモデル
    pcl::ModelCoefficients plane_coeff_fromdata;
    plane_coeff_fromdata.values.resize(4);
    for (int i = 0; i < 4; i++) {
        plane_coeff_fromdata.values[i] = coefficients_plane->values[i];
    }


    // -----------------------------------
    // 平面と円柱の交線検出処理
    // -----------------------------------

    // 円の点群を作成
    const int max_circle_points = 50;
    const float PI = 3.1415;

    // 直線と円柱の軸との交点を検出
    float line[6]; // 0~2は直線が通る点、3~5は方向ベクトル
    for (int i = 0; i < 6; i++) {
        line[i] = coefficients_cylinder->values[i]; 
    }
    float plane[4]; // 0から順に、a, b, c, d(ax+by+cz+d=0に対応)
    for (int i = 0;i < 4; i++) {
        plane[i] = coefficients_plane->values[i];
    }
    float IntersectPoint[3];
    // 実際の計算処理
    detectIntersectLineAndPlane(line, plane, IntersectPoint);
    // 点群データに変換
    pcl::PointXYZRGB IntersectPoint_data;
    IntersectPoint_data.x = IntersectPoint[0];
    IntersectPoint_data.y = IntersectPoint[1];
    IntersectPoint_data.z = IntersectPoint[2];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr IntersectCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    IntersectCloud->push_back(IntersectPoint_data);

    float r = coefficients_cylinder->values[6]; // 円柱の半径

    // x-z平面上の原点中心の円
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_circle (new pcl::PointCloud<pcl::PointXYZRGB>); 
    for (int i = 0; i < max_circle_points; i++) {
        pcl::PointXYZRGB cloud_circle_point;
        cloud_circle_point.x = r * cos(2.0f * PI / (float)max_circle_points * (float)i);
        cloud_circle_point.y = 0.0f;
        cloud_circle_point.z = r * sin(2.0f * PI / (float)max_circle_points * (float)i);
        temp_circle->push_back(cloud_circle_point);
    }
    float plane_norm[3]; // 検出された平面の法線ベクトル
    for (int i = 0; i < 3; i++) {
        plane_norm[i] = plane[i];
    }
    create_unit_vector(plane_norm); // 大きさを１にする
    ROS_INFO("plane_norm: ");
    for (int i = 0; i < 3; i++) {
        ROS_INFO("%f ", plane_norm[i]);
    }
    // 回転軸を外積で求める
    float axis_plane_and_circle[3];
    axis_plane_and_circle[0] = plane_norm[2];
    axis_plane_and_circle[1] = 0.0f;
    axis_plane_and_circle[2] = -plane_norm[0];
    create_unit_vector(axis_plane_and_circle);
    // 回転角を内積で求める
    float theta_plane_and_circle = acos(plane_norm[1]);
    // 回転処理
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_circle (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < temp_circle->points.size(); i++) {
        Eigen::VectorXf input_vec(3), output_vec(3);
        input_vec[0] = temp_circle->points[i].x;
        input_vec[1] = temp_circle->points[i].y;
        input_vec[2] = temp_circle->points[i].z;

        // 所定の回転軸で回転するQuaternionの生成
        Eigen::Quaternionf quat;
        Eigen::Vector3f axis;
        for (int j = 0; j < 3; j++) {
            axis[j] = axis_plane_and_circle[j];
        }
        quat = Eigen::AngleAxisf(theta_plane_and_circle, axis);
        // 回転処理
        output_vec = quat * input_vec;
        pcl::PointXYZRGB temp_point;
        temp_point.x = output_vec[0];
        temp_point.y = output_vec[1];
        temp_point.z = output_vec[2];
        rotated_circle->push_back(temp_point);
    }
    // 平行移動処理
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intersect_cylinder_and_plane_with_rotation (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < rotated_circle->points.size(); i++) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = rotated_circle->points[i].x + IntersectPoint[0];
        temp_point.y = rotated_circle->points[i].y + IntersectPoint[1];
        temp_point.z = rotated_circle->points[i].z + IntersectPoint[2];
        intersect_cylinder_and_plane_with_rotation->push_back(temp_point);
    }

    // 出力点群にコピー
    for (int i = 0; i < intersect_cylinder_and_plane_with_rotation->points.size(); i++) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = intersect_cylinder_and_plane_with_rotation->points[i].x;
        temp_point.y = intersect_cylinder_and_plane_with_rotation->points[i].y;
        temp_point.z = intersect_cylinder_and_plane_with_rotation->points[i].z;
        intersect_circle->push_back(temp_point);
    }

    //
    // 描画処理
    //

    // // 点群データ表示
    // pcl::visualization::PCLVisualizer viewer("detected masses");
    // // 平面
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> plane_color (cloud_plane, 102, 153, 204); // 青
    // viewer.addPointCloud(cloud_plane, plane_color, "cloud_plane");
    // // 円柱(点群データ)
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_cylinder_color (cloud_cylinder, 204, 153, 102); // オレンジ
    // viewer.addPointCloud (cloud_cylinder, cloud_cylinder_color, "cloud_cylinder");
    // // 円柱の中心
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_center_point_color_handler (cloud_center_point, 255, 215, 0);  // 黃
    // viewer.addPointCloud<pcl::PointXYZ> (cloud_center_point, cloud_center_point_color_handler, "cloud_center_point");
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cloud_center_point");
    // // 円柱(数学モデル)
    // // viewer.addCylinder(cylinder_coeff_fromdata, "cylinder_from_data");
    // // 平面のモデルを描画
    // viewer.addPlane(plane_coeff_fromdata, "plane_from_data");

    // // 円柱の軸と平面との交点
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> intersect_color_handler (IntersectCloud, 0, 191, 255); // 青
    // viewer.addPointCloud (IntersectCloud, intersect_color_handler, "intersect_point");
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "intersect_point"); // 点のサイズを大きくする
    // // 回答の交線回転、平行移動後
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> intersect_cylinder_and_plane_with_rotation_color_handler (intersect_cylinder_and_plane_with_rotation, 240, 255, 255); // 薄青
    // viewer.addPointCloud (intersect_cylinder_and_plane_with_rotation, intersect_cylinder_and_plane_with_rotation_color_handler, "intersect color and plane with rotation");
    
    // while (!viewer.wasStopped()) {
    //     viewer.spinOnce();
    // }


    // 点群データ保存
    // filepath = ros::package::getPath("wander_cloud") + "/data/" + "cylinder_downsampled1.pcd";
    // pcl::io::savePCDFile(filepath, *cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr circle_cloud_from_camera (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud_from_camera (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_cloud_from_camera (new pcl::PointCloud<pcl::PointXYZRGB>);


void detect_cylinder_from_realsense(const sensor_msgs::PointCloud2Ptr& input)
{
    pcl::fromROSMsg(*input, *camera_cloud);
    // 検出処理
    detect_cylinder_and_plane(camera_cloud, circle_cloud_from_camera, plane_cloud_from_camera, cylinder_cloud_from_camera);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv,  "wander_detect_cylinder");
    ros::NodeHandle nh;

    // realsense からのデータを得る

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr default_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intersect_circle (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);

    // std::string filepath = ros::package::getPath("wander_cloud") + "/data/" + "wander_cylinder_and_plane.pcd";
    // pcl::io::loadPCDFile(filepath, *default_cloud);
    // detect_cylinder_and_plane(default_cloud, intersect_circle, cloud_plane, cloud_cylinder);

    ros::Subscriber realsense_subs = nh.subscribe("/camera/depth_registered/points", 1, detect_cylinder_from_realsense);

    // 点群データ読み出し
    std::string filepath = ros::package::getPath("wander_cloud") + "/data/" + "cylinder_sample_2.pcd";
    pcl::io::loadPCDFile(filepath, *default_cloud);

    // 検出処理
    detect_cylinder_and_plane(default_cloud, intersect_circle, cloud_plane, cloud_cylinder);
 
    
    // rvizに渡すためのPublisher
    ros::Publisher cloud_cylinder_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_cylinder", 5);
    sensor_msgs::PointCloud2 ros_cylinder;
    pcl::toROSMsg(*cloud_cylinder, ros_cylinder);
    ros_cylinder.header.frame_id = "camera_color_frame";

    ros::Publisher cloud_plane_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_plane", 1);
    sensor_msgs::PointCloud2 ros_plane;
    pcl::toROSMsg(*cloud_plane, ros_plane);
    ros_plane.header.frame_id = "camera_color_frame";

    ros::Publisher cloud_circle_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_circle", 1);
    sensor_msgs::PointCloud2 ros_circle;
    pcl::toROSMsg(*intersect_circle, ros_circle);
    ros_circle.header.frame_id = "camera_color_frame";

    // ros::Publisher test_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_test_pub", 5);
    // sensor_msgs::PointCloud2 ros_test_cloud;
    // pcl::toROSMsg(*intersect_circle, ros_test_cloud);
    // ros_test_cloud.header.frame_id = "camera_color_frame";

    // ros::Rate loop_rate(1);
    // while (ros::ok()) {
    //     // cloud_publisher.publish(msg);
    //     //test_pub.publish(ros_test_cloud);
    //     cloud_cylinder_publisher.publish(ros_cylinder);
    //     cloud_plane_publisher.publish(ros_plane);
    //     cloud_circle_publisher.publish(ros_circle);
        
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    //
    // 描画処理
    //
    pcl::visualization::PCLVisualizer viewer("detect cylinder and plane");
    // // 入力点群
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> input_cloud_color_handler (input_cloud, 0, 191, 255); // 青
    // viewer.addPointCloud (input_cloud, input_cloud_color_handler, "input cloud");
    // 出力の円
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> output_cloud_color_handler (intersect_circle, 127, 255, 0); // 黄緑
    viewer.addPointCloud (intersect_circle, output_cloud_color_handler, "output cloud");
    // 検出された平面
    pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_plane_color_handler (cloud_plane, 102, 153, 204); // 青
    viewer.addPointCloud (cloud_plane, cloud_plane_color_handler, "cloud plane");
    // 検出された円柱
    pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_cylinder_color_handler (cloud_cylinder, 204, 153, 102); // オレンジ
    viewer.addPointCloud (cloud_cylinder, cloud_cylinder_color_handler, "cloud cylinder");
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }



    return 0;
}
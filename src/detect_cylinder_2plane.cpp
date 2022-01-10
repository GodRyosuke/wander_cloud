#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/io/pcd_io.h>

void downSampling(float voxel, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_input_data)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelSampler;
    voxelSampler.setInputCloud(cloud_input_data);
    voxelSampler.setLeafSize(voxel, voxel, voxel);
    voxelSampler.filter(*cloud_sampled);
    cloud_input_data.swap(cloud_sampled);
}

void detect_cylinder_2plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2 (new pcl::PointCloud<pcl::Normal>);


    // ------------------------------------
    // 入力点群から平面と円柱を検出する処理
    // ------------------------------------

    
    // ダウンサンプリング
    downSampling(0.005, cloud);

    // 法線の計算
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setInputCloud (cloud);
    norm_est.setKSearch (50);
    norm_est.compute (*cloud_normals);

    // 平面1検出
    pcl::ModelCoefficients::Ptr coefficients_plane1 (new pcl::ModelCoefficients());
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
    seg.segment(*inliers_plane, *coefficients_plane1);
    std::cerr << "plane coefficients: " << *coefficients_plane1 << std::endl;


    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_plane);
    extract.setNegative(false);
    extract.filter (*cloud_plane);


    extract.setNegative (true);
    extract.filter (*cloud_filtered_2);
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals_2);


    // 平面2検出
    pcl::ModelCoefficients::Ptr coefficients_plane2 (new pcl::ModelCoefficients());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.02);
    seg.setInputCloud (cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers_plane, *coefficients_plane2);
    std::cerr << "plane coefficients: " << *coefficients_plane2 << std::endl;


    extract.setInputCloud (cloud);
    extract.setIndices (inliers_plane);
    extract.setNegative(false);
    extract.filter (*cloud_plane);


    extract.setNegative (true);
    extract.filter (*cloud_filtered_2);
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

    // have_nan = false;
    // for (int i = 0; i < 7; i++) {
    //     if (!std::isfinite(coefficients_cylinder->values[i])) {
    //         have_nan = true;
    //     }
    // }
    // if (have_nan) {
    //     ROS_INFO("error: cannnot detect cylinder!!");
    //     return false;
    // }

    // extract.setInputCloud (cloud_filtered_2);
    // extract.setIndices (inliers_cylinder);
    // extract.setNegative (false);
    // extract.filter (*cloud_cylinder);

    // // 円柱の中心点
    // pcl::PointXYZ center_point;
    // center_point.x = coefficients_cylinder->values[0];
    // center_point.y = coefficients_cylinder->values[1];
    // center_point.z = coefficients_cylinder->values[2];
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_center_point (new pcl::PointCloud<pcl::PointXYZ>);
    // cloud_center_point->push_back (center_point);


    // // 平面のモデル
    // pcl::ModelCoefficients plane_coeff_fromdata;
    // plane_coeff_fromdata.values.resize(4);
    // for (int i = 0; i < 4; i++) {
    //     plane_coeff_fromdata.values[i] = coefficients_plane->values[i];
    // }


    // // -----------------------------------
    // // 平面と円柱の交線(円弧)検出処理
    // // -----------------------------------

    // // 円の点群を作成
    // const int max_circle_points = 50;
    // const float PI = 3.1415;

    // // 直線と円柱の軸との交点を検出
    // float line[6]; // 0~2は直線が通る点、3~5は方向ベクトル
    // for (int i = 0; i < 6; i++) {
    //     line[i] = coefficients_cylinder->values[i]; 
    // }
    // float plane[4]; // 0から順に、a, b, c, d(ax+by+cz+d=0に対応)
    // for (int i = 0;i < 4; i++) {
    //     plane[i] = coefficients_plane->values[i];
    // }
    // float IntersectPoint[3];
    // // 実際の計算処理
    // detectIntersectLineAndPlane(line, plane, IntersectPoint);
    // // 点群データに変換
    // pcl::PointXYZRGB IntersectPoint_data;
    // IntersectPoint_data.x = IntersectPoint[0];
    // IntersectPoint_data.y = IntersectPoint[1];
    // IntersectPoint_data.z = IntersectPoint[2];
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr IntersectCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // IntersectCloud->push_back(IntersectPoint_data);

    // float r = coefficients_cylinder->values[6]; // 円柱の半径
    // cylinder_radius = r;

    // // x-y平面上の原点中心の円を生成
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_circle (new pcl::PointCloud<pcl::PointXYZRGB>); 
    // for (int i = 0; i < max_circle_points; i++) {
    //     pcl::PointXYZRGB cloud_circle_point;
    //     cloud_circle_point.x = r * cos(2.0f * PI * (float)i / (float)max_circle_points);
    //     cloud_circle_point.y = r * sin(2.0f * PI * (float)i / (float)max_circle_points);
    //     cloud_circle_point.z = 0.0f;
    //     temp_circle->push_back(cloud_circle_point);
    // }

    // // 平面、円柱をその交点が原点に重なるようにそれぞれ平行移動
    // float plane_cylinder_mode_direction[3];
    // for (int i = 0; i < 3; i++) {
    //     plane_cylinder_mode_direction[i] = -IntersectPoint[i];
    // }
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfered_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    // transfer_cloud(cloud_plane, transfered_plane, plane_cylinder_mode_direction);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfered_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    // transfer_cloud(cloud_cylinder, transfered_cylinder, plane_cylinder_mode_direction);

    // float plane_norm[3]; // 検出された平面の法線ベクトル
    // for (int i = 0; i < 3; i++) {
    //     plane_norm[i] = plane[i];
    // }
    // create_unit_vector(plane_norm); // 大きさを１にする
    // // 回転軸を外積で求める
    // float axis_plane_and_circle[3];
    // axis_plane_and_circle[0] = -plane_norm[1];
    // axis_plane_and_circle[1] = plane_norm[0];
    // axis_plane_and_circle[2] = 0.0f;
    // create_unit_vector(axis_plane_and_circle); // 回転軸の大きさ１
    // // 回転角を内積で求める
    // float theta_plane_and_circle = acos(plane_norm[2]);
    // // 回転処理
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    // rotate_cloud_with_quat(transfered_plane, rotated_plane, -theta_plane_and_circle, axis_plane_and_circle);
    // rotate_cloud_with_quat(transfered_cylinder, rotated_cylinder, -theta_plane_and_circle, axis_plane_and_circle);

    // // 平面の数学モデルの法線ベクトルも同様に回転
    // plane_coeff_fromdata.values[3] =0.0f;
    // Eigen::VectorXf input_vec(3), output_vec(3);
    // for (int i = 0; i < 3; i++) {
    //     input_vec[i] = plane_coeff_fromdata.values[i];
    // }
    // // 所定の回転軸で回転するQuaternionの生成
    // Eigen::Quaternionf quat;
    // Eigen::Vector3f axis_vector;
    // for (int j = 0; j < 3; j++) {
    //     axis_vector[j] = axis_plane_and_circle[j];
    // }
    // quat = Eigen::AngleAxisf(-theta_plane_and_circle, axis_vector);
    // // 回転処理
    // output_vec = quat * input_vec;
    // for (int i = 0; i < 3; i++) {
    //     plane_coeff_fromdata.values[i] = output_vec[i];
    // }

    // // 円柱のcoeffも回転させる
    // input_vec[0] = coefficients_cylinder->values[3];
    // input_vec[1] = coefficients_cylinder->values[4];
    // input_vec[2] = coefficients_cylinder->values[5];
    // output_vec = quat * input_vec;
    // pcl::ModelCoefficients temp_cylinder_coeff;
    // temp_cylinder_coeff.values.resize(7);
    // for (int i = 0; i < 3; i++) {
    //     temp_cylinder_coeff.values[i] = 0.0f;
    // }
    // temp_cylinder_coeff.values[3] = output_vec[0];
    // temp_cylinder_coeff.values[4] = output_vec[1];
    // temp_cylinder_coeff.values[5] = output_vec[2];
    // temp_cylinder_coeff.values[6] = r;


    // copy_cloud(rotated_plane, cloud_plane_xy);
    // copy_cloud(rotated_cylinder, cloud_cylinder_xy);
    // copy_cloud(temp_circle, cloud_circle_xy);
    // this->cylinder_coeff_xy = temp_cylinder_coeff;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_cylinder_2plane");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr default_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string file_name = ros::package::getPath("wander_cloud") + "/data/" + "cylinder_and_2plane_seg.pcd";
    pcl::io::loadPCDFile(file_name, *default_cloud);

    detect_cylinder_2plane(default_cloud);



    ros::shutdown();
    return 0;
}
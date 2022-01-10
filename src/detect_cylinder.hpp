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


class detect_objects
{
public:
    detect_objects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, float voxel = 0.01f);
    
    // 円がxy平面上にないときの円柱と平面との交線検出
    void detect_intersect_circle();
    // 入力点群cloudから平面と円柱を検出し、その共有線(円)を求める。
    // ただし、平面がxy平面上に、交線となる円はその中心が原点になるように座標変換している。
    void detect_intersect_circle_at_xy();
    // 入力点群から円柱の見える範囲の円弧を抽出する
    void detect_intersect_arc();
    // 円柱が斜めに平面と交わったときにできる楕円を検出
    void detect_ellipse();
    // 円柱と平面が交わった交線だけを検出(弧)
    void detect_ellipse_arc();

    void set_voxel(float voxel) { voxel_size = voxel; }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_plane() { return cloud_plane; }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cylinder() { return cloud_cylinder; }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_circle() { return cloud_circle; }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_plane_xy() { return cloud_plane_xy; }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cylinder_xy() { return cloud_cylinder_xy; }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_circle_xy() { return cloud_circle_xy; }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_arc_xy() { return cloud_arc_xy; }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_ellipse_xy() { return cloud_ellipse_xy; }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_ellipse_arc_xy() { return cloud_ellipse_arc_xy; }
    
private:
    void downSampling(float voxel, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_input_data);
    // 直線と平面との交点を求める
    void detectIntersectLineAndPlane(float line[], float plane[], float* output);
    void create_unit_vector(float* vec); // vecのサイズを１にする
    // input_cloudをdirectionの方向に平行移動し、output_cloudにする
    void transfer_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud,
    float* direction);
    // input_cloudをaxisを回転軸にしてthetaだけ回転してoutput_cloudにする
    void rotate_cloud_with_quat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud,
    float theta, float* axis);
    // xyz入力座標からxy平面上でのx軸に対する偏角を導出
    float lead_rad(float* input_coordinate);
    void swap(float& a, float& b);
    void insertion(float* array, int begin, int end);
    // input_cloudをoutput_cloudにコピー
    void copy_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);

    float voxel_size;
    float cylinder_radius;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_data;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_circle;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_xy;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder_xy;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_circle_xy;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ellipse_xy;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_arc_xy;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ellipse_arc_xy;

    pcl::ModelCoefficients cylinder_coeff_xy; // 円柱の軸、半径のデータ
};

detect_objects::detect_objects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, float voxel)
    :cloud_data(input_cloud),
    voxel_size(voxel)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_plane_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_cylinder_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_circle_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_plane_cloud_xy (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_cylinder_cloud_xy (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_circle_cloud_xy (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_ellipse_xy (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr test_ellipse_arc_xy (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    cloud_plane = test_plane_cloud;
    cloud_cylinder = test_cylinder_cloud;
    cloud_circle = test_circle_cloud;

    cloud_plane_xy = test_plane_cloud_xy;
    cloud_cylinder_xy = test_cylinder_cloud_xy;
    cloud_circle_xy = test_circle_cloud_xy;
    cloud_ellipse_xy = test_ellipse_xy;
    cloud_ellipse_arc_xy = test_ellipse_arc_xy;
}

void detect_objects::copy_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
{
    for (int i = 0; i < input_cloud->points.size(); i++) {
        pcl::PointXYZRGB test_point;
        test_point.x = input_cloud->points[i].x;
        test_point.y = input_cloud->points[i].y;
        test_point.z = input_cloud->points[i].z;
        output_cloud->push_back(test_point);
    }
}


// 円柱と平面が交わった交線だけを検出(弧)
void detect_objects::detect_ellipse_arc()
{
    detect_intersect_circle_at_xy();

    // 円柱の中心軸
    pcl::ModelCoefficients line_coeff;
    line_coeff.values.resize(6);
    for (int i = 0; i < 6; i++) {
        line_coeff.values[i] = this->cylinder_coeff_xy.values[i];
    }
    // x軸
    pcl::ModelCoefficients x_axis;
    x_axis.values.resize(6);
    for (int i = 0; i < 6; i++) {
        x_axis.values[i] = 0;
    }
    x_axis.values[3] = 1.0f;


    // 円柱の中心軸がx軸の真上になるように回転させる
    // 円柱の中心軸の回転
    float cylinder_line_dir[3];
    float rotation_axis[] = {0, 0, 1.0f};
    for (int i = 0; i < 3; i++) {
        cylinder_line_dir[i] = this->cylinder_coeff_xy.values[i + 3];
    }
    create_unit_vector(cylinder_line_dir);
    // 円柱の中心軸とx軸との成す角
    float theta_cylinder_and_x = acos(cylinder_line_dir[0]);
    // 回転処理
    Eigen::VectorXf input_vec(3), output_vec(3);
    input_vec[0] = cylinder_line_dir[0];
    input_vec[1] = cylinder_line_dir[1];
    input_vec[2] = cylinder_line_dir[2];

    Eigen::Quaternionf quat;
    Eigen::Vector3f axis_vector;
    for (int i = 0; i < 3; i++) {
        axis_vector[i] = 0;
    }
    axis_vector[2] = 1.0f;  // z軸回りに回転
    quat = Eigen::AngleAxisf(-theta_cylinder_and_x, axis_vector);
    // 回転処理
    output_vec = quat * input_vec;
    float rotated_cylinder_dir[3];
    for (int i = 0; i < 3; i++) {
        rotated_cylinder_dir[i] = output_vec[i];
    }
    pcl::ModelCoefficients rotated_cylinder_line;
    rotated_cylinder_line.values.resize(6);
    for (int i = 0; i < 3; i++) {
        rotated_cylinder_line.values[i] = 0;
        rotated_cylinder_line.values[i + 3] = rotated_cylinder_dir[i];
    }
    // 円柱の点群を回転
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    this->rotate_cloud_with_quat(this->cloud_cylinder_xy, rotated_cylinder, -theta_cylinder_and_x, rotation_axis);
    // 平面の点群を回転
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    this->rotate_cloud_with_quat(this->cloud_plane_xy, rotated_plane, -theta_cylinder_and_x, rotation_axis);

    // 共有線(楕円)の検出処理
    create_unit_vector(rotated_cylinder_dir);
    float theta_cylinder_and_z = acos(rotated_cylinder_dir[2]);
    float r = cylinder_radius;
    float a = r / cos(theta_cylinder_and_z); // 楕円の長半径
    float b = r; // 楕円の短半径

    // 楕円弧を描画する範囲を決定
    // zの値が小さい点の偏角の範囲を用いた
    float max_z = rotated_cylinder->points[0].z;
    float min_z = rotated_cylinder->points[0].z;
    for (int i = 0; i < rotated_cylinder->points.size(); i++) {
        float this_z = rotated_cylinder->points[i].z;
        if (max_z < this_z) {
            max_z = this_z;
        }
        if (this_z < min_z) {
            min_z = this_z;
        }
    }
    const int divide_num = 5; // 楕円をz軸についてdivide_num分割する
    float delta_z = (max_z - min_z) / (float)divide_num;
    std::vector<float> theta_array;
    for (int i = 0; i < rotated_cylinder->points.size(); i++) {
        float this_z = rotated_cylinder->points[i].z;
        if ((min_z <= this_z) && (this_z <= (min_z + delta_z))) { // xy平面から微小な高さであれば
            float this_coordinate[3];
            this_coordinate[0] = rotated_cylinder->points[i].x;
            this_coordinate[1] = rotated_cylinder->points[i].y;
            this_coordinate[2] = rotated_cylinder->points[i].z;
            float this_theta = lead_rad(this_coordinate);
            theta_array.push_back(this_theta);
        }
    }
    // 十分な偏角の情報が得られなかったとき
    if (theta_array.size() < 5) {
        ROS_INFO("error: not so much data of theta. plese change param: divide_num");
        return;
    }
    // 並び替え
    float min_theta = theta_array[0];
    float* theta_sort_array = new float[theta_array.size()];
    for (int i = 0; i < theta_array.size(); i++) {
        theta_sort_array[i] = theta_array[i];
    }
    insertion(theta_sort_array, 0, theta_array.size() - 1);

    // 最小のthetaと最大のthetaの5平均をthetaとする
    float ellipse_min_theta = 0;
    float ellipse_max_theta = 0;
    for (int i = 0; i < 5; i++) {
        ellipse_min_theta += theta_sort_array[i];
        ellipse_max_theta += theta_sort_array[theta_array.size() - i - 1];
    }
    ellipse_min_theta /= 5;
    ellipse_max_theta /= 5;
    ROS_INFO("max_theta: %f", ellipse_max_theta);
    ROS_INFO("min_theta: %f", ellipse_min_theta);

    // 共有線(楕円弧)
    const int PI = 3.1415;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intersect_ellipse_arc_xy (new pcl::PointCloud<pcl::PointXYZRGB>);
    const int max_ellipse_point = 50;
    for (int i = 0; i < max_ellipse_point; i++) {
        pcl::PointXYZRGB temp_point;
        float this_theta = 2 * PI * (float)i / (float)max_ellipse_point;
        if ((ellipse_min_theta <= this_theta) && (this_theta <= ellipse_max_theta)) {
            temp_point.x = a * cos(this_theta);
            temp_point.y = b * sin(this_theta);
            temp_point.z = 0.0f;
            intersect_ellipse_arc_xy->push_back(temp_point);
            ROS_INFO("added this point");
        }
    }

    copy_cloud(intersect_ellipse_arc_xy, this->cloud_ellipse_arc_xy);
    this->cloud_cylinder_xy->points.clear();
    copy_cloud(rotated_cylinder, this->cloud_cylinder_xy);
    this->cloud_plane_xy->points.clear();
    copy_cloud(rotated_plane, this->cloud_plane_xy);

    

    // //
    // // 描画処理
    // //
    // pcl::visualization::PCLVisualizer viewer("detect cylinder and plane");
    // // // 出力の円
    // // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> output_cloud_color_handler (cloud_circle_xy, 127, 255, 0); // 黄緑
    // // viewer.addPointCloud (cloud_circle_xy, output_cloud_color_handler, "output cloud");
    // // 検出された平面
    // pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_plane_color_handler (cloud_plane_xy, 102, 153, 204); // 青
    // viewer.addPointCloud (cloud_plane_xy, cloud_plane_color_handler, "cloud plane");
    // // // 検出された円柱
    // // pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_cylinder_color_handler (cloud_cylinder_xy, 204, 153, 102); // オレンジ
    // // viewer.addPointCloud (cloud_cylinder_xy, cloud_cylinder_color_handler, "cloud cylinder");
    // // viewer.addCylinder(this->cylinder_coeff_xy, "this_cylinder");
    // // 円柱の中心軸
    // const std::string line_str = "line_model";
    // viewer.addLine(line_coeff, line_str); 
    // // x軸
    // const std::string x_axis_str = "x_axis";
    // viewer.addLine(x_axis, x_axis_str);
    // // 回転後の中心軸
    // const std::string rotated_cylinder_line_str = "rotatd center axis";
    // viewer.addLine(rotated_cylinder_line, rotated_cylinder_line_str);
    // // 回転後の円柱点群
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rotated_cylinder_color_handler (rotated_cylinder, 255, 255, 0); // 紫
    // viewer.addPointCloud (rotated_cylinder, rotated_cylinder_color_handler, "rotated_cylinder_cloud");
    // // 共有線(楕円弧)
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> intersect_ellipse_xy_color_handler(intersect_ellipse_arc_xy, 173, 255, 47); // 黄緑
    // viewer.addPointCloud(intersect_ellipse_arc_xy, intersect_ellipse_xy_color_handler, "intersect_ellipse_xt");
    
    // while (!viewer.wasStopped()) {
    //     viewer.spinOnce();
    // }

}

// 円柱が斜めに平面と交わったときにできる楕円を検出
void detect_objects::detect_ellipse()
{
    detect_intersect_circle_at_xy();

    // 円柱の中心軸
    pcl::ModelCoefficients line_coeff;
    line_coeff.values.resize(6);
    for (int i = 0; i < 6; i++) {
        line_coeff.values[i] = this->cylinder_coeff_xy.values[i];
    }
    // x軸
    pcl::ModelCoefficients x_axis;
    x_axis.values.resize(6);
    for (int i = 0; i < 6; i++) {
        x_axis.values[i] = 0;
    }
    x_axis.values[3] = 1.0f;


    // 円柱の中心軸がx軸の真上になるように回転させる
    // 円柱の中心軸の回転
    float cylinder_line_dir[3];
    float rotation_axis[] = {0, 0, 1.0f};
    for (int i = 0; i < 3; i++) {
        cylinder_line_dir[i] = this->cylinder_coeff_xy.values[i + 3];
    }
    create_unit_vector(cylinder_line_dir);
    // 円柱の中心軸とx軸との成す角
    float theta_cylinder_and_x = acos(cylinder_line_dir[0]);
    // 回転処理
    Eigen::VectorXf input_vec(3), output_vec(3);
    input_vec[0] = cylinder_line_dir[0];
    input_vec[1] = cylinder_line_dir[1];
    input_vec[2] = cylinder_line_dir[2];

    Eigen::Quaternionf quat;
    Eigen::Vector3f axis_vector;
    for (int i = 0; i < 3; i++) {
        axis_vector[i] = 0;
    }
    axis_vector[2] = 1.0f;  // z軸回りに回転
    quat = Eigen::AngleAxisf(-theta_cylinder_and_x, axis_vector);
    // 回転処理
    output_vec = quat * input_vec;
    float rotated_cylinder_dir[3];
    for (int i = 0; i < 3; i++) {
        rotated_cylinder_dir[i] = output_vec[i];
    }
    pcl::ModelCoefficients rotated_cylinder_line;
    rotated_cylinder_line.values.resize(6);
    for (int i = 0; i < 3; i++) {
        rotated_cylinder_line.values[i] = 0;
        rotated_cylinder_line.values[i + 3] = rotated_cylinder_dir[i];
    }
    // 円柱の点群を回転
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    this->rotate_cloud_with_quat(this->cloud_cylinder_xy, rotated_cylinder, -theta_cylinder_and_x, rotation_axis);
    // 平面の点群を回転
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    this->rotate_cloud_with_quat(this->cloud_plane_xy, rotated_plane, -theta_cylinder_and_x, rotation_axis);
    
    // 共有線(楕円)の検出処理
    create_unit_vector(rotated_cylinder_dir);
    float theta_cylinder_and_z = acos(rotated_cylinder_dir[2]);
    float r = cylinder_radius;
    float a = r / cos(theta_cylinder_and_z); // 楕円の長半径
    float b = r; // 楕円の短半径

    // 共有線(楕円)
    const int PI = 3.1415;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intersect_ellipse_xy (new pcl::PointCloud<pcl::PointXYZRGB>);
    const int max_ellipse_point = 50;
    for (int i = 0; i < max_ellipse_point; i++) {
        pcl::PointXYZRGB temp_point;
        temp_point.x = a * cos(2 * PI * (float)i / (float)max_ellipse_point);
        temp_point.y = b * sin(2 * PI * (float)i / (float)max_ellipse_point);
        temp_point.z = 0.0f;
        intersect_ellipse_xy->push_back(temp_point);
    }

    copy_cloud(intersect_ellipse_xy, this->cloud_ellipse_xy);
    this->cloud_cylinder_xy->points.clear();
    copy_cloud(rotated_cylinder, this->cloud_cylinder_xy);
    this->cloud_plane_xy->points.clear();
    copy_cloud(rotated_plane, this->cloud_plane_xy);

    

    // //
    // // 描画処理
    // //
    // pcl::visualization::PCLVisualizer viewer("detect cylinder and plane");
    // // // 出力の円
    // // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> output_cloud_color_handler (cloud_circle_xy, 127, 255, 0); // 黄緑
    // // viewer.addPointCloud (cloud_circle_xy, output_cloud_color_handler, "output cloud");
    // // 検出された平面
    // pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_plane_color_handler (cloud_plane_xy, 102, 153, 204); // 青
    // viewer.addPointCloud (cloud_plane_xy, cloud_plane_color_handler, "cloud plane");
    // // // 検出された円柱
    // // pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_cylinder_color_handler (cloud_cylinder_xy, 204, 153, 102); // オレンジ
    // // viewer.addPointCloud (cloud_cylinder_xy, cloud_cylinder_color_handler, "cloud cylinder");
    // // viewer.addCylinder(this->cylinder_coeff_xy, "this_cylinder");
    // // 円柱の中心軸
    // const std::string line_str = "line_model";
    // viewer.addLine(line_coeff, line_str); 
    // // x軸
    // const std::string x_axis_str = "x_axis";
    // viewer.addLine(x_axis, x_axis_str);
    // // 回転後の中心軸
    // const std::string rotated_cylinder_line_str = "rotatd center axis";
    // viewer.addLine(rotated_cylinder_line, rotated_cylinder_line_str);
    // // 回転後の円柱点群
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rotated_cylinder_color_handler (rotated_cylinder, 255, 255, 0); // 紫
    // viewer.addPointCloud (rotated_cylinder, rotated_cylinder_color_handler, "rotated_cylinder_cloud");
    // // 共有線(楕円)
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> intersect_ellipse_xy_color_handler(intersect_ellipse_xy, 173, 255, 47); // 黄緑
    // viewer.addPointCloud(intersect_ellipse_xy, intersect_ellipse_xy_color_handler, "intersect_ellipse_xt");
    
    // while (!viewer.wasStopped()) {
    //     viewer.spinOnce();
    // }
}

// 円柱と平面、その交線(円)を検出
void detect_objects::detect_intersect_circle()
{
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_data (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder_data (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2 (new pcl::PointCloud<pcl::Normal>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_data;

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
    extract.filter (*cloud_plane_data);



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
    extract.filter (*cloud_cylinder_data);

    // クラスのもつplane, cylinderのデータにコピー
    copy_cloud(cloud_cylinder_data, this->cloud_cylinder);
    copy_cloud(cloud_plane_data, this->cloud_plane);

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
    cylinder_radius = r;

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
    rotate_cloud_with_quat(temp_circle, rotated_circle, theta_plane_and_circle, axis_plane_and_circle);
  
    // 平行移動処理
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intersect_cylinder_and_plane_with_rotation (new pcl::PointCloud<pcl::PointXYZRGB>);
    transfer_cloud(rotated_circle, intersect_cylinder_and_plane_with_rotation, IntersectPoint);

    copy_cloud(intersect_cylinder_and_plane_with_rotation, this->cloud_circle);
}

// 入力座標から偏角を、0~2PIの範囲で求める
float detect_objects::lead_rad(float* input_coordinate)
{
    const float PI = 3.1415;
    
    // PI/2の場合
    if (input_coordinate[0] == 0.0f) {
        if (input_coordinate[1] > 0) {
            return PI / 2;
        } else {
            return -PI / 2;
        }
    }

    float this_tan = input_coordinate[1] / input_coordinate[0];
    if ((input_coordinate[0] > 0) && (input_coordinate[1] > 0)) { // 第１象限にあるとき
        return atan(this_tan);
    } else if ((input_coordinate[0] > 0) && (input_coordinate[1] < 0)) { // 第４象限にあるとき
        return atan(this_tan) + (2 * PI);
    } else {
        return atan(this_tan) + PI;
    }
}


void detect_objects::swap(float& a, float& b)
{
	float temp = a;
	a = b;
	b = temp;
}

// 挿入法並び替え
void detect_objects::insertion(float* array, int begin, int end)
{
	int min_num = array[begin];
	int min_idx = begin;
	for (int i = begin; i <= end; i++) {
		if (array[i] < min_num) {
			min_num = array[i];
			min_idx = i;
		}
	}
	swap(array[begin], array[min_idx]); // �ŏ��l��擪�Ɏ����Ă���

	for (int i = begin + 1; i <= end; i++) {
		for (int j = i;; j--) {
			if (array[j] < array[j - 1]) {
				swap(array[j], array[j - 1]);
			}
			else {
				break;
			}
		}
	}
}


// 入力点群から円柱の見える範囲の円弧を抽出する
void detect_objects::detect_intersect_arc()
{
    detect_intersect_circle_at_xy();
    float r = cylinder_radius;
    
    // cylinderを構成する最大のzと最小のzを導出
    float min_z = cloud_cylinder_xy->points[0].z;
    float max_z = cloud_cylinder_xy->points[0].z;
    for (int i = 0; i < cloud_cylinder_xy->points.size(); i++) {
        if (cloud_cylinder_xy->points[i].z < min_z) {
            min_z = cloud_cylinder_xy->points[i].z;
        }
        if (cloud_cylinder_xy->points[i].z > max_z) {
            max_z = cloud_cylinder_xy->points[i].z;
        }
    }

    const int divide_num = 1000;
    float delta = (max_z - min_z) / divide_num;
    const int buf = 10;
    float max_theta_array[divide_num + buf];
    
    // 各点群をxy平面に射影したときのx軸との成す角を求める
    int theta_array_size = (int)cloud_cylinder_xy->points.size();
    float* theta_array = new float[theta_array_size];
    for (int i = 0; i < cloud_cylinder_xy->points.size(); i++) {
        // x軸との成す角を求める
        float this_coordinate[2];
        this_coordinate[0] = cloud_cylinder_xy->points[i].x;
        this_coordinate[1] = cloud_cylinder_xy->points[i].y;
        float this_theta = lead_rad(this_coordinate);
        theta_array[i] = this_theta;
    }

    // 並び替え
    insertion(theta_array, 0, theta_array_size - 1);
    // 偏角の最大値と最小値を配列の最大、最小の点５つの平均を用いて求める
    float sum_min_rad = 0.0f;
    for (int i = 0; i < 5; i++) {
        sum_min_rad += theta_array[i];
    }
    float min_rad = sum_min_rad / 5;
    float sum_max_rad = 0.0f;
    for (int i = 0; i < 5; i++) {
        sum_max_rad += theta_array[theta_array_size - i - 1];
    }
    float max_rad = sum_max_rad / 5;
    
    delete[] theta_array;


    // 偏角の範囲を考慮して円弧を描く
    const int max_point_num = 50;
    const float PI = 3.1415;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr arc_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < max_point_num; i++) {
        pcl::PointXYZRGB temp_point;
        temp_point.z = 0.0f;
        float this_theta = 2.0f * PI * (float)i / (float)max_point_num;
        if ((min_rad <= this_theta) && (this_theta <= max_rad)) {
            temp_point.x = r * cos(this_theta);
            temp_point.y = r * sin(this_theta);
            arc_cloud->push_back(temp_point);
        } 
    }
    
    ROS_INFO("max_rad: %f, min_rad: %f", max_rad, min_rad);

    cloud_arc_xy = arc_cloud;   
}


// ダウンサンプリングする
void detect_objects::downSampling(float voxel, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_input_data)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelSampler;
    voxelSampler.setInputCloud(cloud_input_data);
    voxelSampler.setLeafSize(voxel, voxel, voxel);
    voxelSampler.filter(*cloud_sampled);
    cloud_input_data.swap(cloud_sampled);
}

// 円柱の軸と平面との交点を求める
void detect_objects::detectIntersectLineAndPlane(float line[], float plane[], float* output)
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
void detect_objects::create_unit_vector(float* vec)
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

// input_cloudをdirectionの方向に平行移動し、output_cloudにする
void detect_objects::transfer_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud,
    float* direction)
{
    for (int i = 0; i < input_cloud->points.size(); i++) {
    pcl::PointXYZRGB temp_point;
    temp_point.x = input_cloud->points[i].x + direction[0];
    temp_point.y = input_cloud->points[i].y + direction[1];
    temp_point.z = input_cloud->points[i].z + direction[2];
    output_cloud->push_back(temp_point);
    }
}

// input_cloudをaxisを回転軸にしてthetaだけ回転してoutput_cloudにする
void detect_objects::rotate_cloud_with_quat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud,
    float theta,
    float* axis)
{
    // 回転処理
    for (int i = 0; i < input_cloud->points.size(); i++) {
        Eigen::VectorXf input_vec(3), output_vec(3);
        input_vec[0] = input_cloud->points[i].x;
        input_vec[1] = input_cloud->points[i].y;
        input_vec[2] = input_cloud->points[i].z;

        // 所定の回転軸で回転するQuaternionの生成
        Eigen::Quaternionf quat;
        Eigen::Vector3f axis_vector;
        for (int j = 0; j < 3; j++) {
            axis_vector[j] = axis[j];
        }
        quat = Eigen::AngleAxisf(theta, axis_vector);
        // 回転処理
        output_vec = quat * input_vec;
        pcl::PointXYZRGB temp_point;
        temp_point.x = output_vec[0];
        temp_point.y = output_vec[1];
        temp_point.z = output_vec[2];
        output_cloud->push_back(temp_point);
    }
}

// 入力点群cloudから平面と円柱を検出し、その共有線(円)を求める。
// ただし、平面がxy平面上に、交線となる円はその中心が原点になるように座標変換している。
void detect_objects::detect_intersect_circle_at_xy()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2 (new pcl::PointCloud<pcl::Normal>);


    // ------------------------------------
    // 入力点群から平面と円柱を検出する処理
    // ------------------------------------

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_data;
    
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

    // 円柱の中心点
    pcl::PointXYZ center_point;
    center_point.x = coefficients_cylinder->values[0];
    center_point.y = coefficients_cylinder->values[1];
    center_point.z = coefficients_cylinder->values[2];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_center_point (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_center_point->push_back (center_point);


    // 平面のモデル
    pcl::ModelCoefficients plane_coeff_fromdata;
    plane_coeff_fromdata.values.resize(4);
    for (int i = 0; i < 4; i++) {
        plane_coeff_fromdata.values[i] = coefficients_plane->values[i];
    }


    // -----------------------------------
    // 平面と円柱の交線(円弧)検出処理
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
    cylinder_radius = r;

    // x-y平面上の原点中心の円を生成
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_circle (new pcl::PointCloud<pcl::PointXYZRGB>); 
    for (int i = 0; i < max_circle_points; i++) {
        pcl::PointXYZRGB cloud_circle_point;
        cloud_circle_point.x = r * cos(2.0f * PI * (float)i / (float)max_circle_points);
        cloud_circle_point.y = r * sin(2.0f * PI * (float)i / (float)max_circle_points);
        cloud_circle_point.z = 0.0f;
        temp_circle->push_back(cloud_circle_point);
    }

    // 平面、円柱をその交点が原点に重なるようにそれぞれ平行移動
    float plane_cylinder_mode_direction[3];
    for (int i = 0; i < 3; i++) {
        plane_cylinder_mode_direction[i] = -IntersectPoint[i];
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfered_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    transfer_cloud(cloud_plane, transfered_plane, plane_cylinder_mode_direction);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfered_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    transfer_cloud(cloud_cylinder, transfered_cylinder, plane_cylinder_mode_direction);

    float plane_norm[3]; // 検出された平面の法線ベクトル
    for (int i = 0; i < 3; i++) {
        plane_norm[i] = plane[i];
    }
    create_unit_vector(plane_norm); // 大きさを１にする
    // 回転軸を外積で求める
    float axis_plane_and_circle[3];
    axis_plane_and_circle[0] = -plane_norm[1];
    axis_plane_and_circle[1] = plane_norm[0];
    axis_plane_and_circle[2] = 0.0f;
    create_unit_vector(axis_plane_and_circle); // 回転軸の大きさ１
    // 回転角を内積で求める
    float theta_plane_and_circle = acos(plane_norm[2]);
    // 回転処理
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    rotate_cloud_with_quat(transfered_plane, rotated_plane, -theta_plane_and_circle, axis_plane_and_circle);
    rotate_cloud_with_quat(transfered_cylinder, rotated_cylinder, -theta_plane_and_circle, axis_plane_and_circle);

    // 平面の数学モデルの法線ベクトルも同様に回転
    plane_coeff_fromdata.values[3] =0.0f;
    Eigen::VectorXf input_vec(3), output_vec(3);
    for (int i = 0; i < 3; i++) {
        input_vec[i] = plane_coeff_fromdata.values[i];
    }
    // 所定の回転軸で回転するQuaternionの生成
    Eigen::Quaternionf quat;
    Eigen::Vector3f axis_vector;
    for (int j = 0; j < 3; j++) {
        axis_vector[j] = axis_plane_and_circle[j];
    }
    quat = Eigen::AngleAxisf(-theta_plane_and_circle, axis_vector);
    // 回転処理
    output_vec = quat * input_vec;
    for (int i = 0; i < 3; i++) {
        plane_coeff_fromdata.values[i] = output_vec[i];
    }

    // 円柱のcoeffも回転させる
    input_vec[0] = coefficients_cylinder->values[3];
    input_vec[1] = coefficients_cylinder->values[4];
    input_vec[2] = coefficients_cylinder->values[5];
    output_vec = quat * input_vec;
    pcl::ModelCoefficients temp_cylinder_coeff;
    temp_cylinder_coeff.values.resize(7);
    for (int i = 0; i < 3; i++) {
        temp_cylinder_coeff.values[i] = 0.0f;
    }
    temp_cylinder_coeff.values[3] = output_vec[0];
    temp_cylinder_coeff.values[4] = output_vec[1];
    temp_cylinder_coeff.values[5] = output_vec[2];
    temp_cylinder_coeff.values[6] = r;


    copy_cloud(rotated_plane, cloud_plane_xy);
    copy_cloud(rotated_cylinder, cloud_cylinder_xy);
    copy_cloud(temp_circle, cloud_circle_xy);
    this->cylinder_coeff_xy = temp_cylinder_coeff;
}


// ----------------------------------------------

// float voxel_size(0.01);

// // ダウンサンプリングをする
// void downSampling(float voxel, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_input_data)
// {
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::VoxelGrid<pcl::PointXYZRGB> voxelSampler;
//     voxelSampler.setInputCloud(cloud_input_data);
//     voxelSampler.setLeafSize(voxel, voxel, voxel);
//     voxelSampler.filter(*cloud_sampled);
//     cloud_input_data.swap(cloud_sampled);
// }

// // 円柱の軸と平面との交点を求める
// void detectIntersectLineAndPlane(float line[], float plane[], float* output)
// {
//     float d[3]; // 直線の方向ベクトル
//     for (int i = 0; i < 3; i++) {
//         d[i] = line[i + 3];
//     }
//     float b[3]; // 直線の通る点
//     for (int i = 0; i < 3; i++) {
//         b[i] = line[i];
//     }
//     float n[3];
//     for (int i = 0; i < 3; i++) {
//         n[i] = plane[i];
//     }
//     float a4 = plane[3];

//     // 計算処理
//     // (a4- b*n / d*n) / d*n
//     float bn = 0;
//     for (int i = 0; i < 3; i++) {
//         bn += b[i] * n[i];
//     }
//     float dn = 0;
//     for (int i = 0; i < 3; i++) {
//         dn += d[i] * n[i];
//     }
//     float k = (-a4 - bn) / dn; // 係数

//     // 交点を導出
//     for (int i = 0; i < 3; i++) {
//         output[i] = b[i] + k * d[i];
//     }
// }

// // vecの大きさを１にする
// void create_unit_vector(float* vec)
// {
//     float abst = 0;
//     for (int i = 0; i < 3; i++) {
//         abst += vec[i] * vec[i];
//     }
//     abst = sqrt(abst);
//     for (int i = 0; i < 3; i++) {
//         vec[i] /= abst;
//     }
// }


// // input_cloudをdirectionの方向に平行移動し、output_cloudにする
// void transfer_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud,
//     float* direction)
// {
//     for (int i = 0; i < input_cloud->points.size(); i++) {
//     pcl::PointXYZRGB temp_point;
//     temp_point.x = input_cloud->points[i].x + direction[0];
//     temp_point.y = input_cloud->points[i].y + direction[1];
//     temp_point.z = input_cloud->points[i].z + direction[2];
//     output_cloud->push_back(temp_point);
//     }
// }

// // input_cloudをaxisを回転軸にしてthetaだけ回転してoutput_cloudにする
// void rotate_cloud_with_quat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud,
//     float theta,
//     float* axis)
// {
//     // 回転処理
//     for (int i = 0; i < input_cloud->points.size(); i++) {
//         Eigen::VectorXf input_vec(3), output_vec(3);
//         input_vec[0] = input_cloud->points[i].x;
//         input_vec[1] = input_cloud->points[i].y;
//         input_vec[2] = input_cloud->points[i].z;

//         // 所定の回転軸で回転するQuaternionの生成
//         Eigen::Quaternionf quat;
//         Eigen::Vector3f axis_vector;
//         for (int j = 0; j < 3; j++) {
//             axis_vector[j] = axis[j];
//         }
//         quat = Eigen::AngleAxisf(theta, axis_vector);
//         // 回転処理
//         output_vec = quat * input_vec;
//         pcl::PointXYZRGB temp_point;
//         temp_point.x = output_vec[0];
//         temp_point.y = output_vec[1];
//         temp_point.z = output_vec[2];
//         output_cloud->push_back(temp_point);
//     }
// }


// // 入力点群cloudから平面と円柱を検出し、その共有線(円)を求める。
// // ただし、平面がxy平面上に、交線となる円はその中心が原点になるように座標変換している。
// // @param cylinder_radius 円柱の半径
// void detect_intersect_circle_at_xy(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& intersect_circle, 
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_plane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cylinder,
//     float& cylinder_radius)
// {
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    
//     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2 (new pcl::PointCloud<pcl::Normal>);


//     // ------------------------------------
//     // 入力点群から平面と円柱を検出する処理
//     // ------------------------------------

//     // ダウンサンプリング
//     downSampling(0.005, cloud);

//     // 法線の計算
//     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
//     norm_est.setInputCloud (cloud);
//     norm_est.setKSearch (50);
//     norm_est.compute (*cloud_normals);

//     // 平面検出
//     pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients());
//     pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices());
//     pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
//     seg.setOptimizeCoefficients(true);
//     seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
//     seg.setNormalDistanceWeight(0.1);
//     seg.setMethodType(pcl::SAC_RANSAC);
//     seg.setMaxIterations(1000);
//     seg.setDistanceThreshold (0.02);
//     seg.setInputCloud (cloud);
//     seg.setInputNormals(cloud_normals);
//     seg.segment(*inliers_plane, *coefficients_plane);
//     std::cerr << "plane coefficients: " << *coefficients_plane << std::endl;

//     pcl::ExtractIndices<pcl::PointXYZRGB> extract;
//     extract.setInputCloud (cloud);
//     extract.setIndices (inliers_plane);
//     extract.setNegative(false);
//     extract.filter (*cloud_plane);

//     // // 出力平面点群にコピー
//     // for (int i = 0; i < cloud_plane->points.size(); i++) {
//     //     pcl::PointXYZRGB temp_point;
//     //     temp_point.x = cloud_plane->points[i].x;
//     //     temp_point.y = cloud_plane->points[i].y;
//     //     temp_point.z = cloud_plane->points[i].z;
//     //     output_plane->push_back(temp_point);
//     // }

//     extract.setNegative (true);
//     extract.filter (*cloud_filtered_2);
//     pcl::ExtractIndices<pcl::Normal> extract_normals;
//     extract_normals.setNegative (true);
//     extract_normals.setInputCloud (cloud_normals);
//     extract_normals.setIndices (inliers_plane);
//     extract_normals.filter (*cloud_normals_2);

//     // 円柱検出
//     pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients ());
//     pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices ());
//     seg.setOptimizeCoefficients (true);
//     seg.setModelType (pcl::SACMODEL_CYLINDER);
//     seg.setMethodType (pcl::SAC_RANSAC);
//     seg.setNormalDistanceWeight (0.1);
//     seg.setMaxIterations (10000);
//     seg.setDistanceThreshold (0.05);
//     seg.setRadiusLimits (0.04, 0.1);
//     seg.setInputCloud (cloud_filtered_2);
//     seg.setInputNormals (cloud_normals_2);
//     seg.segment (*inliers_cylinder, *coefficients_cylinder);
//     std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

//     extract.setInputCloud (cloud_filtered_2);
//     extract.setIndices (inliers_cylinder);
//     extract.setNegative (false);
//     extract.filter (*cloud_cylinder);

//     // // 出力円柱点群にコピー
//     // for (int i = 0; i < cloud_cylinder->points.size(); i++) {
//     //     pcl::PointXYZRGB temp_point;
//     //     temp_point.x = cloud_cylinder->points[i].x;
//     //     temp_point.y = cloud_cylinder->points[i].y;
//     //     temp_point.z = cloud_cylinder->points[i].z;
//     //     output_cylinder->push_back(temp_point);
//     // }

//     // 円柱の中心点
//     pcl::PointXYZ center_point;
//     center_point.x = coefficients_cylinder->values[0];
//     center_point.y = coefficients_cylinder->values[1];
//     center_point.z = coefficients_cylinder->values[2];
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_center_point (new pcl::PointCloud<pcl::PointXYZ>);
//     cloud_center_point->push_back (center_point);


//     // 平面のモデル
//     pcl::ModelCoefficients plane_coeff_fromdata;
//     plane_coeff_fromdata.values.resize(4);
//     for (int i = 0; i < 4; i++) {
//         plane_coeff_fromdata.values[i] = coefficients_plane->values[i];
//     }


//     // -----------------------------------
//     // 平面と円柱の交線(円弧)検出処理
//     // -----------------------------------

//     // 円の点群を作成
//     const int max_circle_points = 50;
//     const float PI = 3.1415;

//     // 直線と円柱の軸との交点を検出
//     float line[6]; // 0~2は直線が通る点、3~5は方向ベクトル
//     for (int i = 0; i < 6; i++) {
//         line[i] = coefficients_cylinder->values[i]; 
//     }
//     float plane[4]; // 0から順に、a, b, c, d(ax+by+cz+d=0に対応)
//     for (int i = 0;i < 4; i++) {
//         plane[i] = coefficients_plane->values[i];
//     }
//     float IntersectPoint[3];
//     // 実際の計算処理
//     detectIntersectLineAndPlane(line, plane, IntersectPoint);
//     // 点群データに変換
//     pcl::PointXYZRGB IntersectPoint_data;
//     IntersectPoint_data.x = IntersectPoint[0];
//     IntersectPoint_data.y = IntersectPoint[1];
//     IntersectPoint_data.z = IntersectPoint[2];
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr IntersectCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//     IntersectCloud->push_back(IntersectPoint_data);

//     float r = coefficients_cylinder->values[6]; // 円柱の半径
//     cylinder_radius = r;

//     // x-y平面上の原点中心の円を生成
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_circle (new pcl::PointCloud<pcl::PointXYZRGB>); 
//     for (int i = 0; i < max_circle_points; i++) {
//         pcl::PointXYZRGB cloud_circle_point;
//         cloud_circle_point.x = r * cos(2.0f * PI * (float)i / (float)max_circle_points);
//         cloud_circle_point.y = r * sin(2.0f * PI * (float)i / (float)max_circle_points);
//         cloud_circle_point.z = 0.0f;
//         temp_circle->push_back(cloud_circle_point);
//     }

//     // 平面、円柱をその交点が原点に重なるようにそれぞれ平行移動
//     float plane_cylinder_mode_direction[3];
//     for (int i = 0; i < 3; i++) {
//         plane_cylinder_mode_direction[i] = -IntersectPoint[i];
//     }
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfered_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
//     transfer_cloud(cloud_plane, transfered_plane, plane_cylinder_mode_direction);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfered_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
//     transfer_cloud(cloud_cylinder, transfered_cylinder, plane_cylinder_mode_direction);

//     float plane_norm[3]; // 検出された平面の法線ベクトル
//     for (int i = 0; i < 3; i++) {
//         plane_norm[i] = plane[i];
//     }
//     create_unit_vector(plane_norm); // 大きさを１にする
//     // 回転軸を外積で求める
//     float axis_plane_and_circle[3];
//     axis_plane_and_circle[0] = -plane_norm[1];
//     axis_plane_and_circle[1] = plane_norm[0];
//     axis_plane_and_circle[2] = 0.0f;
//     create_unit_vector(axis_plane_and_circle); // 回転軸の大きさ１
//     // 回転角を内積で求める
//     float theta_plane_and_circle = acos(plane_norm[2]);
//     // 回転処理
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
//     rotate_cloud_with_quat(transfered_plane, rotated_plane, -theta_plane_and_circle, axis_plane_and_circle);
//     rotate_cloud_with_quat(transfered_cylinder, rotated_cylinder, -theta_plane_and_circle, axis_plane_and_circle);

//     // 平面の数学モデルの法線ベクトルも同様に回転
//     plane_coeff_fromdata.values[3] =0.0f;
//     Eigen::VectorXf input_vec(3), output_vec(3);
//     for (int i = 0; i < 3; i++) {
//         input_vec[i] = plane_coeff_fromdata.values[i];
//     }
//     // 所定の回転軸で回転するQuaternionの生成
//     Eigen::Quaternionf quat;
//     Eigen::Vector3f axis_vector;
//     for (int j = 0; j < 3; j++) {
//         axis_vector[j] = axis_plane_and_circle[j];
//     }
//     quat = Eigen::AngleAxisf(-theta_plane_and_circle, axis_vector);
//     // 回転処理
//     output_vec = quat * input_vec;
//     for (int i = 0; i < 3; i++) {
//         plane_coeff_fromdata.values[i] = output_vec[i];
//     }

//     output_plane = rotated_plane;
//     output_cylinder = rotated_cylinder;
//     intersect_circle = temp_circle;

//     //
//     // 描画処理
//     //
//     // pcl::visualization::PCLVisualizer viewer("detected masses");

//     // // 平面
//     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> plane_color (cloud_plane, 102, 153, 204); // 青
//     // viewer.addPointCloud(rotated_plane, plane_color, "cloud_plane");
//     // // 円柱(点群データ)
//     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_cylinder_color (cloud_cylinder, 204, 153, 102); // オレンジ
//     // viewer.addPointCloud (rotated_cylinder, cloud_cylinder_color, "cloud_cylinder");
//     // // 円柱の中心
//     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_center_point_color_handler (cloud_center_point, 255, 215, 0);  // 黃
//     // viewer.addPointCloud<pcl::PointXYZ> (cloud_center_point, cloud_center_point_color_handler, "cloud_center_point");
//     // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cloud_center_point");
//     // // 円柱(数学モデル)
//     // // viewer.addCylinder(cylinder_coeff_fromdata, "cylinder_from_data");
//     // // 平面のモデルを描画
//     // viewer.addPlane(plane_coeff_fromdata, "plane_from_data");

//     // // 円柱の軸と平面との交点
//     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> intersect_color_handler (IntersectCloud, 0, 191, 255); // 青
//     // viewer.addPointCloud (IntersectCloud, intersect_color_handler, "intersect_point");
//     // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "intersect_point"); // 点のサイズを大きくする
//     // // 回答の交線回転、平行移動後
//     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> intersect_cylinder_and_plane_with_rotation_color_handler (temp_circle, 240, 255, 255); // 薄青
//     // viewer.addPointCloud (temp_circle, intersect_cylinder_and_plane_with_rotation_color_handler, "intersect color and plane with rotation");
    
//     // while (!viewer.wasStopped()) {
//     //     viewer.spinOnce();
//     // }
// }

// void detect_cylinder_and_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& intersect_circle, 
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_plane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cylinder)
// {
//     //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    
//     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2 (new pcl::PointCloud<pcl::Normal>);


//     // ダウンサンプリング
//     downSampling(0.005, cloud);

//     // 法線の計算
//     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
//     norm_est.setInputCloud (cloud);
//     norm_est.setKSearch (50);
//     norm_est.compute (*cloud_normals);

//     // 平面検出
//     pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients());
//     pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices());
//     pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
//     seg.setOptimizeCoefficients(true);
//     seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
//     seg.setNormalDistanceWeight(0.1);
//     seg.setMethodType(pcl::SAC_RANSAC);
//     seg.setMaxIterations(1000);
//     seg.setDistanceThreshold (0.02);
//     seg.setInputCloud (cloud);
//     seg.setInputNormals(cloud_normals);
//     seg.segment(*inliers_plane, *coefficients_plane);
//     std::cerr << "plane coefficients: " << *coefficients_plane << std::endl;

//     pcl::ExtractIndices<pcl::PointXYZRGB> extract;
//     extract.setInputCloud (cloud);
//     extract.setIndices (inliers_plane);
//     extract.setNegative(false);
//     extract.filter (*cloud_plane);

//     // 出力平面点群にコピー
//     for (int i = 0; i < cloud_plane->points.size(); i++) {
//         pcl::PointXYZRGB temp_point;
//         temp_point.x = cloud_plane->points[i].x;
//         temp_point.y = cloud_plane->points[i].y;
//         temp_point.z = cloud_plane->points[i].z;
//         output_plane->push_back(temp_point);
//     }

//     extract.setNegative (true);
//     extract.filter (*cloud_filtered_2);
//     pcl::ExtractIndices<pcl::Normal> extract_normals;
//     extract_normals.setNegative (true);
//     extract_normals.setInputCloud (cloud_normals);
//     extract_normals.setIndices (inliers_plane);
//     extract_normals.filter (*cloud_normals_2);

//     // 円柱検出
//     pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients ());
//     pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices ());
//     seg.setOptimizeCoefficients (true);
//     seg.setModelType (pcl::SACMODEL_CYLINDER);
//     seg.setMethodType (pcl::SAC_RANSAC);
//     seg.setNormalDistanceWeight (0.1);
//     seg.setMaxIterations (10000);
//     seg.setDistanceThreshold (0.05);
//     seg.setRadiusLimits (0.04, 0.1);
//     seg.setInputCloud (cloud_filtered_2);
//     seg.setInputNormals (cloud_normals_2);
//     seg.segment (*inliers_cylinder, *coefficients_cylinder);
//     std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

//     extract.setInputCloud (cloud_filtered_2);
//     extract.setIndices (inliers_cylinder);
//     extract.setNegative (false);
//     extract.filter (*cloud_cylinder);

//     // 出力円柱点群にコピー
//     for (int i = 0; i < cloud_cylinder->points.size(); i++) {
//         pcl::PointXYZRGB temp_point;
//         temp_point.x = cloud_cylinder->points[i].x;
//         temp_point.y = cloud_cylinder->points[i].y;
//         temp_point.z = cloud_cylinder->points[i].z;
//         output_cylinder->push_back(temp_point);
//     }

//     // 円柱の中心点
//     pcl::PointXYZ center_point;
//     center_point.x = coefficients_cylinder->values[0];
//     center_point.y = coefficients_cylinder->values[1];
//     center_point.z = coefficients_cylinder->values[2];
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_center_point (new pcl::PointCloud<pcl::PointXYZ>);
//     cloud_center_point->push_back (center_point);


//     // 円柱のモデル
//     // 微小な高さの円柱で円の代わりとする
//     pcl::ModelCoefficients cylinder_coeff_fromdata;
//     cylinder_coeff_fromdata.values.resize(7);
//     for (int i = 0; i < 7; i++) {
//         cylinder_coeff_fromdata.values[i] = coefficients_cylinder->values[i];
//     }

//     // 平面のモデル
//     pcl::ModelCoefficients plane_coeff_fromdata;
//     plane_coeff_fromdata.values.resize(4);
//     for (int i = 0; i < 4; i++) {
//         plane_coeff_fromdata.values[i] = coefficients_plane->values[i];
//     }


//     // -----------------------------------
//     // 平面と円柱の交線検出処理
//     // -----------------------------------

//     // 円の点群を作成
//     const int max_circle_points = 50;
//     const float PI = 3.1415;

//     // 直線と円柱の軸との交点を検出
//     float line[6]; // 0~2は直線が通る点、3~5は方向ベクトル
//     for (int i = 0; i < 6; i++) {
//         line[i] = coefficients_cylinder->values[i]; 
//     }
//     float plane[4]; // 0から順に、a, b, c, d(ax+by+cz+d=0に対応)
//     for (int i = 0;i < 4; i++) {
//         plane[i] = coefficients_plane->values[i];
//     }
//     float IntersectPoint[3];
//     // 実際の計算処理
//     detectIntersectLineAndPlane(line, plane, IntersectPoint);
//     // 点群データに変換
//     pcl::PointXYZRGB IntersectPoint_data;
//     IntersectPoint_data.x = IntersectPoint[0];
//     IntersectPoint_data.y = IntersectPoint[1];
//     IntersectPoint_data.z = IntersectPoint[2];
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr IntersectCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//     IntersectCloud->push_back(IntersectPoint_data);

//     float r = coefficients_cylinder->values[6]; // 円柱の半径

//     // x-z平面上の原点中心の円
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_circle (new pcl::PointCloud<pcl::PointXYZRGB>); 
//     for (int i = 0; i < max_circle_points; i++) {
//         pcl::PointXYZRGB cloud_circle_point;
//         cloud_circle_point.x = r * cos(2.0f * PI / (float)max_circle_points * (float)i);
//         cloud_circle_point.y = 0.0f;
//         cloud_circle_point.z = r * sin(2.0f * PI / (float)max_circle_points * (float)i);
//         temp_circle->push_back(cloud_circle_point);
//     }
//     float plane_norm[3]; // 検出された平面の法線ベクトル
//     for (int i = 0; i < 3; i++) {
//         plane_norm[i] = plane[i];
//     }
//     create_unit_vector(plane_norm); // 大きさを１にする
//     ROS_INFO("plane_norm: ");
//     for (int i = 0; i < 3; i++) {
//         ROS_INFO("%f ", plane_norm[i]);
//     }
//     // 回転軸を外積で求める
//     float axis_plane_and_circle[3];
//     axis_plane_and_circle[0] = plane_norm[2];
//     axis_plane_and_circle[1] = 0.0f;
//     axis_plane_and_circle[2] = -plane_norm[0];
//     create_unit_vector(axis_plane_and_circle);
//     // 回転角を内積で求める
//     float theta_plane_and_circle = acos(plane_norm[1]);
//     // 回転処理
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_circle (new pcl::PointCloud<pcl::PointXYZRGB>);
//     for (int i = 0; i < temp_circle->points.size(); i++) {
//         Eigen::VectorXf input_vec(3), output_vec(3);
//         input_vec[0] = temp_circle->points[i].x;
//         input_vec[1] = temp_circle->points[i].y;
//         input_vec[2] = temp_circle->points[i].z;

//         // 所定の回転軸で回転するQuaternionの生成
//         Eigen::Quaternionf quat;
//         Eigen::Vector3f axis;
//         for (int j = 0; j < 3; j++) {
//             axis[j] = axis_plane_and_circle[j];
//         }
//         quat = Eigen::AngleAxisf(theta_plane_and_circle, axis);
//         // 回転処理
//         output_vec = quat * input_vec;
//         pcl::PointXYZRGB temp_point;
//         temp_point.x = output_vec[0];
//         temp_point.y = output_vec[1];
//         temp_point.z = output_vec[2];
//         rotated_circle->push_back(temp_point);
//     }
//     // 平行移動処理
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr intersect_cylinder_and_plane_with_rotation (new pcl::PointCloud<pcl::PointXYZRGB>);
//     for (int i = 0; i < rotated_circle->points.size(); i++) {
//         pcl::PointXYZRGB temp_point;
//         temp_point.x = rotated_circle->points[i].x + IntersectPoint[0];
//         temp_point.y = rotated_circle->points[i].y + IntersectPoint[1];
//         temp_point.z = rotated_circle->points[i].z + IntersectPoint[2];
//         intersect_cylinder_and_plane_with_rotation->push_back(temp_point);
//     }

//     // 出力点群にコピー
//     for (int i = 0; i < intersect_cylinder_and_plane_with_rotation->points.size(); i++) {
//         pcl::PointXYZRGB temp_point;
//         temp_point.x = intersect_cylinder_and_plane_with_rotation->points[i].x;
//         temp_point.y = intersect_cylinder_and_plane_with_rotation->points[i].y;
//         temp_point.z = intersect_cylinder_and_plane_with_rotation->points[i].z;
//         intersect_circle->push_back(temp_point);
//     }
// }



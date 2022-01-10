

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

// 注意
// 余計な点群が多く含まれていると、ほしい平面が平面として
// 認識されなくなる

// Distance Thresholdの値が小さいと、正確に平面を認識してもらえない
// 確率が大きくなる

const int detection_thresh = 100;
void abs_one(double* norm)
{
    double sum = 0;
    for (int i = 0; i < 3; i++) {
        sum += norm[i] * norm[i];
    }
    double sq = sqrt(sum);
    for (int i = 0; i < 3; i++) {
        norm[i] /= sq;
    }
}

int color_buf[][3]= {
    {0, 255, 255},   // シアン
    {127, 255, 0},   // 緑
    {255, 165, 0},  // オレンジ
    {255, 20, 147},  // ピンク
    {205, 92, 92}, // 茶色
    {65, 105, 255}, // 青
};

void downSampling(float voxel, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_input_data)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelSampler;
    voxelSampler.setInputCloud(cloud_input_data);
    voxelSampler.setLeafSize(voxel, voxel, voxel);
    voxelSampler.filter(*cloud_sampled);
    cloud_input_data.swap(cloud_sampled);
}

void detect_vertical(pcl::PointCloud<pcl::PointXYZRGB>::Ptr default_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_data1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_data2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder_data (new pcl::PointCloud<pcl::PointXYZRGB>);
   
    
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2 (new pcl::PointCloud<pcl::Normal>);

    // ダウンサンプリング
    downSampling(0.005, default_cloud);

    // 法線の計算
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setInputCloud (default_cloud);
    norm_est.setKSearch (50);
    norm_est.compute (*cloud_normals);

    // 平面検出 その１
    pcl::ModelCoefficients::Ptr coefficients_plane1 (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers_plane1 (new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud (default_cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    // seg.setInputNormals(cloud_normals);
    seg.setDistanceThreshold (0.002);
    seg.segment(*inliers_plane1, *coefficients_plane1);
    std::cerr << "coefficients_plane1: " << *coefficients_plane1 << std::endl;

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (default_cloud);
    extract.setIndices (inliers_plane1);

    // 検出された平面のみをとりだす
    extract.setNegative(false);
    extract.filter (*cloud_plane_data1);
    // 検出された平面以外をとりだす
    extract.setNegative (true);
    extract.filter (*cloud_filtered_2);

    // pcl::ExtractIndices<pcl::Normal> extract_normals;
    // extract_normals.setNegative (true);
    // extract_normals.setInputCloud (cloud_normals);
    // extract_normals.setIndices (inliers_plane1);
    // extract_normals.filter (*cloud_normals_2);

    // 平面検出 その２
    pcl::ModelCoefficients::Ptr coefficients_plane2 (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers_plane2 (new pcl::PointIndices ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.005);
    // seg.setRadiusLimits (0.04, 0.1);
    seg.setInputCloud (cloud_filtered_2);
    // seg.setInputNormals (cloud_normals_2);
    seg.segment (*inliers_plane2, *coefficients_plane2);
    std::cerr << "coefficients_plane2: " << *coefficients_plane2 << std::endl;

    extract.setInputCloud (cloud_filtered_2);
    extract.setIndices (inliers_plane2);
    extract.setNegative (false);
    extract.filter (*cloud_plane_data2); // 平面を取り出す

    double norm1[3];
    double norm2[3];
    for (int i = 0; i < 3; i++) {
        norm1[i] = coefficients_plane1->values[i];
        norm2[i] = coefficients_plane2->values[i];
    }
    // 大きさを1にする
    double sum1 = 0;
    double sum2 = 0;
    for (int i = 0; i < 3; i++) {
        sum1 += norm1[i] * norm1[i];
        sum2 += norm2[i] * norm2[i];
    }
    double sq1 = sqrt(sum1);
    double sq2 = sqrt(sum2);
    for (int i = 0; i < 3; i++) {
        norm1[i] /= sq1;
        norm2[i] /= sq2;
    }
    // 直交性を調べる
    double iproduct = 0; // 内積
    for (int i = 0; i < 3; i++) {
        iproduct += norm1[i] * norm2[i];
    }
    double rad = acos(iproduct);
    double degree = 180.0 * rad / 3.1415;
    ROS_INFO("degree: %f", degree);


    

    // 描画処理
    pcl::visualization::PCLVisualizer viewer("check vertical");
    // 平面１
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> output_cloud_color_handler1 (cloud_plane_data1, 127, 255, 0); // 黄緑
    viewer.addPointCloud (cloud_plane_data1, output_cloud_color_handler1, "cloud plane1");   
    // 平面２
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> output_cloud_color_handler2 (cloud_plane_data2, 65, 105, 225); // 青
    viewer.addPointCloud (cloud_plane_data2, output_cloud_color_handler2, "cloud plane2");   
    // 除去された平面
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> filtered_color_handler (cloud_filtered_2, 255, 0, 0); // 赤
    // viewer.addPointCloud (cloud_filtered_2, filtered_color_handler, "filetered");
    // 検出された平面1
    viewer.addPlane(*coefficients_plane1, "plane1");
    // 検出された平面2
    viewer.addPlane(*coefficients_plane2, "plane2");
    // // 検出された平面
    // pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_plane_color_handler (cloud_cylinder_data, 102, 153, 204); // 青
    // viewer.addPointCloud (cloud_cylinder_data, cloud_plane_color_handler, "cloud cylinder");
    // 検出された円柱
    // pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_cylinder_color_handler (cloud_cylinder_xy, 204, 153, 102); // オレンジ
    // viewer.addPointCloud (cloud_cylinder_xy, cloud_cylinder_color_handler, "cloud cylinder");
    // viewer.addCylinder(*coefficients_cylinder, "this_cylinder");
    while(!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}

void detect_cylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_data)
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
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    // seg.setRadiusLimits (0.04, 0.1);
    seg.setInputCloud (cloud_filtered_2);
    seg.setInputNormals (cloud_normals_2);
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "plane2 coefficients: " << *coefficients_cylinder << std::endl;

    extract.setInputCloud (cloud_filtered_2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud_cylinder);

    // 描画処理
    pcl::visualization::PCLVisualizer viewer("check vertical");
    // 平面
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> output_cloud_color_handler1 (cloud_plane, 127, 255, 0); // 黄緑
    viewer.addPointCloud (cloud_plane, output_cloud_color_handler1, "cloud plane");   
    // 円柱
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> output_cloud_color_handler2 (cloud_cylinder, 65, 105, 225); // 青
    viewer.addPointCloud (cloud_cylinder, output_cloud_color_handler2, "cloud cylinder");   
    // 検出された平面1
    viewer.addPlane(*coefficients_plane, "plane1");
    // 検出された平面2
    viewer.addPlane(*coefficients_cylinder, "cylinder");
    // // 検出された平面
    // pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_plane_color_handler (cloud_cylinder_data, 102, 153, 204); // 青
    // viewer.addPointCloud (cloud_cylinder_data, cloud_plane_color_handler, "cloud cylinder");
    // 検出された円柱
    // pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_cylinder_color_handler (cloud_cylinder_xy, 204, 153, 102); // オレンジ
    // viewer.addPointCloud (cloud_cylinder_xy, cloud_cylinder_color_handler, "cloud cylinder");
    // viewer.addCylinder(*coefficients_cylinder, "this_cylinder");
    while(!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}


double detect_vertical_sum(pcl::PointCloud<pcl::PointXYZRGB>::Ptr default_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_3 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_data1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_data2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_data3 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder_data (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr> plane_clouds;
    std::map<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::ModelCoefficients::Ptr> plane_map;
   
    
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_2 (new pcl::PointCloud<pcl::Normal>);

    // ダウンサンプリング
    downSampling(0.005, default_cloud);

    // 法線の計算
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setInputCloud (default_cloud);
    norm_est.setKSearch (50);
    norm_est.compute (*cloud_normals);


    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = default_cloud;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    while (true) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane_data (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices());
        seg.setOptimizeCoefficients(true);
        seg.setInputCloud (cloud_filtered);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold (0.005);
        seg.segment(*inliers_plane, *coefficients_plane);
        std::cerr << "coefficients_plane: " << *coefficients_plane << std::endl;

        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers_plane);

        // 検出された平面のみをとりだす
        extract.setNegative(false);
        extract.filter (*cloud_plane_data);
        // 検出された平面以外をとりだす
        extract.setNegative (true);
        extract.filter (*cloud_filtered);
        // 点群数が基準に満たなかったら、検出終了
        if (cloud_plane_data->points.size() < detection_thresh) {
            break;
        }

        plane_clouds.push_back(cloud_plane_data);
        plane_map.insert(std::make_pair(cloud_plane_data, coefficients_plane));
    }

    ROS_INFO("cloud_num: %d", plane_clouds.size());
    for (int i = 0; i < plane_clouds.size(); i++) {
        auto this_cloud = plane_clouds[i];
        ROS_INFO("cloud size: %d: %d", i + 1, this_cloud->points.size());
    }

    // 2平面の角度を検出する
    double degree = 0;
    if (plane_map.size() == 2) {
        double norms[2][3];
        int k = 0;

        for (auto itr = plane_map.begin(); itr != plane_map.end(); itr++) {
            for (int i = 0; i < 3; i++) {
                norms[k][i] = itr->second->values[i];
            }
            k++;
        }
        for (int i = 0; i < 2; i++) {
            abs_one(norms[i]);
        }
        double iproduct = 0;
        for (int i = 0; i < 3; i++) {
            iproduct += norms[0][i] * norms[1][i];
        }
        degree = 180.0 * acos(iproduct) / 3.1415;

        ROS_INFO("degree: %f", degree);
    } else {
        // 複数平面が検出されたとき、トップ2大平面を検出対象とする。
        ROS_INFO("NOT two planes were detected");
        double norms[2][3];
        int k = 0;
        // 平面の法線ベクトル取り出し
        for (auto itr = plane_clouds.begin(); k < 2; itr++) {
            for (int i = 0; i < 3; i++) {
                // mapは並び替えられてしまうので、vectorでまわすこと。  
                auto this_plane_coeff = plane_map.find(*itr)->second;
                norms[k][i] = this_plane_coeff->values[i];
            }
            k++;

        }

        for (int i = 0; i < 2; i++) {
            abs_one(norms[i]);
        }
        double iproduct = 0;
        for (int i = 0; i < 3; i++) {
            iproduct += norms[0][i] * norms[1][i];
        }
        degree = 180.0 * acos(iproduct) / 3.1415;
        ROS_INFO("degree: %f", degree);
        // exit(1);
    }


    // 描画処理
    pcl::visualization::PCLVisualizer viewer("check vertical");


    for (int i = 0; i < plane_clouds.size(); i++) {
        auto this_cloud = plane_clouds[i];
        int* this_color = color_buf[i % 7];
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler (this_cloud, 
            this_color[0], this_color[1], this_color[2]); 
        std::string cloud_name = "cloud plane" + std::to_string(i + 1);
        viewer.addPointCloud(this_cloud, color_handler, cloud_name);
    }
    // {
    //     int k = 1;
    //     for (auto itr = plane_map.begin(); itr != plane_map.end(); itr++) {
    //         std::string plane_name = "plane" + std::to_string(k);
    //         viewer.addPlane(*itr->second, plane_name);itr->first->points.size();
    //         k++;
    //     }
    // }
    
    while(!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return degree;
}

std::vector<double> detect_vertical_sum_clouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds)
{
    std::vector<double> degrees;
    for (auto itr = clouds.begin(); itr != clouds.end(); itr++) {
        degrees.push_back(detect_vertical_sum(*itr));
    }
    return degrees;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "check_vertical");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string pcd_file_name;
    pnh.getParam("pcd_name", pcd_file_name);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr two_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_and_plane(new pcl::PointCloud<pcl::PointXYZRGB>);

    // // 点群データ読み出し
    // std::string filepath = ros::package::getPath("wander_cloud") + "/data/" + "two_plane2_segmented.pcd";
    // pcl::io::loadPCDFile(filepath, *two_plane);
    // filepath = ros::package::getPath("wander_cloud") + "/data/" + "cylinder_and_plane.pcd";
    // pcl::io::loadPCDFile(filepath, *cylinder_and_plane);

    // detect_vertical(two_plane);
    // detect_cylinder(two_plane);
    ROS_INFO("pcd_name: %s", pcd_file_name.c_str());

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    for (int i = 1; i <= 5; i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        std::string file_name = ros::package::getPath("wander_cloud") + "/data/" + pcd_file_name + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(file_name, *temp_cloud);
        clouds.push_back(temp_cloud);
    }

#ifdef TRUE_AVE
    std::vector<double> degree_aves;
    for (int i = 0; i < 10; i++) {
        auto degrees = detect_vertical_sum_clouds(clouds);
        double degree_ave = 0;
        for (auto itr = degrees.begin(); itr != degrees.end(); itr++) {
            ROS_INFO("degree: %f", *itr);
            degree_ave += *itr;
        }
        degree_ave /= degrees.size();
        ROS_INFO("degree ave: %f", degree_ave);
        degree_aves.push_back(degree_ave);
    }
    double true_ave = 0;
    for (auto itr = degree_aves.begin(); itr != degree_aves.end(); itr++) {
        true_ave += *itr;
    }
    true_ave /= 10;
    ROS_INFO("true_ave: %f", true_ave);

#else
    auto degrees = detect_vertical_sum_clouds(clouds);
    double degree_ave = 0;
    for (auto itr = degrees.begin(); itr != degrees.end(); itr++) {
        ROS_INFO("degree: %f", *itr);
        degree_ave += *itr;
    }
    degree_ave /= degrees.size();
    ROS_INFO("degree ave: %f", degree_ave);
#endif

    ros::shutdown();
}
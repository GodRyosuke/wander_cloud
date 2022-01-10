#include "detect_cylinder.hpp"


pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);


// void downSampling(float voxel, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_input_data)
// {
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::VoxelGrid<pcl::PointXYZRGB> voxelSampler;
//     voxelSampler.setInputCloud(cloud_input_data);
//     voxelSampler.setLeafSize(voxel, voxel, voxel);
//     voxelSampler.filter(*cloud_sampled);
//     cloud_input_data.swap(cloud_sampled);
// }

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


// // 入力点群cloudから平面と円柱を検出し、その共有線を求める。
// // ただし、平面がxy平面上に、交線となる円はその中心が原点になるように座標変換している。
// void detect_intersect_arc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& intersect_circle, 
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_plane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cylinder)
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


//     //
//     // 描画処理
//     //
//     pcl::visualization::PCLVisualizer viewer("detected masses");

//     // 平面
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> plane_color (cloud_plane, 102, 153, 204); // 青
//     viewer.addPointCloud(rotated_plane, plane_color, "cloud_plane");
//     // 円柱(点群データ)
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_cylinder_color (cloud_cylinder, 204, 153, 102); // オレンジ
//     viewer.addPointCloud (rotated_cylinder, cloud_cylinder_color, "cloud_cylinder");
//     // 円柱の中心
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_center_point_color_handler (cloud_center_point, 255, 215, 0);  // 黃
//     viewer.addPointCloud<pcl::PointXYZ> (cloud_center_point, cloud_center_point_color_handler, "cloud_center_point");
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cloud_center_point");
//     // 円柱(数学モデル)
//     // viewer.addCylinder(cylinder_coeff_fromdata, "cylinder_from_data");
//     // 平面のモデルを描画
//     viewer.addPlane(plane_coeff_fromdata, "plane_from_data");

//     // 円柱の軸と平面との交点
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> intersect_color_handler (IntersectCloud, 0, 191, 255); // 青
//     viewer.addPointCloud (IntersectCloud, intersect_color_handler, "intersect_point");
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "intersect_point"); // 点のサイズを大きくする
//     // 回答の交線回転、平行移動後
//     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> intersect_cylinder_and_plane_with_rotation_color_handler (temp_circle, 240, 255, 255); // 薄青
//     viewer.addPointCloud (temp_circle, intersect_cylinder_and_plane_with_rotation_color_handler, "intersect color and plane with rotation");
    
//     while (!viewer.wasStopped()) {
//         viewer.spinOnce();
//     }
// }


// ---------------------------------------------

// // 入力座標から偏角を、0~2PIの範囲で求める
// float lead_rad(float* input_coordinate)
// {
//     const float PI = 3.1415;
    
//     // PI/2の場合
//     if (input_coordinate[0] == 0.0f) {
//         if (input_coordinate[1] > 0) {
//             return PI / 2;
//         } else {
//             return -PI / 2;
//         }
//     }

//     float this_tan = input_coordinate[1] / input_coordinate[0];
//     if ((input_coordinate[0] > 0) && (input_coordinate[1] > 0)) { // 第１象限にあるとき
//         return atan(this_tan);
//     } else if ((input_coordinate[0] > 0) && (input_coordinate[1] < 0)) { // 第４象限にあるとき
//         return atan(this_tan) + (2 * PI);
//     } else {
//         return atan(this_tan) + PI;
//     }
// }


// void swap(float& a, float& b)
// {
// 	float temp = a;
// 	a = b;
// 	b = temp;
// }

// // 挿入法並び替え
// void insertion(float* array, int begin, int end)
// {
// 	int min_num = array[begin];
// 	int min_idx = begin;
// 	for (int i = begin; i <= end; i++) {
// 		if (array[i] < min_num) {
// 			min_num = array[i];
// 			min_idx = i;
// 		}
// 	}
// 	swap(array[begin], array[min_idx]); // �ŏ��l��擪�Ɏ����Ă���

// 	for (int i = begin + 1; i <= end; i++) {
// 		for (int j = i;; j--) {
// 			if (array[j] < array[j - 1]) {
// 				swap(array[j], array[j - 1]);
// 			}
// 			else {
// 				break;
// 			}
// 		}
// 	}
// }

// // 入力点群から円柱の見える範囲の円弧を抽出する
// void detect_intersect_arc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_plane,
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cylinder,
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_arc)
// {
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr circle_data (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_data (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_data (new pcl::PointCloud<pcl::PointXYZRGB>);

//     float r;
//     detect_intersect_circle_at_xy(input_cloud, circle_data, plane_data, cylinder_data, r);

//     // cylinderを構成する最大のzと最小のzを導出
//     float min_z = cylinder_data->points[0].z;
//     float max_z = cylinder_data->points[0].z;
//     for (int i = 0; i < cylinder_data->points.size(); i++) {
//         if (cylinder_data->points[i].z < min_z) {
//             min_z = cylinder_data->points[i].z;
//         }
//         if (cylinder_data->points[i].z > max_z) {
//             max_z = cylinder_data->points[i].z;
//         }
//     }

//     const int divide_num = 1000;
//     float delta = (max_z - min_z) / divide_num;
//     const int buf = 10;
//     float max_theta_array[divide_num + buf];
    
//     // 各点群をxy平面に射影したときのx軸との成す角を求める
//     int theta_array_size = (int)cylinder_data->points.size();
//     float* theta_array = new float[theta_array_size];
//     for (int i = 0; i < cylinder_data->points.size(); i++) {
//         // x軸との成す角を求める
//         float this_coordinate[2];
//         this_coordinate[0] = cylinder_data->points[i].x;
//         this_coordinate[1] = cylinder_data->points[i].y;
//         float this_theta = lead_rad(this_coordinate);
//         theta_array[i] = this_theta;
//     }

//     // 並び替え
//     insertion(theta_array, 0, theta_array_size - 1);
//     // 偏角の最大値と最小値を配列の最大、最小の点５つの平均を用いて求める
//     float sum_min_rad = 0.0f;
//     for (int i = 0; i < 5; i++) {
//         sum_min_rad += theta_array[i];
//     }
//     float min_rad = sum_min_rad / 5;
//     float sum_max_rad = 0.0f;
//     for (int i = 0; i < 5; i++) {
//         sum_max_rad += theta_array[theta_array_size - i - 1];
//     }
//     float max_rad = sum_max_rad / 5;
    
//     delete[] theta_array;


//     // 偏角の範囲を考慮して円弧を描く
//     const int max_point_num = 50;
//     const float PI = 3.1415;
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr arc_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//     for (int i = 0; i < max_point_num; i++) {
//         pcl::PointXYZRGB temp_point;
//         temp_point.z = 0.0f;
//         float this_theta = 2.0f * PI * (float)i / (float)max_point_num;
//         if ((min_rad <= this_theta) && (this_theta <= max_rad)) {
//             temp_point.x = r * cos(this_theta);
//             temp_point.y = r * sin(this_theta);
//             arc_cloud->push_back(temp_point);
//         } 
//     }
    
//     ROS_INFO("max_rad: %f, min_rad: %f", max_rad, min_rad);

//     output_cylinder = cylinder_data;
//     output_plane = plane_data;
//     output_arc = arc_cloud;   
// }

pcl::PointCloud<pcl::PointXYZRGB>::Ptr circle_cloud_from_camera (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud_from_camera (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_cloud_from_camera (new pcl::PointCloud<pcl::PointXYZRGB>);


void detect_cylinder_from_realsense(const sensor_msgs::PointCloud2Ptr& input)
{
    pcl::fromROSMsg(*input, *camera_cloud);
    // 検出処理
    float r;
    // detect_intersect_circle_at_xy(camera_cloud, circle_cloud_from_camera, plane_cloud_from_camera, cylinder_cloud_from_camera, r);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv,  "wander_detect_cylinder");
    ros::NodeHandle nh;

    // realsense からのデータを得る
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr default_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intersect_carves (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZRGB>);

    // std::string filepath = ros::package::getPath("wander_cloud") + "/data/" + "wander_cylinder_and_plane.pcd";
    // pcl::io::loadPCDFile(filepath, *default_cloud);
    // detect_cylinder_and_plane(default_cloud, intersect_circle, cloud_plane, cloud_cylinder);

    // ros::Subscriber realsense_subs = nh.subscribe("/camera/depth_registered/points", 1, detect_cylinder_from_realsense);

    // 点群データ読み出し
    std::string filepath = ros::package::getPath("wander_cloud") + "/data/" + "cylinder_diagonal.pcd";
    if (argc == 2) {
        filepath = argv[1];
    }
    pcl::io::loadPCDFile(filepath, *default_cloud);

    // 検出処理
    detect_objects* detect_handler = new detect_objects(default_cloud, 0.01f);
    detect_handler->detect_ellipse_arc();

    // intersect_circle = detect_handler->get_circle();
    // cloud_plane = detect_handler->get_plane();
    // cloud_cylinder = detect_handler->get_cylinder();
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
    // xy平面
    pcl::ModelCoefficients plane_math_model;
    plane_math_model.values.resize(4);
    plane_math_model.values[0] = 0;
    plane_math_model.values[1] = 0;
    plane_math_model.values[2] = 1;
    plane_math_model.values[3] = 0;
    //viewer.addPlane(plane_math_model, "plane_math_model");
    
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
    
    // // rvizに渡すためのPublisher
    // ros::Publisher cloud_cylinder_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_cylinder", 5);
    // sensor_msgs::PointCloud2 ros_cylinder;
    // pcl::toROSMsg(*cloud_cylinder, ros_cylinder);
    // ros_cylinder.header.frame_id = "camera_color_frame";

    // ros::Publisher cloud_plane_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_plane", 1);
    // sensor_msgs::PointCloud2 ros_plane;
    // pcl::toROSMsg(*cloud_plane, ros_plane);
    // ros_plane.header.frame_id = "camera_color_frame";

    // ros::Publisher cloud_circle_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_circle", 1);
    // sensor_msgs::PointCloud2 ros_circle;
    // pcl::toROSMsg(*intersect_circle, ros_circle);
    // ros_circle.header.frame_id = "camera_color_frame";

    // // ros::Publisher test_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_test_pub", 5);
    // // sensor_msgs::PointCloud2 ros_test_cloud;
    // // pcl::toROSMsg(*intersect_circle, ros_test_cloud);
    // // ros_test_cloud.header.frame_id = "camera_color_frame";

    // // ros::Rate loop_rate(1);
    // // while (ros::ok()) {
    // //     // cloud_publisher.publish(msg);
    // //     //test_pub.publish(ros_test_cloud);
    // //     cloud_cylinder_publisher.publish(ros_cylinder);
    // //     cloud_plane_publisher.publish(ros_plane);
    // //     cloud_circle_publisher.publish(ros_circle);
        
    // //     ros::spinOnce();
    // //     loop_rate.sleep();
    // // }

    // //
    // // 描画処理
    // //
    // pcl::visualization::PCLVisualizer viewer("detect cylinder and plane");
    // // // 入力点群
    // // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> input_cloud_color_handler (input_cloud, 0, 191, 255); // 青
    // // viewer.addPointCloud (input_cloud, input_cloud_color_handler, "input cloud");
    // // 出力の円
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> output_cloud_color_handler (intersect_circle, 127, 255, 0); // 黄緑
    // viewer.addPointCloud (intersect_circle, output_cloud_color_handler, "output cloud");
    // // 検出された平面
    // pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_plane_color_handler (cloud_plane, 102, 153, 204); // 青
    // viewer.addPointCloud (cloud_plane, cloud_plane_color_handler, "cloud plane");
    // // 検出された円柱
    // pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGB> cloud_cylinder_color_handler (cloud_cylinder, 204, 153, 102); // オレンジ
    // viewer.addPointCloud (cloud_cylinder, cloud_cylinder_color_handler, "cloud cylinder");
    // while (!viewer.wasStopped()) {
    //     viewer.spinOnce();
    // }



    return 0;
}
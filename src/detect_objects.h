#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

//
// 物体と平面との交線導出インターフェース
//

class detect_objects
{
public:
    detect_objects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, float voxel = 0.01f);
    
    // 円がxy平面上にないときの円柱と平面との交線検出
    void detect_intersect_circle();
    // 入力点群cloudから平面と円柱を検出し、その共有線(円)を求める。
    // ただし、平面がxy平面上に、交線となる円はその中心が原点になるように座標変換している。
    bool detect_intersect_circle_at_xy();
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
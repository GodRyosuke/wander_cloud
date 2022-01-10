#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_opencv");
    ros::NodeHandle nh;

    // 画像の読み込み処理
    std::string file_dir = ros::package::getPath("wander_cloud") + "/img/";
    std::string input_file_path = file_dir + "testImage1.jpg";
    cv::Mat source_image = cv::imread(input_file_path, cv::IMREAD_GRAYSCALE);
    // 画像の描画処理
    cv::imshow("image", source_image);
    cv::waitKey();
    // 画像の書き込み処理
    std::string output_file_path = file_dir + "capture.jpg";
    cv::imwrite(output_file_path, source_image);

    ros::shutdown();
    return 0;
}
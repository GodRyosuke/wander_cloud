#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "wander_create_2value_img");
    ros::NodeHandle nh;
    ros::Rate looprate(10);

    std::string file_path = ros::package::getPath("wander_cloud") + "/img/testImage1.jpg";
    cv::Mat default_image = cv::imread(file_path, cv::IMREAD_COLOR);
    cv::Mat gray_image;
    cv::cvtColor(default_image, gray_image, CV_BGR2GRAY); // グレースケールに変換
    // 閾値を設定し、それを超えるピクセルはmaxValに設定
    cv::Mat bin_img;
    cv::threshold(gray_image, bin_img, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    // 画像の表示
    cv::namedWindow("wander_two_value_img", CV_WINDOW_AUTOSIZE);

    while (ros::ok()) {
        // if (cv::waitKey(1) == 'q') {
        //     break;
        // }
        if (cv::waitKey(1) == 's') {
            std::string save_file_name = ros::package::getPath("wander_cloud") + "/img/two_value_img.jpg";
            cv::imwrite(save_file_name, bin_img);
            break;
        }
        cv::imshow("wander_two_value_img", bin_img);

        ros::spinOnce();
        looprate.sleep();
    }


    cv::destroyWindow("wander_two_value_img");
    ros::shutdown();
    return 0;
}   
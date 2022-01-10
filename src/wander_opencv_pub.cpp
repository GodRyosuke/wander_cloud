#include <time.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

image_transport::Publisher image_pub_drawn;
int circle_center_idx = 0;

void writeC(const sensor_msgs::ImageConstPtr& msg) // subscribeされるたびに呼び出し
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // 円の描画
    const double PI = 3.1415;
    int crcle_r = 20;
    int B_circle_r = 300;
    int center_x = (int)(cv_ptr->image.cols / 2);
    int center_y = (int)(cv_ptr->image.rows / 2);
    const int max_N = 50;

    // 中心center_x, center_y、半径B_circle_rの円
    int circle_x = B_circle_r * cos(2 * PI * circle_center_idx / max_N) + center_x;
    int circle_y = B_circle_r * sin(2 * PI * circle_center_idx / max_N) + center_y;
    circle_center_idx++;

    cv::circle(cv_ptr->image, cv::Point(circle_x, circle_y), crcle_r, CV_RGB(124, 252, 0), -1);
    cv::circle(cv_ptr->image, cv::Point(center_x, center_y), crcle_r, CV_RGB(138, 43, 226), -1);

    cv::imshow("WanderTest", cv_ptr->image);
    image_pub_drawn.publish(cv_ptr->toImageMsg()); // 加工後のデータをPublish
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wander_opencv_pub");
    ros::NodeHandle nh;
    ros::Rate looprate(10);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub_ori = it.advertise("/river_picture", 1); // 元画像をpublishするpublisher
    image_transport::Subscriber image_sub = it.subscribe("/river_picture", 1, writeC); // publishされた元画像を受け取るsubscriber
    image_pub_drawn = it.advertise("/river_picture/drawn", 1); // 加工した画像をPublishするpublisher

    cv::namedWindow("WanderTest", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);

    // 加工前の元データを読み込む
    std::string file_path = ros::package::getPath("wander_cloud") + "/img/testImage1.jpg";
    cv::Mat default_image = cv::imread(file_path, cv::IMREAD_COLOR);
    sensor_msgs::ImagePtr default_pic_msg = cv_bridge::CvImage(std_msgs::Header(),
     "bgr8", default_image).toImageMsg(); // もとの画像データをpublishするためのmsg
    
    while (ros::ok()) {
        if (cv::waitKey(1) == 'q') {
            break;
        }
        image_pub_ori.publish(default_pic_msg); // 元データをPublish

        ros::spinOnce();
        looprate.sleep();
    }
    
    cv::destroyWindow("WanderTest");
    ros::shutdown();
    return 0;
}
#include <time.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

const std::string OPENCV_WINDOW = "ImageWindow";

class ImageConverter {
public:
    ImageConverter(ros::NodeHandle& nh)
    {
        image_transport::ImageTransport it(nh);
        image_sub_ = it.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
        image_pub_ori_ = it.advertise("/camera/image_raw", 1);
        image_pub_drawn_ = it.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void publishReadImage()
    {
        // 画像を読み込む
        std::string file_path = ros::package::getPath("wander_cloud") + "/img/testImage1.jpg";
        cv::Mat color_image = cv::imread(file_path, cv::IMREAD_COLOR);
        // publishするメッセージ
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
        image_pub_ori_.publish(msg);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        // メッセージから送られてきた画像データ
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 円の描画
        cv::RNG rng(clock());
        int circle_r = 20; // 半径20の円
        // 円の中心位置を乱数で決める
        int rand_x = (int)rng.uniform(0, cv_ptr->image.cols - circle_r * 2);
        int rand_y = (int)rng.uniform(0, cv_ptr->image.rows - circle_r * 2);
        cv::circle(cv_ptr->image, cv::Point(rand_x, rand_y), circle_r, CV_RGB(255, 0, 0), -1);

        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        image_pub_drawn_.publish(cv_ptr->toImageMsg());
    }

private:
    ros::NodeHandle nh_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_ori_;
    image_transport::Publisher image_pub_drawn_;
};

int main(int argc, char** argv)
{
    ros::init (argc, argv, "opencv_test_pub");
    ros::NodeHandle nh;
    ImageConverter ic(nh);

    ros::Rate looprate (5);
    while (ros::ok()) {
        if (cv::waitKey(1) == 'q') {
            break;
        }

        ic.publishReadImage();
        ros::spinOnce();
        looprate.sleep();
    }

    return 0;
}
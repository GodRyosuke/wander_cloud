#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// 注意!!
// 顔認識処理に渡す画像はそこそこ大きめの画像でないと、圧縮して処理するため、
// コンピュータに正しく認識&処理してもらえない

// #define USE_PC_CAMERA true

image_transport::Publisher detected_image_pub;
int circle_center_idx = 0;

bool detect_face(cv::Mat input_img, cv::Mat& output_img)
{
    if (input_img.empty()) {
        ROS_INFO("error: failed to default image");
        return false;
    }
    output_img = input_img;

    double scale = 4.0;
    cv::Mat gray_img,
     smallImg(cv::saturate_cast<int>(input_img.rows/scale), cv::saturate_cast<int>(input_img.cols/scale), CV_8UC1);
    // グレースケールに変換
    cv::cvtColor(input_img, gray_img, CV_BGR2GRAY);
    // 画像縮小(処理時間短縮)
    cv::resize(gray_img, smallImg, smallImg.size(), 0, 0, cv::INTER_LINEAR);
    cv::equalizeHist(smallImg, smallImg);

    // 顔認識のための分類機読み込み
    std::string cascadeName = ros::package::getPath("wander_cloud") + "/FeatureDetector/haarcascade_frontalface_alt.xml";
    cv::CascadeClassifier cascade;
    if (!cascade.load(cascadeName)) {
        ROS_INFO("error: failed to load cascade data");
        return false;
    }

    // 顔認識処理をする
    std::vector<cv::Rect> faces;
    cascade.detectMultiScale(smallImg, faces, 1.1, 2, 
     CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
    // ROS_INFO("faces array size: %d", (int)faces.size());

    // 結果の描画(元画像に検出した顔のところに円を貼り付ける)
    for (auto itr = faces.begin(); itr != faces.end(); itr++) {
        cv::Point center;
        int radius;
        center.x = cv::saturate_cast<int>((itr->x + itr->width * 0.5) * scale);
        center.y = cv::saturate_cast<int>((itr->y + itr->height * 0.5) * scale);
        radius = cv::saturate_cast<int>((itr->width + itr->height) * 0.25 * scale);
        cv::circle(output_img, center, radius, cv::Scalar(80, 80, 255), 3, 8, 0);
    }

    return true;
}

void detect_face_realsense(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImage output_image_ptr;

    cv::Mat input_img = cv_ptr->image;
    double scale = 4.0;
    cv::Mat gray_img,
     smallImg(cv::saturate_cast<int>(input_img.rows/scale), cv::saturate_cast<int>(input_img.cols/scale), CV_8UC1);
    // グレースケールに変換
    cv::cvtColor(input_img, gray_img, CV_BGR2GRAY);
    // 画像縮小(処理時間短縮)
    cv::resize(gray_img, smallImg, smallImg.size(), 0, 0, cv::INTER_LINEAR);
    cv::equalizeHist(smallImg, smallImg);

    // 顔認識のための分類機読み込み
    std::string cascadeName = ros::package::getPath("wander_cloud") + "/FeatureDetector/haarcascade_frontalface_alt.xml";
    cv::CascadeClassifier cascade;
    if (!cascade.load(cascadeName)) {
        ROS_INFO("error: failed to load cascade data");
        return;
    }

    // 顔認識処理をする
    std::vector<cv::Rect> faces;
    cascade.detectMultiScale(smallImg, faces, 1.1, 2, 
     CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
    // ROS_INFO("faces array size: %d", (int)faces.size());

    // 結果の描画(元画像に検出した顔のところに円を貼り付ける)
    for (auto itr = faces.begin(); itr != faces.end(); itr++) {
        cv::Point center;
        int radius;
        center.x = cv::saturate_cast<int>((itr->x + itr->width * 0.5) * scale);
        center.y = cv::saturate_cast<int>((itr->y + itr->height * 0.5) * scale);
        radius = cv::saturate_cast<int>((itr->width + itr->height) * 0.25 * scale);
        cv::circle(cv_ptr->image, center, radius, cv::Scalar(80, 80, 255), 3, 8, 0);
    }


    detected_image_pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wander_detect_face");
    ros::NodeHandle nh;
    ros::Rate looprate(10);

    // pcカメラを使う
    #ifdef USE_PC_CAMERA
    // カメラを開く
    cv::VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1200);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 800);
    if (!cap.isOpened()) {
        ROS_INFO("error: failed to open camera");
        return -1;
    }
    cv::namedWindow("capture", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    while (ros::ok()) {
        cv::Mat frame_img;
        cv::Mat detected_img;
        cap >> frame_img;
        if (!detect_face(frame_img, detected_img)) { // カメラ画像について、顔認識
            ROS_INFO("error: failed to detect face");
            return -1;
        }
        cv::imshow("capture", detected_img);
        if (cv::waitKey(1) == 'q') {
            break;
        }

        ros::spinOnce();
        looprate.sleep();
    }

    #else


    // realsense から得た画像データを使う
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber raelsense_sub = it.subscribe(
        "/camera/color/image_raw", 1, detect_face_realsense);
    detected_image_pub = it.advertise("/wander_detect_face", 1);

    ros::spin();

    #endif


    // realsense camera から画像データ取得
    // ros::Subscriber realsense_sub = nh.subscribe()

    // while (1) {
    //     cv::Mat frame_img;
    //     cap >> frame_img;
    //     cv::imshow("capture", frame_img);
    //     if (cv::waitKey(1) == 'q') {
    //         break;
    //     }
    // }

    // // 画像元データ読み込み
    // std::string file_dir = ros::package::getPath("wander_cloud") + "/img/";
    // std::string input_file_name = file_dir + "woman.jpg";
    // cv::Mat default_img = cv::imread(input_file_name);
    // cv::Mat detected_img;
    // if (!detect_face(default_img, detected_img)) {
    //     ROS_INFO("error: failed to detect face");
    //     return -1;
    // }
    

    // cv::namedWindow("detected_faces", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
    // while (ros::ok()) {
    //     cv::imshow("detected_faces", detected_img);
    //     if (cv::waitKey(1) == 'q') {
    //         break;
    //     }
    //     ros::spinOnce();
    //     looprate.sleep();
    // }

    // cv::destroyWindow("detected_faces");

    
    ros::shutdown();
    return 0;
}
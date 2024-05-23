#ifndef SRC_CAMERA_DETECTION_H
#define SRC_CAMERA_DETECTION_H

#include <fstream>
#include <thread>
// #include <mutex>
// #include <pthread.h>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.h>
// #include <sensor_msgs/msg/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "mmr_base/msg/bounding_box.hpp"
#include "mmr_base/msg/bounding_boxes.hpp"
#include "../tkDNN/include/tkDNN/DarknetParser.h"
#include "../tkDNN/include/tkDNN/Yolo3Detection.h"

// Decomment next line to activate debugging prints
//#define DEBUG

using std::placeholders::_1;
using std::placeholders::_2;

class CameraDetection : public rclcpp::Node {
private:
    /* ***** PARAMETERS ***** */

    std::string location;
    bool output_video;

    /* ********************** */

    rclcpp::Time pointCloudTimestamp;
    std::vector<cv::Mat> batch_frame;
    std::vector<cv::Mat> batch_dnn_input;
    cv::Mat lastFrameLeft;
    cv::Mat lastFrameRight;
    mmr_base::msg::BoundingBoxes boundingBoxesLeft, boundingBoxesRight;
    tk::dnn::Yolo3Detection yolo;

    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>> image_sync;

    /* Bounding Boxes publishers */
    rclcpp::Publisher<mmr_base::msg::BoundingBoxes>::SharedPtr boundingBoxesPubLeft, boundingBoxesPubRight;

    /* Message filter subscribers */
    message_filters::Subscriber<sensor_msgs::msg::Image> img_left_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> img_right_sub;

    /* ***** PRIVATE METHODS ***** */

    void setNet();
    void loadParameters();
    static void showImage(const cv::Mat &frameLeft, const cv::Mat &frameRight);

    /* *************************** */
    
    /* Detection method */
    void* detection(rclcpp::Time timeStamp);

    /* Message Filter callback */
    void messageFilterCallback(const sensor_msgs::msg::Image::ConstSharedPtr& frameLeft, const sensor_msgs::msg::Image::ConstSharedPtr& frameRight);

public:
    CameraDetection();
    ~CameraDetection();

};

#endif //SRC_CAMERA_DETECTION_H

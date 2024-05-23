#include "tkdnn_ros2/camera_detection.hpp"
static const std::string OPENCV_WINDOW = "Image window";

//Constructor
CameraDetection::CameraDetection() : Node("tkdnn_ros2_node")
{

  RCLCPP_INFO(this->get_logger(), "LOADING PARAMS");

  this->loadParameters();

  RCLCPP_INFO(this->get_logger(), "CREATING PUBS");

  /* Bounding Boxes publishers */
  this->boundingBoxesPubLeft = this->create_publisher<mmr_base::msg::BoundingBoxes>("/camera/left_bb", 1);
  this->boundingBoxesPubRight = this->create_publisher<mmr_base::msg::BoundingBoxes>("/camera/right_bb", 1);

  RCLCPP_INFO(this->get_logger(), "CREATING SUBS");

  /* Message filter subscribers */
  this->img_left_sub.subscribe(this, "/camera/left_image", rmw_qos_profile_sensor_data);
  this->img_right_sub.subscribe(this, "/camera/right_image", rmw_qos_profile_sensor_data);
  
  batch_dnn_input.reserve(2);
  batch_frame.reserve(2);
  batch_frame.push_back(lastFrameLeft); 
  batch_frame.push_back(lastFrameRight);
  batch_dnn_input.push_back(lastFrameLeft);
  batch_dnn_input.push_back(lastFrameRight);

  //OpenCv frame just for display frames
  if (this->output_video) {
   	cv::namedWindow("camera_left", cv::WINDOW_AUTOSIZE);
	  cv::namedWindow("camera_right", cv::WINDOW_AUTOSIZE);
  }

  RCLCPP_INFO(this->get_logger(), "STARTING NETWORK");

  //TKDNN initialization...
  std::string network = "/home/orin/mmr-drive/src/1_perception/tkdnn_ros2/tkdnn_ros2_fp16.rt";
  network = network.replace(network.rfind("/"), network.length(), "/tkdnn_ros2_fp16.rt");
  std::ifstream f(network, std::ifstream::in);
  if (!f.is_open()){
#ifdef DEBUG
    ROS_INFO("set net");
#endif

    setNet();
  } else 
  {
    RCLCPP_INFO(this->get_logger(), "STARTING");
  }
  yolo.init(network, 4, 2);
  RCLCPP_INFO(this->get_logger(), "STARTED");

  /* Assign message filter callback to subscribers */
  this->image_sync = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(img_left_sub, img_right_sub, 10);
  this->image_sync->registerCallback(std::bind(&CameraDetection::messageFilterCallback, this, _1, _2));
}

inline void CameraDetection::loadParameters(){

	/* ***** DECLARING PARAMETERS ***** */

	declare_parameter("location", "/home/orin/mmr-drive/src/1_perception/tkdnn_ros2");
	declare_parameter("output_video", true);

	/* ******************************** */

	/* ***** READING PARAMETERS ***** */
	get_parameter("location", this->location);
	get_parameter("output_video", this->output_video);

	/* ****************************** */
  RCLCPP_INFO(this->get_logger(), "Location: %s", this->location.c_str());
  RCLCPP_INFO(this->get_logger(), "output video: %d", this->output_video);
}

//Network initialization
inline void CameraDetection::setNet(){
  RCLCPP_INFO(this->get_logger(), "SETTING NET");
  /**
     * modificare la directory di home
     */
  std::string bin_path = this->location;
  std::vector<std::string> input_bins = {
    bin_path + "/layers/input.bin"};
  std::vector<std::string> output_bins = {
    bin_path + "/debug/layer139_out.bin",
    bin_path + "/debug/layer150_out.bin",
    bin_path + "/debug/layer161_out.bin"};

  std::string wgs_path = bin_path + "/layers";
  std::string cfg_path = bin_path + "/cfg/yolov4-mmr.cfg";
  std::string name_path = bin_path + "/names/cones.names";

  // parse darknet network
  tk::dnn::Network *net = tk::dnn::darknetParser(cfg_path, wgs_path, name_path);
  net->print();

  //convert network to tensorRT
  tk::dnn::NetworkRT *netRT = new tk::dnn::NetworkRT(net, net->getNetworkRTName(bin_path.c_str()));

  net->releaseLayers();
  delete net;
  delete netRT;
}

//Destructor
CameraDetection::~CameraDetection(){
  #ifdef DEBUG
    cv::destroyWindow(OPENCV_WINDOW);
  #endif
}

void CameraDetection::messageFilterCallback(const sensor_msgs::msg::Image::ConstSharedPtr& frameLeft, const sensor_msgs::msg::Image::ConstSharedPtr& frameRight)
{
  try{
      this->lastFrameLeft = cv_bridge::toCvCopy(frameLeft, sensor_msgs::image_encodings::BGR8)->image;
      this->lastFrameRight = cv_bridge::toCvCopy(frameRight, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch (cv_bridge::Exception &e){
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
  this->detection(frameLeft->header.stamp);
}

void *CameraDetection::detection(rclcpp::Time timeStamp)
{
  rclcpp::Time start, end;
  batch_dnn_input.at(0) = this->lastFrameLeft;
  batch_dnn_input.at(1) = this->lastFrameRight;
  if (this->output_video) {
    batch_frame.at(0) = this->lastFrameLeft;
    batch_frame.at(1) = this->lastFrameRight;
  }
  start = this->now();
  yolo.update(batch_dnn_input, 2);

  end = this->now();
  rclcpp::Duration exe_time = end - start;
  // RCLCPP_INFO(this->get_logger(), "Time (ms): %lf", ((float)exe_time.nanoseconds() * 1e-6));

  if (this->output_video) yolo.draw(batch_frame);
  bool atLeastOneDetectionLeft = false;
  bool atLeastOneDetectionRight = false;
  boundingBoxesLeft.bounding_boxes.clear();
  boundingBoxesRight.bounding_boxes.clear();
  int i = 0;
  
  for (auto bb_vec : yolo.batchDetected) {
    for (auto b : bb_vec){
      if (i == 0) {
        atLeastOneDetectionLeft = true;
      }
      else {
        atLeastOneDetectionRight = true;
      }
      mmr_base::msg::BoundingBox boundingBox;
      boundingBox.top_left_x = static_cast<unsigned int>(round(b.x));
      boundingBox.top_left_y = static_cast<unsigned int>(round(b.y));
      boundingBox.width = static_cast<unsigned int>(round(b.w));
      boundingBox.height = static_cast<unsigned int>(round(b.h));
      boundingBox.probability = b.prob;
      boundingBox.id = static_cast<uint8_t>(b.cl);
      if (i == 0)
        boundingBoxesLeft.bounding_boxes.push_back(boundingBox);
      else
        boundingBoxesRight.bounding_boxes.push_back(boundingBox);
    }
    i++;
  }

  //Draw BB on images
  if (this->output_video) {
    showImage(batch_frame[0], batch_frame[1]);
  }

  // adding timestamp to boundingBoxes
  boundingBoxesLeft.header.stamp = timeStamp;
  boundingBoxesRight.header.stamp = timeStamp;

  sensor_msgs::msg::Image::SharedPtr leftFrame, rightFrame;

  boundingBoxesPubLeft->publish(boundingBoxesLeft);
  boundingBoxesPubRight->publish(boundingBoxesRight);
  
  return nullptr;
}

inline void CameraDetection::showImage(const cv::Mat &frameLeft, const cv::Mat &frameRight){
    cv::imshow("camera_left", frameLeft);
    cv::imshow("camera_right", frameRight);
    cv::waitKey(3);
}

#include "tkdnn_ros2/camera_detection.hpp"
#include <unistd.h>

void handleSignal(int signal) {
    if (signal == SIGINT) {
        std::cout << "Received SIGINT. Killing tkdnn_ros2 process.\n";
        rclcpp::shutdown();
    }
}

int main(int argc, char * argv[])
{
  signal(SIGINT, handleSignal);
  /* node initialization */
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CameraDetection>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
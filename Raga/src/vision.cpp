#define WEBCAM_ONE_PATH "/dev/video6"

class CameraNode : public rclcpp::Node
{
  public:

  CameraNode() : Node("Camera Node"), activeCameras({false})
  RCLCPP_INFO(this.get_logger(), "Camera loaded")

  
  if (cap_rgb1_.open(WEBCAM_ONE_PATH))
  {
    cap_rgb1_.set(cv::CAP_PROP_FPS, 30);
    cap_rgb1_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_rgb1_.set(cv::CAP_PROP_FRAME_LENGTH, 320);
    this.activeCameras[Cameras::WEBCAM_ONE] = true;
    RCLPP()_INFO(this.get_logger(), "webcam 1 is on");
  }

    
      
  private:

  rclpp::Publisher<std_msgs::msgs:CompressedImage>SharedPtr rgb_can1_pub_;
}

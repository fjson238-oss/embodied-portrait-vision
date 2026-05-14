#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

namespace
{
  int fourcc_from_string(const char *value)
  {
    return cv::VideoWriter::fourcc(value[0], value[1], value[2], value[3]);
  }
}

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : Node("camera_publisher")
    {
      //1.声明与读取参数
      this->declare_parameter<int>("device_id", 0);
      this->declare_parameter<std::string>("window_name", "camera");
      this->declare_parameter<int>("image_width", 1280);
      this->declare_parameter<int>("image_height", 720);
      this->declare_parameter<int>("fps", 30);
      this->declare_parameter<std::string>("frame_id", "camera_frame");
      this->declare_parameter<bool>("show_window", false);
      device_id_ = this->get_parameter("device_id").as_int();
      window_name_ = this->get_parameter("window_name").as_string();
      image_width_ = this->get_parameter("image_width").as_int();
      image_height_ = this->get_parameter("image_height").as_int();
      fps_ = this->get_parameter("fps").as_int();
      frame_id_ = this->get_parameter("frame_id").as_string();
      show_window_ = this->get_parameter("show_window").as_bool();

      // 2 创建发布者
      image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
      // 3.打开摄像头
      cap_.open(device_id_, cv::CAP_V4L2);
      if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera device_id=%d", device_id_);
        throw std::runtime_error("Failed to open camera");
      }

      //4.设置相机属性
      cap_.set(cv::CAP_PROP_FOURCC, static_cast<double>(fourcc_from_string("MJPG")));
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(image_width_));
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(image_height_));

      if (show_window_) {
        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
      }

      // 5. 预热几帧
      for (int i = 0; i < 5; ++i) {
        cap_.grab();
        cap_.retrieve(frame_);
        std::this_thread::sleep_for(10ms);
      }


      // 6.创建定时器，每 1 秒调用一次 timer_callback
      const auto period = std::chrono::milliseconds(1000 / fps_);
      timer_ = this->create_wall_timer(
          period,
          std::bind(&CameraPublisher::timer_callback, this));

      RCLCPP_INFO(this->get_logger(), "CameraPublisher started. device_id=%d, size=%dx%d, fps=%d",
      device_id_, image_width_, image_height_, fps_);
    }

    ~CameraPublisher() override
    {
      if (cap_.isOpened()) {
        cap_.release();
      }
      if (show_window_) {
        cv::destroyAllWindows();
      }
    }

private:
    void timer_callback()
    {
      if (!cap_.grab() || !cap_.retrieve(frame_) || frame_.empty()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Empty frame received");
        return;
      }
      if (show_window_) {
        cv::imshow(window_name_, frame_);
        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') {
          RCLCPP_INFO(this->get_logger(), "Exit requested by keyboard");
          rclcpp::shutdown();
          return;
        }
      }

      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = frame_id_;
      //带上时间戳、帧号？转换格式并发出
      auto msg = cv_bridge::CvImage(header, "bgr8", frame_).toImageMsg();
      image_pub_->publish(*msg);
    }
private:
    // ROS2
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 参数
    int device_id_{0};
    int image_width_{1280};
    int image_height_{720};
    int fps_{30};
    bool show_window_{true};
    std::string window_name_{"camera"};
    std::string frame_id_{"camera_frame"};
    int count_;

    // OpenCV
    cv::VideoCapture cap_;
    cv::Mat frame_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber")
  {
    // 1. 声明参数
    this->declare_parameter<std::string>("topic_name", "/camera/image_raw");
    this->declare_parameter<std::string>("window_name", "image_subscriber");
    this->declare_parameter<bool>("show_info", true);

    // 2. 读取参数
    topic_name_ = this->get_parameter("topic_name").as_string();
    window_name_ = this->get_parameter("window_name").as_string();
    show_info_ = this->get_parameter("show_info").as_bool();

    // 3. 创建窗口
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);

    // 4. 创建订阅者
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name_,
      10,
      std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ImageSubscriber started.");
    RCLCPP_INFO(this->get_logger(), "Subscribed topic: %s", topic_name_.c_str());
  }

  ~ImageSubscriber() override
  {
    cv::destroyAllWindows();
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // 1. ROS2 Image -> cv::Mat
      cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

      if (frame.empty()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "Received empty image frame");
        return;
      }

      // 2. 显示图像
      cv::imshow(window_name_, frame);

      // 3. 键盘退出
      const int key = cv::waitKey(1);
      if (key == 27 || key == 'q' || key == 'Q') {
        RCLCPP_INFO(this->get_logger(), "Exit requested by keyboard");
        rclcpp::shutdown();
        return;
      }

      // 4. 可选打印部分信息
      if (show_info_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          3000,
          "Received image: %u x %u, encoding=%s, frame_id=%s",
          msg->width,
          msg->height,
          msg->encoding.c_str(),
          msg->header.frame_id.c_str());
      }

    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "cv_bridge exception: %s",
        e.what());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Exception in image_callback: %s",
        e.what());
    }
  }

private:
  std::string topic_name_;
  std::string window_name_;
  bool show_info_{true};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <stdexcept>

class DetectorNode : public rclcpp::Node
{
public:
    DetectorNode() : Node("detector_node")
    {
        this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("model_path", "");
        this->declare_parameter<std::string>("model_name", "yolov8n_slim.onnx");
        std::string image_topic = this->get_parameter("image_topic").as_string();

        std::string model_path = this->get_parameter("model_path").as_string();
        if (model_path.empty())
        {
            const std::string model_name = this->get_parameter("model_name").as_string();
            model_path =
                ament_index_cpp::get_package_share_directory("vision_detector") +
                "/models/" + model_name;
        }

        RCLCPP_INFO(this->get_logger(), "Loading YOLO model: %s", model_path.c_str());
        if (!std::ifstream(model_path).good())
        {
            throw std::runtime_error("YOLO model file does not exist or is not readable: " + model_path);
        }

        try
        {
            net_ = cv::dnn::readNetFromONNX(model_path);
        }
        catch (const cv::Exception& e)
        {
            RCLCPP_FATAL(
                this->get_logger(),
                "Failed to load YOLO ONNX model. This usually means the model is not compatible "
                "with the OpenCV DNN version linked by this ROS2 node. Error: %s",
                e.what()
            );
            throw;
        }

        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        RCLCPP_INFO(this->get_logger(), "YOLO model loaded successfully.");

        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Detector node started.");
        RCLCPP_INFO(this->get_logger(), "Subscribing image topic: %s", image_topic.c_str());
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received image: %d x %d",
            msg->width, msg->height);
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;
            cv::Mat blob;

            cv::dnn::blobFromImage(
                frame,
                blob,
                1.0 / 255.0,
                cv::Size(640, 640),
                cv::Scalar(),
                true,
                false
            );

            net_.setInput(blob);

            std::vector<cv::Mat> outputs;
            net_.forward(outputs, net_.getUnconnectedOutLayersNames());

            if (frame_count_ % 30 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "YOLO output num: %zu", outputs.size());

                for (size_t i = 0; i < outputs.size(); ++i)
                {
                    std::string shape_str;

                    for (int d = 0; d < outputs[i].dims; ++d)
                    {
                        shape_str += std::to_string(outputs[i].size[d]);

                        if (d != outputs[i].dims - 1)
                        {
                            shape_str += " x ";
                        }
                    }

                    RCLCPP_INFO(
                        this->get_logger(),
                        "Output[%zu] dims=%d shape=%s",
                        i,
                        outputs[i].dims,
                        shape_str.c_str()
                    );
                }
            }
            frame_count_++;
            if (frame_count_ % 30 == 0)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Received image: width=%d, height=%d, channels=%d",
                    frame.cols,
                    frame.rows,
                    frame.channels()
                );
            }
            cv::Mat result = processFrame(frame);
            cv::imshow("detector_result", result);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

private:
    cv::Mat processFrame(const cv::Mat& frame)
    {
        cv::Mat result = frame.clone();
        int center_x = result.cols / 2;
        int center_y = result.rows / 2;
        cv::circle(
            result,
            cv::Point(center_x, center_y),
            6,
            cv::Scalar(0, 0, 255),
            -1
        );
        cv::line(
            result,
            cv::Point(center_x - 30, center_y),
            cv::Point(center_x + 30, center_y),
            cv::Scalar(0, 255, 0),
            2
        );
        cv::line(
            result,
            cv::Point(center_x, center_y - 30),
            cv::Point(center_x, center_y + 30),
            cv::Scalar(0, 255, 0),
            2
        );
        return result;
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    int frame_count_ = 0;
    cv::dnn::Net net_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

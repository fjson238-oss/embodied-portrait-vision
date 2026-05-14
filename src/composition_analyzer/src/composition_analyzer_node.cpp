#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <portrait_interfaces/msg/composition_score.hpp>
#include <portrait_interfaces/msg/detection_array.hpp>
#include <portrait_interfaces/msg/skeleton_array.hpp>

#include "composition_analyzer/composition_rules.hpp"

#include <string>
#include <vector>

using composition_analyzer::CompositionRules;
using composition_analyzer::FrameInfo;
using composition_analyzer::Keypoint2D;
using composition_analyzer::PersonBox;
using composition_analyzer::SkeletonInfo;
using portrait_interfaces::msg::CompositionScore;
using portrait_interfaces::msg::DetectionArray;
using portrait_interfaces::msg::SkeletonArray;

class CompositionAnalyzerNode : public rclcpp::Node
{
public:
    CompositionAnalyzerNode()
    : Node("composition_analyzer_node")
    {
        this->declare_parameter<std::string>("detection_topic", "/person/detections");
        this->declare_parameter<std::string>("skeleton_topic", "/portrait/skeletons");
        this->declare_parameter<std::string>("score_topic", "/composition/score");
        const std::string detection_topic = this->get_parameter("detection_topic").as_string();
        const std::string skeleton_topic = this->get_parameter("skeleton_topic").as_string();
        const std::string score_topic = this->get_parameter("score_topic").as_string();

        detection_subscriber_ = this->create_subscription<DetectionArray>(
            detection_topic,
            10,
            std::bind(
                &CompositionAnalyzerNode::detection_callback,
                this,
                std::placeholders::_1
            )
        );

        skeleton_subscriber_ = this->create_subscription<SkeletonArray>(
            skeleton_topic,
            10,
            std::bind(
                &CompositionAnalyzerNode::skeleton_callback,
                this,
                std::placeholders::_1
            )
        );

        suggestion_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/composition/suggestion",
            10
        );
        score_publisher_ = this->create_publisher<CompositionScore>(
            score_topic,
            10
        );

        RCLCPP_INFO(this->get_logger(), "Composition analyzer node started.");
        RCLCPP_INFO(this->get_logger(), "Subscribing topic: %s", detection_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing topic: %s", skeleton_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing topic: /composition/suggestion");
        RCLCPP_INFO(this->get_logger(), "Publishing topic: %s", score_topic.c_str());
    }

private:
    void skeleton_callback(const SkeletonArray::SharedPtr msg)
    {
        latest_skeletons_.clear();
        latest_skeletons_.reserve(msg->skeletons.size());

        for (const auto& skeleton_msg : msg->skeletons) {
            SkeletonInfo skeleton;
            skeleton.person_id = skeleton_msg.person_id;
            skeleton.has_pose = skeleton_msg.has_pose;
            skeleton.score = skeleton_msg.score;
            skeleton.keypoints.reserve(skeleton_msg.keypoints.size());

            for (const auto& keypoint_msg : skeleton_msg.keypoints) {
                skeleton.keypoints.push_back(Keypoint2D{
                    keypoint_msg.name,
                    keypoint_msg.x,
                    keypoint_msg.y,
                    keypoint_msg.score,
                    keypoint_msg.valid
                });
            }

            latest_skeletons_.push_back(skeleton);
        }

        RCLCPP_DEBUG(
            this->get_logger(),
            "cached skeletons: %zu",
            latest_skeletons_.size()
        );
    }

    void detection_callback(const DetectionArray::SharedPtr msg)
    {
        const FrameInfo frame{
            msg->frame_width,
            msg->frame_height
        };

        std::vector<PersonBox> person_boxes;
        person_boxes.reserve(msg->detections.size());

        for (const auto& detection : msg->detections) {
            const auto& box = detection.bbox;
            person_boxes.push_back(PersonBox{
                box.x1,
                box.y1,
                box.x2,
                box.y2,
                detection.score
            });
        }

        const auto result = rules_.analyze_id_photo(frame, person_boxes, latest_skeletons_);
        const std::string suggestion = format_suggestion(result);

        RCLCPP_INFO(
            this->get_logger(),
                "frame: %d x %d, detections: %zu, skeletons: %zu, composition score: %d",
                frame.width,
                frame.height,
                person_boxes.size(),
                latest_skeletons_.size(),
                result.score
            );

        publish_suggestion(suggestion);
        publish_score(msg->header, frame, result, person_boxes.size(), latest_skeletons_.size());
    }

    std::string format_suggestion(const composition_analyzer::RuleResult& result) const
    {
        std::string text = "证件照构图评分：" + std::to_string(result.score) + "/100。";
        text += result.summary;

        for (const auto& suggestion : result.suggestions) {
            text += "\n- " + suggestion;
        }

        return text;
    }

    void publish_suggestion(const std::string& text)
    {
        std_msgs::msg::String suggestion_msg;
        suggestion_msg.data = text;
        suggestion_publisher_->publish(suggestion_msg);

        RCLCPP_INFO(this->get_logger(), "Composition suggestion: %s", text.c_str());
    }

    void publish_score(
        const std_msgs::msg::Header& header,
        const FrameInfo& frame,
        const composition_analyzer::RuleResult& result,
        const size_t detection_count,
        const size_t skeleton_count)
    {
        CompositionScore score_msg;
        score_msg.header = header;
        score_msg.score = result.score;
        score_msg.level = result.level;
        score_msg.profile = "id_photo";
        score_msg.summary = result.summary;
        score_msg.suggestions = result.suggestions;
        score_msg.frame_width = frame.width;
        score_msg.frame_height = frame.height;
        score_msg.detection_count = static_cast<uint32_t>(detection_count);
        score_msg.skeleton_count = static_cast<uint32_t>(skeleton_count);
        score_publisher_->publish(score_msg);
    }

    rclcpp::Subscription<DetectionArray>::SharedPtr detection_subscriber_;
    rclcpp::Subscription<SkeletonArray>::SharedPtr skeleton_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr suggestion_publisher_;
    rclcpp::Publisher<CompositionScore>::SharedPtr score_publisher_;
    CompositionRules rules_;
    std::vector<SkeletonInfo> latest_skeletons_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CompositionAnalyzerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

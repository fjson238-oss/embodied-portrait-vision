#include "composition_analyzer/composition_rules.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace composition_analyzer
{

namespace
{
double clamp_ratio(double value)
{
    return std::clamp(value, 0.0, 1.0);
}

int clamp_score(int score)
{
    return std::clamp(score, 0, 100);
}

std::string percent_text(double ratio)
{
    std::ostringstream stream;
    stream.precision(1);
    stream << std::fixed << ratio * 100.0 << "%";
    return stream.str();
}
}  // namespace

RuleResult CompositionRules::analyze_id_photo(
    const FrameInfo& frame,
    const std::vector<PersonBox>& boxes,
    const std::vector<SkeletonInfo>& skeletons) const
{
    RuleResult result;
    result.score = 100;

    if (frame.width <= 0 || frame.height <= 0) {
        result.score = 0;
        result.summary = "画面尺寸无效，无法进行构图分析。";
        result.suggestions.push_back("请检查检测节点发布的 image_width / image_height 字段。");
        return result;
    }

    if (boxes.empty()) {
        result.score = 0;
        result.summary = "当前画面中未检测到人物。";
        result.suggestions.push_back("请确认人物在画面内，并检查 YOLO 检测节点是否正常输出人体框。");
        return result;
    }

    const PersonBox person = select_best_person(boxes);
    analyze_horizontal_position(frame, person, result);
    analyze_person_scale(frame, person, result);
    const SkeletonInfo* skeleton = select_best_skeleton(skeletons);
    analyze_skeleton_posture(skeleton, result);
    analyze_headroom_hint(skeleton, result);

    result.score = clamp_score(result.score);
    if (result.score >= 85) {
        result.level = "good";
        result.summary = "证件照构图整体较好。";
    } else if (result.score >= 65) {
        result.level = "ok";
        result.summary = "证件照构图基本可用，但仍有调整空间。";
    } else {
        result.level = "poor";
        result.summary = "证件照构图偏差较明显，建议重新调整人物位置和距离。";
    }

    return result;
}

PersonBox CompositionRules::select_best_person(const std::vector<PersonBox>& boxes) const
{
    return *std::max_element(
        boxes.begin(),
        boxes.end(),
        [](const PersonBox& lhs, const PersonBox& rhs) {
            return lhs.score < rhs.score;
        }
    );
}

const SkeletonInfo* CompositionRules::select_best_skeleton(const std::vector<SkeletonInfo>& skeletons) const
{
    auto best = std::max_element(
        skeletons.begin(),
        skeletons.end(),
        [](const SkeletonInfo& lhs, const SkeletonInfo& rhs) {
            return lhs.score < rhs.score;
        }
    );

    if (best == skeletons.end() || !best->has_pose) {
        return nullptr;
    }
    return &(*best);
}

const Keypoint2D* CompositionRules::find_keypoint(const SkeletonInfo& skeleton, const std::string& name) const
{
    auto match = std::find_if(
        skeleton.keypoints.begin(),
        skeleton.keypoints.end(),
        [&name](const Keypoint2D& keypoint) {
            return keypoint.valid && keypoint.name == name;
        }
    );

    if (match == skeleton.keypoints.end()) {
        return nullptr;
    }
    return &(*match);
}

void CompositionRules::analyze_horizontal_position(
    const FrameInfo& frame,
    const PersonBox& person,
    RuleResult& result) const
{
    const double center_x = (person.x1 + person.x2) / 2.0;
    const double normalized_offset = (center_x - frame.width / 2.0) / frame.width;
    const double abs_offset = std::abs(normalized_offset);

    if (abs_offset <= 0.05) {
        result.suggestions.push_back("人物水平居中较好，符合证件照主体居中的基本要求。");
        return;
    }

    if (abs_offset <= 0.10) {
        result.score -= 8;
        result.suggestions.push_back(
            normalized_offset < 0.0
                ? "人物略微偏左，建议向画面中心微调。"
                : "人物略微偏右，建议向画面中心微调。"
        );
        return;
    }

    result.score -= 18;
    result.suggestions.push_back(
        normalized_offset < 0.0
            ? "人物明显偏左，证件照建议让人物面部和躯干回到画面中轴线附近。"
            : "人物明显偏右，证件照建议让人物面部和躯干回到画面中轴线附近。"
    );
}

void CompositionRules::analyze_person_scale(
    const FrameInfo& frame,
    const PersonBox& person,
    RuleResult& result) const
{
    const int box_width = std::max(0, person.x2 - person.x1);
    const int box_height = std::max(0, person.y2 - person.y1);
    const double height_ratio = clamp_ratio(static_cast<double>(box_height) / frame.height);
    const double area_ratio = clamp_ratio(
        static_cast<double>(box_width) * box_height /
        static_cast<double>(frame.width * frame.height)
    );

    // 证件照风格：人物应占画面主体，但不要大到贴边。这里用人体检测框做近似规则。
    constexpr double min_height_ratio = 0.62;
    constexpr double max_height_ratio = 0.88;
    constexpr double min_area_ratio = 0.22;
    constexpr double max_area_ratio = 0.55;

    if (height_ratio < min_height_ratio || area_ratio < min_area_ratio) {
        result.score -= 18;
        result.suggestions.push_back(
            "人物画面占比偏小，当前人体框高度约为画面高度的 " +
            percent_text(height_ratio) +
            "，建议人物靠近镜头或适当放大画面。"
        );
        return;
    }

    if (height_ratio > max_height_ratio || area_ratio > max_area_ratio) {
        result.score -= 18;
        result.suggestions.push_back(
            "人物画面占比偏大，当前人体框高度约为画面高度的 " +
            percent_text(height_ratio) +
            "，证件照建议略微拉远，避免头部或肩部过于贴边。"
        );
        return;
    }

    result.suggestions.push_back(
        "人物画面占比较合适，人体框高度约为画面高度的 " +
        percent_text(height_ratio) +
        "。"
    );
}

void CompositionRules::analyze_skeleton_posture(const SkeletonInfo* skeleton, RuleResult& result) const
{
    if (skeleton == nullptr) {
        result.score -= 8;
        result.suggestions.push_back("暂未获得有效骨架信息，无法判断肩线、身体中轴和头肩姿态。");
        return;
    }

    analyze_shoulder_line(*skeleton, result);
    analyze_body_axis(*skeleton, result);
    analyze_head_shoulder_pose(*skeleton, result);
}

void CompositionRules::analyze_shoulder_line(const SkeletonInfo& skeleton, RuleResult& result) const
{
    const Keypoint2D* left_shoulder = find_keypoint(skeleton, "left_shoulder");
    const Keypoint2D* right_shoulder = find_keypoint(skeleton, "right_shoulder");

    if (left_shoulder == nullptr || right_shoulder == nullptr) {
        result.score -= 5;
        result.suggestions.push_back("肩部关键点不完整，暂时无法准确判断肩线是否水平。");
        return;
    }

    const double shoulder_width = std::abs(left_shoulder->x - right_shoulder->x);
    if (shoulder_width < 1.0) {
        result.score -= 5;
        result.suggestions.push_back("肩部关键点距离过小，肩线判断不稳定。");
        return;
    }

    const double shoulder_slope = std::abs(left_shoulder->y - right_shoulder->y) / shoulder_width;
    if (shoulder_slope <= 0.08) {
        result.suggestions.push_back("肩线基本水平，证件照姿态较稳定。");
    } else if (shoulder_slope <= 0.15) {
        result.score -= 8;
        result.suggestions.push_back("肩线略有倾斜，建议放松双肩并保持身体端正。");
    } else {
        result.score -= 16;
        result.suggestions.push_back("肩线倾斜较明显，证件照建议调整站姿或坐姿，让双肩尽量保持水平。");
    }
}

void CompositionRules::analyze_body_axis(const SkeletonInfo& skeleton, RuleResult& result) const
{
    const Keypoint2D* left_shoulder = find_keypoint(skeleton, "left_shoulder");
    const Keypoint2D* right_shoulder = find_keypoint(skeleton, "right_shoulder");
    const Keypoint2D* left_hip = find_keypoint(skeleton, "left_hip");
    const Keypoint2D* right_hip = find_keypoint(skeleton, "right_hip");

    if (left_shoulder == nullptr || right_shoulder == nullptr || left_hip == nullptr || right_hip == nullptr) {
        result.score -= 5;
        result.suggestions.push_back("肩部或髋部关键点不完整，身体中轴线判断不稳定。");
        return;
    }

    const double shoulder_center_x = (left_shoulder->x + right_shoulder->x) / 2.0;
    const double shoulder_center_y = (left_shoulder->y + right_shoulder->y) / 2.0;
    const double hip_center_x = (left_hip->x + right_hip->x) / 2.0;
    const double hip_center_y = (left_hip->y + right_hip->y) / 2.0;
    const double body_height = std::abs(hip_center_y - shoulder_center_y);

    if (body_height < 1.0) {
        result.score -= 5;
        result.suggestions.push_back("肩部和髋部关键点距离过小，身体中轴判断不稳定。");
        return;
    }

    const double axis_offset = std::abs(shoulder_center_x - hip_center_x) / body_height;
    if (axis_offset <= 0.10) {
        result.suggestions.push_back("身体中轴基本端正，适合证件照构图。");
    } else if (axis_offset <= 0.20) {
        result.score -= 8;
        result.suggestions.push_back("身体略有侧倾，建议让上半身回到更垂直的姿态。");
    } else {
        result.score -= 16;
        result.suggestions.push_back("身体侧倾较明显，证件照建议调整坐姿或站姿，保持上半身正直。");
    }
}

void CompositionRules::analyze_head_shoulder_pose(const SkeletonInfo& skeleton, RuleResult& result) const
{
    const Keypoint2D* nose = find_keypoint(skeleton, "nose");
    const Keypoint2D* left_shoulder = find_keypoint(skeleton, "left_shoulder");
    const Keypoint2D* right_shoulder = find_keypoint(skeleton, "right_shoulder");

    if (nose == nullptr || left_shoulder == nullptr || right_shoulder == nullptr) {
        result.score -= 5;
        result.suggestions.push_back("头部或肩部关键点不完整，头肩姿态判断不稳定。");
        return;
    }

    const double shoulder_center_x = (left_shoulder->x + right_shoulder->x) / 2.0;
    const double shoulder_width = std::abs(left_shoulder->x - right_shoulder->x);
    if (shoulder_width < 1.0) {
        result.score -= 5;
        result.suggestions.push_back("肩宽关键点过小，头肩居中判断不稳定。");
        return;
    }

    const double head_offset = std::abs(nose->x - shoulder_center_x) / shoulder_width;
    if (head_offset <= 0.12) {
        result.suggestions.push_back("头部相对肩部居中较好，符合证件照正面姿态。");
    } else if (head_offset <= 0.22) {
        result.score -= 8;
        result.suggestions.push_back("头部相对肩部略偏，建议面部和肩部尽量朝向镜头正中。");
    } else {
        result.score -= 16;
        result.suggestions.push_back("头部相对肩部偏移明显，证件照建议摆正头部，避免歪头或侧身。");
    }
}

void CompositionRules::analyze_headroom_hint(const SkeletonInfo* skeleton, RuleResult& result) const
{
    if (skeleton == nullptr) {
        result.suggestions.push_back("头顶留白规则已预留：需要有效骨架关键点后才能进一步判断。");
        return;
    }

    const Keypoint2D* nose = find_keypoint(*skeleton, "nose");
    const Keypoint2D* left_eye = find_keypoint(*skeleton, "left_eye");
    const Keypoint2D* right_eye = find_keypoint(*skeleton, "right_eye");
    if (nose == nullptr || left_eye == nullptr || right_eye == nullptr) {
        result.suggestions.push_back("头顶留白规则已预留：当前头部关键点不完整，后续可结合人脸框进一步优化。");
        return;
    }

    result.suggestions.push_back("头顶留白规则已接入头部关键点基础信息，后续可结合人脸框或头顶估计继续细化。");
}

}  // namespace composition_analyzer

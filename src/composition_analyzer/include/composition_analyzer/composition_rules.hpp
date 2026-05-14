#pragma once

#include <string>
#include <vector>

namespace composition_analyzer
{

struct FrameInfo
{
    int width{0};
    int height{0};
};

struct PersonBox
{
    int x1{0};
    int y1{0};
    int x2{0};
    int y2{0};
    double score{0.0};
};

struct Keypoint2D
{
    std::string name;
    double x{0.0};
    double y{0.0};
    double score{0.0};
    bool valid{false};
};

struct SkeletonInfo
{
    unsigned int person_id{0};
    bool has_pose{false};
    double score{0.0};
    std::vector<Keypoint2D> keypoints;
};

struct RuleResult
{
    int score{0};
    std::string level;
    std::string summary;
    std::vector<std::string> suggestions;
};

class CompositionRules
{
public:
    RuleResult analyze_id_photo(
        const FrameInfo& frame,
        const std::vector<PersonBox>& boxes,
        const std::vector<SkeletonInfo>& skeletons = {}) const;

private:
    PersonBox select_best_person(const std::vector<PersonBox>& boxes) const;
    const SkeletonInfo* select_best_skeleton(const std::vector<SkeletonInfo>& skeletons) const;
    const Keypoint2D* find_keypoint(const SkeletonInfo& skeleton, const std::string& name) const;
    void analyze_horizontal_position(const FrameInfo& frame, const PersonBox& person, RuleResult& result) const;
    void analyze_person_scale(const FrameInfo& frame, const PersonBox& person, RuleResult& result) const;
    void analyze_skeleton_posture(const SkeletonInfo* skeleton, RuleResult& result) const;
    void analyze_shoulder_line(const SkeletonInfo& skeleton, RuleResult& result) const;
    void analyze_body_axis(const SkeletonInfo& skeleton, RuleResult& result) const;
    void analyze_head_shoulder_pose(const SkeletonInfo& skeleton, RuleResult& result) const;
    void analyze_headroom_hint(const SkeletonInfo* skeleton, RuleResult& result) const;
};

}  // namespace composition_analyzer

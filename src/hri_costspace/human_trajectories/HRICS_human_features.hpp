#ifndef HRICS_HUMANFEATURES_HPP
#define HRICS_HUMANFEATURES_HPP

#include "feature_space/features.hpp"

#include "hri_costspace/HRICS_distance.hpp"
#include "hri_costspace/HRICS_visibility.hpp"
#include "hri_costspace/HRICS_natural.hpp"

namespace HRICS
{

class DistanceFeature : public Move3D::Feature
{
public:
    DistanceFeature( Move3D::Robot* active, Move3D::Robot* passive );
    Move3D::FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
    Move3D::FeatureVect computeDistances() const;

    void draw();

private:
    Move3D::Robot* human_active_;
    Move3D::Robot* human_passive_;

    Distance* dist_cost_;

    std::vector<int> distance_joint_ids_;

    std::vector<std::string> distance_names_;

    std::vector<Move3D::Joint*> human_active_joints_;
    std::vector<Move3D::Joint*> human_passive_joints_;
};

class VisibilityFeature : public Move3D::Feature
{
public:
    VisibilityFeature() : Feature() {}
    Move3D::FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
};

class MuskuloskeletalFeature : public Move3D::Feature
{
public:
    MuskuloskeletalFeature() : Feature() {}
    Move3D::FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
};

class ReachabilityFeature : public Move3D::Feature
{
public:
    ReachabilityFeature() : Feature() {}
    Move3D::FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
};

class LegibilityFeature : public Move3D::Feature
{
public:
    LegibilityFeature() : Feature() {}
    Move3D::FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
};

}

#endif // HRICS_HUMANFEATURES_HPP

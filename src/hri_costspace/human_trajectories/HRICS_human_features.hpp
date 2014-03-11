#ifndef HRICS_HUMANFEATURES_HPP
#define HRICS_HUMANFEATURES_HPP

#include "HRICS_features.hpp"

#include "hri_costspace/HRICS_Distance.hpp"
#include "hri_costspace/HRICS_Visibility.hpp"
#include "hri_costspace/HRICS_Natural.hpp"

namespace HRICS
{

class DistanceFeature : public Feature
{
public:
    DistanceFeature( Move3D::Robot* active, Move3D::Robot* passive );
    FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
    FeatureVect computeDistances() const;

private:
    Move3D::Robot* human_active_;
    Move3D::Robot* human_passive_;

    Distance* dist_cost_;

    std::vector<int> active_joints_;
};

class VisibilityFeature : public Feature
{
public:
    VisibilityFeature() : Feature() {}
    FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
};

class MuskuloskeletalFeature : public Feature
{
public:
    MuskuloskeletalFeature() : Feature() {}
    FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
};

class ReachabilityFeature : public Feature
{
public:
    ReachabilityFeature() : Feature() {}
    FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
};

class LegibilityFeature : public Feature
{
public:
    LegibilityFeature() : Feature() {}
    FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
};

}

#endif // HRICS_HUMANFEATURES_HPP

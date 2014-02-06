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
    DistanceFeature( Robot* active, Robot* passive );
    FeatureVect getFeatureCount(const API::Trajectory& t);
    FeatureVect getFeatures(const Configuration& q);
    FeatureVect computeDistances() const;

private:
    Robot* human_active_;
    Robot* human_passive_;

    Distance* dist_cost_;

    std::vector<int> active_joints_;
};

class VisibilityFeature : public Feature
{
public:
    VisibilityFeature() : Feature() {}
    FeatureVect getFeatureCount(const API::Trajectory& t);
    FeatureVect getFeatures(const Configuration& q);
};

class MuskuloskeletalFeature : public Feature
{
public:
    MuskuloskeletalFeature() : Feature() {}
    FeatureVect getFeatureCount(const API::Trajectory& t);
    FeatureVect getFeatures(const Configuration& q);
};

class ReachabilityFeature : public Feature
{
public:
    ReachabilityFeature() : Feature() {}
    FeatureVect getFeatureCount(const API::Trajectory& t);
    FeatureVect getFeatures(const Configuration& q);
};

class LegibilityFeature : public Feature
{
public:
    LegibilityFeature() : Feature() {}
    FeatureVect getFeatureCount(const API::Trajectory& t);
    FeatureVect getFeatures(const Configuration& q);
};

}

#endif // HRICS_HUMANFEATURES_HPP

#ifndef HRICS_FEATURES_HPP
#define HRICS_FEATURES_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>

#include "API/Trajectory/trajectory.hpp"

namespace HRICS
{
class Distance;
class Visibility;
class Natural;

typedef Eigen::VectorXd FeatureVect;
typedef Eigen::VectorXd WeightVect;

class Feature
{
public:
    Feature() {}
    virtual FeatureVect getFeatureCount(const API::Trajectory& t) = 0;
};

class DistanceFeature : public Feature
{
public:
    DistanceFeature() : Feature() {}
    FeatureVect getFeatureCount(const API::Trajectory& t);
};

class VisibilityFeature : public Feature
{
public:
    VisibilityFeature() : Feature() {}
    FeatureVect getFeatureCount(const API::Trajectory& t);
};

class ReachabilityFeature : public Feature
{
public:
    ReachabilityFeature() : Feature() {}
    FeatureVect getFeatureCount(const API::Trajectory& t);
};

class LegibilityFeature : public Feature
{
public:
    LegibilityFeature() : Feature() {}
    FeatureVect getFeatureCount(const API::Trajectory& t);
};

}

#endif // HRICS_FEATURES_HPP

#ifndef HRICS_FEATURES_HPP
#define HRICS_FEATURES_HPP

#include <API/Device/robot.hpp>

namespace HRICS
{
class Distance;
class Visibility;
class Natural;

class Feature
{
public:
    Feature(Robot* r) : robot_(r) {}
    virtual double getValue();
    Robot* robot_;
};

class DistanceFeature : public Feature
{
public:
    DistanceFeature(Robot* r) : Feature(r) {}
    double getValue();
};

class VisibilityFeature : public Feature
{
public:
    VisibilityFeature(Robot* r) : Feature(r) {}
    double getValue();
};

class ReachabilityFeature : public Feature
{
public:
    ReachabilityFeature(Robot* r) : Feature(r) {}
    double getValue();
};

class LegibilityFeature : public Feature
{
public:
    LegibilityFeature(Robot* r) : Feature(r) {}
    double getValue();
};

}

#endif // HRICS_FEATURES_HPP

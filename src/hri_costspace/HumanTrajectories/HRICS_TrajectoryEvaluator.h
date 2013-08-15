#ifndef HRICS_TRAJECTORYEVALUATOR_H
#define HRICS_TRAJECTORYEVALUATOR_H

#include "HRICS_features.hpp"
#include <vector>

typedef std::vector<std::vector<double> > trajectory_t;

class TrajectorySampler
{
public:
    TrajectorySampler();

private:

};

class TrajectoryEvaluator
{
public:
    TrajectoryEvaluator();
    std::vector<double> getFeatureCount( );

private:
    double getControlCost();
    std::vector<HRICS::Feature> features_;
};

#endif // HRICS_TRAJECTORYEVALUATOR_H

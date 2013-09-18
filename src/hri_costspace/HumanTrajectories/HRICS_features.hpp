#ifndef HRICS_FEATURES_HPP
#define HRICS_FEATURES_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>

#include "API/Trajectory/trajectory.hpp"

#include "planner/TrajectoryOptim/Stomp/control_cost.hpp"
#include "planner/TrajectoryOptim/Stomp/covariant_trajectory_policy.hpp"
#include "planner/TrajectoryOptim/Chomp/chompTrajectory.hpp"

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
    virtual FeatureVect getFeatures(const Configuration& q) = 0;

    double cost( Configuration& q );

    void setWeights( const WeightVect& w ) { w_ = w; }
    WeightVect getWeights() { return w_; }
    int getNumberOfFeatures() { return w_.size(); }

protected:

    FeatureVect w_;
};

class StackedFeatures : public Feature
{
public:
    StackedFeatures();

    virtual FeatureVect getFeatureCount(const API::Trajectory& t);
    virtual FeatureVect getFeatures(const Configuration& q);

    void addFeatureFunction( Feature* fct );

protected:

    int nb_features_;
    std::vector<Feature*> feature_stack_;
};

class TrajectorySmoothness : public Feature
{
public:
    TrajectorySmoothness();

    //! Returns a smoothness cost for the trajectory
    FeatureVect getFeatureCount(const API::Trajectory& t);
    FeatureVect getFeatures(const Configuration& q);

    void setActivejoints( const std::vector<int>& active_joints );

private:
    double computeControlCost( const Eigen::MatrixXd& traj );

    ControlCost control_cost_;
    std::vector<int> active_dofs_;
};

}

#endif // HRICS_FEATURES_HPP

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
    double costTraj( const API::Trajectory& t );

    virtual void setWeights( const WeightVect& w ) { w_ = w; }
    virtual WeightVect getWeights() { return w_; }
    virtual int getNumberOfFeatures() { return w_.size(); }
    virtual void printWeights() const { std::cout << " w_.transpose() : " << w_.transpose() << std::endl; }

protected:

    FeatureVect w_;
};

class StackedFeatures : public Feature
{
public:
    StackedFeatures();

    virtual FeatureVect getFeatureCount(const API::Trajectory& t);
    virtual FeatureVect getFeatures(const Configuration& q);

    void setWeights( const WeightVect& w );
    WeightVect getWeights();

    void addFeatureFunction( Feature* fct );
    int getNumberOfFeatures() { return nb_features_; }

    void printWeights() const;

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

    void setActiveDofs( const std::vector<int>& active_dofs );

    //! Prints the control cost along the trajectory
    void printControlCosts( const std::vector<Eigen::VectorXd>& control_cost  );

private:
    double computeControlCost( const Eigen::MatrixXd& traj );

    ControlCost control_cost_;
    std::vector<int> active_dofs_;
};

}

#endif // HRICS_FEATURES_HPP

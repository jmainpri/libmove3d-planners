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

typedef Eigen::VectorXd FeatureProfile;
typedef Eigen::VectorXd FeatureVect;
typedef Eigen::MatrixXd FeatureJacobian;
typedef Eigen::VectorXd WeightVect;

////////////////////////////////////////
class Feature
{
public:
    Feature() {}

    virtual FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_features = std::vector<int>(0)) = 0;
    virtual FeatureProfile getFeatureProfile(const Move3D::Trajectory& t);
    virtual FeatureVect getFeatureCount(const Move3D::Trajectory& t);

    virtual FeatureJacobian getFeaturesJacobian(const Move3D::Configuration& q );
    virtual FeatureProfile getFeatureJacobianProfile(const Move3D::Trajectory& t);
    virtual FeatureJacobian getFeatureJacobian(const Move3D::Trajectory& t);
    virtual double getFeaturesJacobianMagnitude(const Move3D::Configuration& q);

    double getJacobianSum(const Move3D::Trajectory& t);

    double cost( Move3D::Configuration& q );
    double costTraj( const Move3D::Trajectory& t );

    virtual void setWeights( const WeightVect& w );
    virtual WeightVect getWeights() { return w_; }

    virtual void setActiveFeatures( const std::vector<int>& active_features ) { active_features_ = active_features; }
    virtual const std::vector<int>& getActiveFeatures() const { return active_features_; }

    virtual void setActiveDoFs( const std::vector<int>& active_dofs ) { active_dofs_ = active_dofs; }
    virtual const std::vector<int>& getActiveDoFs() const { return active_dofs_; }

    virtual int getNumberOfFeatures() const { return w_.size(); }
    virtual void printWeights() const { std::cout << " w_.transpose() : " << w_.transpose() << std::endl; }

protected:
    std::vector<int> active_dofs_;
    std::vector<int> active_features_;
    FeatureVect w_;
};

////////////////////////////////////////
class StackedFeatures : public Feature
{
public:
    StackedFeatures();

    virtual FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    virtual FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));

    void setWeights( const WeightVect& w );
    WeightVect getWeights();

    void setActiveFeatures( const std::vector<int>& active_features );

    bool addFeatureFunction( Feature* fct );
    int getNumberOfFeatures() { return nb_features_; }

    void printWeights() const;
    void printStackInfo() const;

    Feature* getFeatureFunction(int i) { return feature_stack_[i]; }

protected:
    int nb_features_;
    std::vector<Feature*> feature_stack_;
};

////////////////////////////////////////
class TrajectorySmoothness : public Feature
{
public:
    TrajectorySmoothness();

    //! Returns a smoothness cost for the trajectory
    FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));

    //! Prints the control cost along the trajectory
    void printControlCosts( const std::vector<Eigen::VectorXd>& control_cost  );

private:
    double computeControlCost( const Eigen::MatrixXd& traj );
    ControlCost control_cost_;
};

}

#endif // HRICS_FEATURES_HPP

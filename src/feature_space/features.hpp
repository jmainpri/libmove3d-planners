/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#ifndef HRICS_FEATURES_HPP
#define HRICS_FEATURES_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>

#include "API/ConfigSpace/configuration.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "API/Roadmap/graph.hpp"

#include "planner/TrajectoryOptim/Stomp/control_cost.hpp"

namespace Move3D
{
typedef Eigen::VectorXd FeatureProfile;
typedef Eigen::VectorXd FeatureVect;
typedef Eigen::MatrixXd FeatureJacobian;
typedef Eigen::VectorXd WeightVect;

////////////////////////////////////////
class Feature
{
public:
    Feature(std::string name) : is_stacked_(false), name_(name) {}

    std::string getName() { return name_; }

    virtual FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_features = std::vector<int>(0)) = 0;
    virtual FeatureProfile getFeatureProfile(const Move3D::Trajectory& t);
    virtual FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    virtual FeatureVect getFeatureCount( Move3D::LocalPath& p, int& nb_calls );

    virtual FeatureJacobian getFeaturesJacobian(const Move3D::Configuration& q );
    virtual FeatureProfile getFeatureJacobianProfile(const Move3D::Trajectory& t);
    virtual FeatureJacobian getFeatureJacobian(const Move3D::Trajectory& t);
    virtual double getFeaturesJacobianMagnitude(const Move3D::Configuration& q);

    double getJacobianSum(const Move3D::Trajectory& t);

    double cost( Move3D::Configuration& q );
    double costTraj( const Move3D::Trajectory& t );
    virtual double costPath( Move3D::LocalPath& p, int& nb_calls );

    virtual void setWeights( const WeightVect& w );
    virtual WeightVect getWeights() const { return w_; }

    virtual void setActiveFeatures( const std::vector<int>& active_features ) { active_features_ = active_features; }
    virtual const std::vector<int>& getActiveFeatures() const { return active_features_; }

    virtual void setActiveDoFs( const std::vector<int>& active_dofs ) { active_dofs_ = active_dofs; }
    virtual const std::vector<int>& getActiveDoFs() const { return active_dofs_; }

    virtual int getNumberOfFeatures() const { return w_.size(); }
    virtual void printWeights() const { std::cout << " w_.transpose() : " << w_.transpose() << std::endl; }
    virtual void printInfo() const { }

    std::vector<Move3D::Trajectory*> extractAllTrajectories( Move3D::Graph* g, Move3D::confPtr_t q_init, Move3D::confPtr_t q_goal, int nb_divisions );

    bool is_active_;

protected:
    std::vector<int> active_dofs_;
    std::vector<int> active_features_;
    FeatureVect w_;
    bool is_stacked_;
    std::string name_;
};

////////////////////////////////////////
class StackedFeatures : public Feature
{
public:
    StackedFeatures();

    virtual FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
    virtual FeatureVect getFeatureCount(const Move3D::Trajectory& t);

    void setWeights( const WeightVect& w );
    WeightVect getWeights() const;

    void setActiveFeatures( const std::vector<int>& active_features );
    void setActiveFeatures( const std::vector<std::string>& active_features );
    void setAllFeaturesActive();

    bool addFeatureFunction( Feature* fct );
    int getNumberOfFeatureFunctions() { return feature_stack_.size(); }
    int getNumberOfFeatures() { return nb_features_; }

    void printWeights() const;
    void printInfo() const;

    Feature* getFeatureFunction(int i) { return feature_stack_[i]; }
    Feature* getFeatureFunction(std::string name);

protected:
    int nb_features_;
    std::vector<Feature*> feature_stack_;
};

////////////////////////////////////////
class LengthFeature : public Move3D::Feature
{
public:
    LengthFeature();
    Move3D::FeatureVect getFeatureCount( const Move3D::Trajectory& t );
    Move3D::FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0) );
    double scaling_;
};

////////////////////////////////////////
class TrajectorySmoothness : public Feature
{
public:
    TrajectorySmoothness();

    //! Returns a smoothness cost for the trajectory
    FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
    FeatureVect getFeatureCount(const Move3D::Trajectory& t);

    void setWeights(const WeightVect &w);

    //! Prints the control cost along the trajectory
    void printControlCosts( const std::vector<Eigen::VectorXd>& control_cost  );

private:
    double computeControlCost( const Eigen::MatrixXd& traj );
    ControlCost control_cost_;
};

}

extern Move3D::Feature* global_activeFeatureFunction;

#endif // HRICS_FEATURES_HPP

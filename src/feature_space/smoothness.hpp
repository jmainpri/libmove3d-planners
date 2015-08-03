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

#ifndef SMOOTHNESS_HPP
#define SMOOTHNESS_HPP

#include "features.hpp"
#include "planner/TrajectoryOptim/Stomp/control_cost.hpp"

//! Ugly...
#include "hri_costspace/human_trajectories/HRICS_human_features.hpp"

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API

#include <vector>

namespace Move3D
{

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

    //! get smoothed trajectory as matrix
    Eigen::MatrixXd getSmoothedTrajectory( const Move3D::Trajectory& t ) const;

    //! returns the control costs
    double getControlCosts(const Eigen::MatrixXd& traj_smooth, std::vector<Eigen::VectorXd>& control_costs, double dt );

    //! Set Buffer set buffer
    virtual void setBuffer(const std::vector<Eigen::VectorXd>& buffer);

    //! Set Buffer as not filled
    void clearBuffer();

    //! Save profile to file
    void saveAbsValuesToFile(const Move3D::Trajectory& t, std::string folder ) const;

    //! get diff rule
    int get_left_padding() { return control_cost_.get_left_padding(); }
    int get_right_padding() { return control_cost_.get_right_padding(); }
    int get_diff_rule() { return control_cost_.getDiffRuleLength(); }

protected:
    ControlCost control_cost_;
    bool buffer_is_filled_;

private:
    double computeControlCost( const Eigen::MatrixXd& traj );

};

////////////////////////////////////////
class LengthFeature : public TrajectorySmoothness
{
public:
    LengthFeature();
    Move3D::FeatureVect getFeatureCount( const Move3D::Trajectory& t );
    Move3D::FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0) );
    double getControlCosts(const Eigen::MatrixXd& traj_smooth, std::vector<Eigen::VectorXd>& control_costs);
    double scaling_;
};

////////////////////////////////////////
class VelocitySmoothness : public TrajectorySmoothness
{
public:
    VelocitySmoothness();
    void setWeights(const WeightVect &w);
};

////////////////////////////////////////
class AccelerationSmoothness : public TrajectorySmoothness
{
public:
    AccelerationSmoothness();
    void setWeights(const WeightVect &w);
};

////////////////////////////////////////
class JerkSmoothness : public TrajectorySmoothness
{
public:
    JerkSmoothness();
    void setWeights(const WeightVect &w);
};

////////////////////////////////////////
class TaskSmoothnessFeature : public TrajectorySmoothness
{
public:
    TaskSmoothnessFeature( Move3D::Robot* active, Move3D::Joint* joint_task );
    Move3D::FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));

    Eigen::MatrixXd getTaskTrajectory( const Move3D::Trajectory& t );
    Eigen::VectorXd getTaskPose( Move3D::confPtr_t _q );

    void saveAbsValuesToFileVelocity(const Move3D::Trajectory& t, std::string folder );
    void saveAbsValuesToFileAcceleration(const Move3D::Trajectory& t, std::string folder );
    void saveAbsValuesToFileJerk(const Move3D::Trajectory& t, std::string folder );

    // Compute velocity between two configurations
    double getDist( const Move3D::Trajectory& t,Eigen::VectorXd& control_costs );
    double getVelocity( const Move3D::Trajectory& t,Eigen::VectorXd& control_costs );
    double getAcceleration( const Move3D::Trajectory& t, Eigen::VectorXd& control_costs );
    double getJerk( const Move3D::Trajectory& t, Eigen::VectorXd& control_costs );

    double getDist( const Eigen::MatrixXd& t,Eigen::VectorXd& control_costs );
    double getVelocity( const Eigen::MatrixXd& t,Eigen::VectorXd& control_costs, double dt );
    double getAcceleration(const  Eigen::MatrixXd& t, Eigen::VectorXd& control_costs, double dt );
    double getJerk( const Eigen::MatrixXd& t, Eigen::VectorXd& control_costs, double dt );

    Eigen::VectorXd getControlCosts( const std::vector<Eigen::VectorXd>& control_cost, double dt ) const;

    //! Set Task buffer
    void setBuffer( const std::vector<Eigen::VectorXd>& buffer );

    // Draw velocities
    void draw();

private:

    Move3D::Robot* robot_;
    std::vector<int> veclocity_joint_ids_;
    std::vector<Move3D::Joint*> task_joints_;
};

////////////////////////////////////////
class SmoothnessFeature : public Move3D::Feature
{
public:
    SmoothnessFeature(Move3D::Robot* robot, Move3D::Joint* joint_task);
    Move3D::FeatureVect getFeatureCount( const Move3D::Trajectory& t );
    Move3D::FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0) );
    void setActiveDoFs( const std::vector<int>& active_dofs );
    void setWeights(const WeightVect &w);

    //! Set Buffer
    void setBuffer(const std::vector<Eigen::VectorXd>& buffer);
    void clearBuffer();

    LengthFeature length_;
    VelocitySmoothness velocity_;
    AccelerationSmoothness acceleration_;
    JerkSmoothness jerk_;
    TaskSmoothnessFeature task_features_;
};



}

extern double smoothness_phi_coeff_0;
extern double smoothness_phi_coeff_1;
extern double smoothness_phi_coeff_2;
extern double smoothness_phi_coeff_3;
extern double smoothness_phi_coeff_4;
extern double smoothness_phi_coeff_5;
extern double smoothness_phi_coeff_6;
extern double smoothness_phi_coeff_7;

#endif // SMOOTHNESS_HPP

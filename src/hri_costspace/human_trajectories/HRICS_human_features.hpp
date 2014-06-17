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
#ifndef HRICS_HUMANFEATURES_HPP
#define HRICS_HUMANFEATURES_HPP

#include "feature_space/features.hpp"

#include "hri_costspace/HRICS_distance.hpp"
#include "hri_costspace/HRICS_visibility.hpp"
#include "hri_costspace/HRICS_natural.hpp"

namespace HRICS
{

extern Move3D::FeatureVect w_distance_16;

class DistanceFeature : public Move3D::Feature
{
public:
    DistanceFeature( Move3D::Robot* active, Move3D::Robot* passive );
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
    Move3D::FeatureVect computeDistances() const;
    std::vector<std::string> getDistanceNames() { return distance_names_; }

    void draw();

private:
    Move3D::Robot* human_active_;
    Move3D::Robot* human_passive_;

    Distance* dist_cost_;

    std::vector<int> distance_joint_ids_;

    std::vector<std::string> distance_names_;

    std::vector<Move3D::Joint*> human_active_joints_;
    std::vector<Move3D::Joint*> human_passive_joints_;
};

class VelocityFeature : public Move3D::Feature
{
public:
    VelocityFeature( Move3D::Robot* active );
    Move3D::FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));

    // Compute velocity between two configurations
    std::vector<Eigen::Vector3d> getVelocity(const Move3D::Configuration& q_0, const Move3D::Configuration& q_1, double dt);

    // Draw velocities
    void draw();

    Move3D::Robot* getRobot() { return human_active_; }

private:

    void stackVelocities( Move3D::FeatureVect& stack , const std::vector<Eigen::Vector3d>& velocities );

    Move3D::Robot* human_active_;
    std::vector<int> veclocity_joint_ids_;
    std::vector<Move3D::Joint*> human_active_joints_;
    Move3D::confPtr_t q_last_;
};

class CollisionFeature : public Move3D::Feature
{
public:
    CollisionFeature( Move3D::Robot* robot );
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));

    // Set weights
    void setWeights( const Move3D::WeightVect& w );

    // Compute collision between two configurations
    double getCollisionCost( const Move3D::Configuration& q );

    //! init collision space
    bool init();

    // Draw collision
    void draw();

private:

    Move3D::Robot* robot_;
};

class VisibilityFeature : public Move3D::Feature
{
public:
    VisibilityFeature() : Feature("Visbility") {}
    Move3D::FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
};

class MuskuloskeletalFeature : public Move3D::Feature
{
public:
    MuskuloskeletalFeature() : Feature("Muskuloskeletal") {}
    Move3D::FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
};

class ReachabilityFeature : public Move3D::Feature
{
public:
    ReachabilityFeature() : Feature("Reachability") {}
    Move3D::FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
};

class LegibilityFeature : public Move3D::Feature
{
public:
    LegibilityFeature() : Feature("Legibility") {}
    Move3D::FeatureVect getFeatureCount(const Move3D::Trajectory& t);
    Move3D::FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
};

}

#endif // HRICS_HUMANFEATURES_HPP

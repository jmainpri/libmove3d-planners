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

#ifndef HRICS_HUMAN_COSTSPACE_HPP
#define HRICS_HUMAN_COSTSPACE_HPP

#include "API/Device/robot.hpp"

#include "HRICS_human_features.hpp"
#include "hri_costspace/gestures/HRICS_record_motion.hpp"

namespace HRICS
{

class HumanTrajCostSpace : public Move3D::StackedFeatures
{

public:
    HumanTrajCostSpace( Move3D::Robot* active, Move3D::Robot* passive );
    ~HumanTrajCostSpace();

    void draw() { }

//    FeatureVect getFeatureCount(const Move3D::Trajectory& t);
//    FeatureVect getFeatures(const Configuration& t);

    //! Add a passive trajectory
    void setPassiveTrajectory( const motion_t& traj );

    //! Set the passive configuration
    void setPassiveConfig( const Move3D::Configuration& q );

    Move3D::Robot* getActiveHuman() { return human_active_; }
    Move3D::Robot* getPassiveHuman() { return human_passive_; }

    //! init collision space
    bool initCollisionSpace() { return collision_feat_.init(); }

private:

    Move3D::Robot* human_active_;
    Move3D::Robot* human_passive_;

    Move3D::Trajectory passive_traj_;
    int nb_way_points_;

    DistanceFeature dist_feat_;
    VisibilityFeature visi_feat_;
    MusculoskeletalFeature musc_feat_;
    ReachabilityFeature reach_feat_;
    LegibilityFeature legib_feat_;
    CollisionFeature collision_feat_;

    Move3D::LengthFeature length_feat_;
    Move3D::TrajectorySmoothness smoothness_feat_;
};

}

extern HRICS::HumanTrajCostSpace* global_ht_cost_space;

//! main test function for human planning
void HRICS_run_human_planning();
bool HRICS_init_human_trajectory_cost();

#endif // HRICS_HUMAN_COSTSPACE_HPP

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

#ifndef HRICS_HUMAN_SIMULATOR_HPP
#define HRICS_HUMAN_SIMULATOR_HPP

#include "HRICS_human_cost_space.hpp"

namespace HRICS
{

class HumanTrajSimulator
{
public:
    HumanTrajSimulator( HumanTrajCostSpace* cost_space );

    bool init();
    double run();

    std::vector<Move3D::Joint*> getActiveJoints() { return active_joints_; }

    std::vector< std::vector<motion_t> > getMotions();
    std::vector<Move3D::Trajectory> getDemoTrajectories() const;
    std::vector<Move3D::confPtr_t> getContext() const;

private:

    // ------------------------------------------------------------------------

    void loadHumanTrajectory( const motion_t& motion );
    Eigen::Transform3d getHumanPose();
    void setPassiveHumanConfig( Move3D::confPtr_t q );
    void setMatrixCol(Eigen::MatrixXd& matrix, int j, Move3D::confPtr_t q) const;
    bool updateMotion();
    bool loadActiveHumanGoalConfig();
    void printCosts() const;
    Move3D::Trajectory setTrajectoryToMainRobot(const Move3D::Trajectory& traj) const;
    void runStandardStomp( int iter );
    void execute(const Move3D::Trajectory& path, bool to_end = true);

    // ------------------------------------------------------------------------

    void setPelvisBounds();
    void setReplanningDemonstrations();
    void setHumanColor( Move3D::Robot* human, int color);
    void setActiveJoints();
    void addCutMotions();

    // ------------------------------------------------------------------------

    int id_of_demonstration_;
    motion_t human_passive_motion_;
    Eigen::MatrixXd current_human_traj_;
    Move3D::confPtr_t q_init_;
    Move3D::confPtr_t q_goal_;
    Move3D::Trajectory executed_path_;
    Move3D::Trajectory path_;

    std::vector<double> cost_;
    std::vector<double> colors_;
    int best_path_id_;
    int current_frame_;
    int human_passive_increment_;
    int human_active_increments_per_exection_;
    double human_active_step_;
    bool is_scenario_init_;

    // ------------------------------------------------------------------------

    HumanTrajCostSpace* cost_space_;

    Move3D::Robot* human_active_;
    Move3D::Robot* human_passive_;

    bool init_scenario_;

    std::string traj_folder_;

    std::vector<Move3D::Joint*> active_joints_;
    std::vector<int> active_dofs_;

    std::vector<HRICS::RecordMotion*> motion_recorders_;
    std::vector<motion_t> human_1_motions_;
    std::vector<motion_t> human_2_motions_;
    Eigen::VectorXd pelvis_max_;
    Eigen::VectorXd pelvis_min_;
    int minimal_demo_size_;
    bool trajectories_cut_;
};

}

extern HRICS::HumanTrajSimulator* global_ht_simulator;

#endif // HRICS_HUMAN_SIMULATOR_HPP

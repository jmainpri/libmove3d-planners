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
#ifndef HRICS_HUMANPREDICTIONSIMULATOR_HPP
#define HRICS_HUMANPREDICTIONSIMULATOR_HPP

#include "HRICS_record_motion.hpp"
#include "HRICS_classify_motion.hpp"
#include "HRICS_workspace_occupancy.hpp"

#include "API/Trajectory/trajectory.hpp"

#include <string>

void HRICS_initOccupancyPredictionFramework(std::string robot, std::string human);

namespace HRICS
{
class HumanPredictionSimulator
{
public:
    HumanPredictionSimulator( Move3D::Robot* robot, Move3D::Robot* human, RecordMotion* recorder, ClassifyMotion* classifier, WorkspaceOccupancyGrid* occupacy_grid );

    void loadHumanTrajectory( const motion_t& motion );
    int classifyMotion( const motion_t& motion );
    void runVoxelOccupancy();
    void setHumanTransform( const Eigen::Transform3d& t ) { m_T_human = t; }
    Eigen::Transform3d getHumanPose();
    double run();


private:
    bool updateMotion();
    void predictVoxelOccupancy();
    void loadGoalConfig();
    Move3D::Trajectory setTrajectoryToMainRobot(const Move3D::Trajectory& traj) const;
    void runMultipleStomp( int iter );
    void runParallelStomp( int iter, int id_goal );
    void runStandardStomp(int iter, int id_goal );
    void execute(const Move3D::Trajectory& path, bool to_end=false);
    int getBestPathId();
    void setMatrixCol(Eigen::MatrixXd& mat, int j, Move3D::confPtr_t q);
    void setHumanConfig( Move3D::confPtr_t q );
    void printCosts() const;

    Move3D::Robot* m_robot;
    Move3D::Robot* m_human;
    motion_t m_motion;
    Eigen::MatrixXd m_current_human_traj;
    int m_human_increment;
    RecordMotion* m_recorder;
    ClassifyMotion* m_classifier;
    WorkspaceOccupancyGrid* m_occupacy_grid;

    Move3D::confPtr_t m_q_start;
    std::vector<Move3D::confPtr_t> m_goal_config;
    std::vector<Move3D::Trajectory> m_paths;

    Move3D::Trajectory m_executed_path;
    std::vector< std::vector<double> > m_cost;
    std::vector< std::vector<double> > m_colors;

    bool m_use_previous_trajectory;
    bool m_is_scenario_init;

    int m_best_path_id;
    int m_robot_steps_per_exection;
    double m_robot_step;

    bool m_use_openrave;

    // Drawing translation
    Eigen::Transform3d m_T_human;

    // Human joint mapping
    std::map<std::string,int> m_human_dof_map;
};
}

extern HRICS::HumanPredictionSimulator* global_humanPredictionSimulator;

#endif // HRICS_HUMANPREDICTIONSIMULATOR_HPP

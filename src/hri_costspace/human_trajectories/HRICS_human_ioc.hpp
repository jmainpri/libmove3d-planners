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
#ifndef HRICS_HUMANIOC_HPP
#define HRICS_HUMANIOC_HPP

#include "HRICS_ioc.hpp"
#include "hri_costspace/gestures/HRICS_record_motion.hpp"

namespace HRICS
{

class HumanIoc : public IocEvaluation
{
public:
    HumanIoc( Move3D::Robot* active, Move3D::Robot* passive, int nb_demos, int nb_samples, int nb_way_points,
              MultiplePlanners& planners, Move3D::StackedFeatures* features, std::vector<int> active_joints,
              std::string folder,  std::string traj_folder, std::string tmp_data_folder );

//    void runSampling();
    void setPlanningGroup();
    void setDemos( const std::vector<motion_t>& stored_motions );

private:
    Move3D::Trajectory getTrajectoryFromMotion( const motion_t& m ) const;

};

}

void HRICS_run_human_ioc_from_recorded_motion();
void HRICS_run_human_ioc_evaluation();

#endif // HRICS_HUMANIOC_HPP

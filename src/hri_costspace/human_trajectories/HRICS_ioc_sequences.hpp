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
#ifndef HRICS_IOC_SEQUENCES_HPP
#define HRICS_IOC_SEQUENCES_HPP

#include "feature_space/features.hpp"
#include "HRICS_human_ioc.hpp"

#include <iostream>

namespace HRICS
{

class IocSequences
{
public:
    IocSequences();
    ~IocSequences() { }

    bool run();
    void set_features();

private:

    void setSamplingFeatures();
    void setSamplingFeaturesIk();
    void setGenerationFeatures();
    void setCompareFeatures();
    void GenerateResults();


    // IOC PHASES
    enum phase_t {
        generate=0,
        sample=1,
        compare=2,
        run_planner=3,
        default_phase=4,
        monte_carlo=5,
        simulation=6,
        save_feature_and_demo_size=7,
        generate_results=8};

    // TYPE OF FEATURES
    enum feature_t {
        no_features,
        spheres,
        human_trajs };

    // SAMPLE IK
    enum sample_ik_t {
        no_ik=0,
        only_ik=1,
        ik_and_traj=2 };

    phase_t phase_;
    feature_t features_type_;
    sample_ik_t sample_ik_;

    std::vector<int> active_joints_;
    Move3D::StackedFeatures* feature_fct_;
    bool use_human_simulation_demo_;
    HRICS::IocEvaluation* eval_;

    // Trajectories results
    std::vector<Move3D::Trajectory> demos_;
    std::vector< std::vector<Move3D::Trajectory> >
    baseline_agressive_;
    std::vector< std::vector<Move3D::Trajectory> >
    baseline_conservative_;
    std::vector< std::vector<Move3D::Trajectory> >
    recovered_;
    std::vector< std::vector<Move3D::Trajectory> >
    noreplan_baseline_agressive_;
    std::vector< std::vector<Move3D::Trajectory> >
    noreplan_baseline_conservative_;
    std::vector< std::vector<Move3D::Trajectory> >
    noreplan_recovered_;
};

}

void hrics_ioc_compute_results();

#endif // HRICS_IOC_SEQUENCES_HPP

/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI, MPI
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

#include "trajOptimizer.hpp"
#include "lampTrajectory.hpp"
#include "lampComputeCost.hpp"

#ifndef LAMP_STOMP_LOOP_H_
#define LAMP_STOMP_LOOP_H_

namespace Move3D
{

class Stomp : public SamplingBasedInnerLoop
{
public:
    Stomp( Move3D::Robot* robot ) : SamplingBasedInnerLoop(robot) { }

    bool initialize( bool single_rollout, double discretization );
    void run_single_iteration();
    void end();
    bool rollout_probabilities();
    bool compute_parameter_updates( const std::vector<Move3D::VectorTrajectory> & deltas, Eigen::VectorXd& parameter_updates );
    bool precompute_projection_matrices();

private:

    std::vector< Eigen::VectorXd > probabilities_;
    Eigen::MatrixXd projection_matrix_;

};

}

#endif /* LAMP_STOMP_LOOP_H_ */

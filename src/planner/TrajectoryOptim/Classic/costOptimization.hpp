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

#ifndef COST_OPTIMIZATION_HPP_
#define COST_OPTIMIZATION_HPP_

#include "API/ConfigSpace/configuration.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"

/**
 * @ingroup Trajectory
 * @brief Genera Cost Optimization of a trajectory
 */
namespace Move3D 
{
class CostOptimization : public Smoothing
{
public:

    /**
      * Constructors and Destructors of the class
      */
    CostOptimization();
    CostOptimization( const Trajectory& T );
    CostOptimization( Robot* R, traj* t );

    ~CostOptimization();
    
    /**
      * Prints debug information
      */
    void printDebugInfo();

    /**
      * Returns true if the new trajectory is in collision
      */
    bool deformInCollision() { return m_inCollision; }

    /**
      * Set the cheat
      */
    void setCheat() { m_cheat = true; }
    
    /**
      * Get the minimal cost
      */
    double getMinCost() { return m_mincost; }
    
    /**
     * One loop of the deformation strategy
     * @param step is the distance between 2 configurations
     */
    bool oneLoopDeform();

    /**
     * One loop of the deformation strategy with recomputing
     * of the trajectory portion cost as it might change
     * @param step is the distance between 2 configurations
     */
    bool oneLoopDeformRecompute();

    /**
     * Stops at the last descending configuration
     * on the cost map
     */
    static double getLastDescendingConfParam( LocalPath& directionPath );
    
    /**
     * Adds a configuration to the trajectory by starting from
     * the begining of the trajectory
     */
    bool connectConfigurationToClosestAtBegin( confPtr_t q, double step, bool consider_valid );

    /**
     * Adds a configuration to the trajectory by starting from
     * the begining of the trajectory
     */
    bool connectConfigurationToBegin( confPtr_t q, double step, bool consider_cost=false );

    /**
     * Adds a configuration to the trajectory by starting from
     * the end of the trajectory
     */
    bool connectConfigurationToEnd( confPtr_t q, double step, bool consider_cost=false );

    /**
     * Expand the configuration to a ne
     */
    static confPtr_t perturbCurrent( confPtr_t qCurrPt, confPtr_t qRandPt, double step, bool descent );

    /**
     *
     */
    void runDeformation( int nbIteration , int idRun=0 );
    

protected:

    /**
     * Cheat for Justin
     */
    confPtr_t cheat();

    /**
     * Create new trajectories to show in debug mode
     * also calls the g3d_draw function to plot in the OpenGl display
     */
    void debugShowTraj(double lPrev,double lNext, confPtr_t qNew , int color);
    
    /**
     * Returns 3 random configurations along the trajtectory
     * @param the step between the 3 configuration
     * @return vector of configuration
     * @return
     */
    std::vector<confPtr_t> get3RandSuccesConfAlongTraj(
            double& prevDistPt,
            double& randDistPt,
            double& nextDistPt,
            double step);

    /**
     * Returns the 3 configurations that are the closest to the input configuration
     * @param
     */
    std::vector<confPtr_t> getClosestConfOnTraj(
            double& prevDistPt,
            double& randDistPt,
            double& nextDistPt,
            confPtr_t ptrConf,
            double step);


private:

    bool					m_cheat;
    double					m_mincost;
    unsigned int			m_nbErrors;
    std::vector<double>		m_Errors;
    bool					m_DeformBiased;
    bool					m_inCollision;
    bool					m_descent;
    int                     m_shortcutRatio;

    //	bool oneLoopShortCut(double step);
};
}

#endif /* COST_OPTIMIZATION_HPP_ */

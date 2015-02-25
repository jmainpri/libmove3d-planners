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
#ifndef COST_SPACE_HPP_INCLUDED
#define COST_SPACE_HPP_INCLUDED

#include <boost/function.hpp>
#include <map>
#include <string>

#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/localpath.hpp"
#include "API/Roadmap/graph.hpp"

namespace Move3D {

/*!
 * Delta step cost method enum
 */
enum CostSpaceDeltaStepMethod 
{
    cs_integral,
    cs_mechanical_work,
    cs_visibility,
    cs_average,
    cs_config_cost_and_dist,
    cs_boltzman_cost,
    cs_max
};

/*!
 * Delta step cost method enum
 */
enum CostSpaceResolutionMethod
{
    cs_classic,
    cs_pr2_manip
};

/*!
 * Class thats holding the CostSpace
 */
class CostSpace
{
public:
    CostSpace();

    // Get selected cost function name
    std::string getSelectedCostName();

    // Get All Cost Functions
    std::vector<std::string> getAllCost();

    // Select the cost function with the given name in the map
    bool setCost(std::string name);

    // Register a new cost function.
    void addCost(std::string name, boost::function<double(Configuration&)> f);

    // Select the cost function with the given name in the map
    bool setPathCost(std::string name);

    // Register a new cost function.
    void addPathCost(std::string name, boost::function<double(LocalPath&,int&)> f);

    // Delete a cost function
    void deleteCost(std::string name);

    // Compute the cost of the configuration conf.
    double cost(Configuration& conf);

    // Compute the cost of
    double cost(LocalPath& path, int& nb_test);

    //! The jacobian is of m by n -> nb features by active dofs
    //! column per dof
    //! row per feature
    Eigen::VectorXd getJacobian( const Configuration& q_0, std::vector<int> active_dofs = std::vector<int>() );

    //! The Hessian
    Eigen::MatrixXd getHessian( const Configuration& q, std::vector<int> active_dofs = std::vector<int>() );

    // Set node cost
    void setNodeCost(Node* node, Node* parent);

    // Initializes the Cost space motion planning problem
    void initMotionPlanning(graph* graph, node* start, node* goal);

    // Set DeltaStepCost
    void setDistanceMethod(CostSpaceResolutionMethod method) { m_resolution = method; }

    // Set DeltaStepCost
    void setDeltaStepMethod(CostSpaceDeltaStepMethod method) { m_deltaMethod = method; }

    // Get DeltaStepCost
    CostSpaceDeltaStepMethod getDeltaStepMethod() { return m_deltaMethod; }

    // Compute the delta step cost
    double deltaStepCost(double cost1, double cost2, double length);

protected:

    double path_cost_default(LocalPath& path, int& nb_test);

    std::string mSelectedCostName;
    boost::function<double(Configuration&)> mSelectedCost;

    std::string mSelectedPathCostName;
    boost::function<double(LocalPath&,int&)> mSelectedPathCost;

    std::map<std::string, boost::function<double(Configuration&)> > mFunctions;
    std::map<std::string, boost::function<double(LocalPath&,int&)> > mPathFunctions;

    void getPr2ArmConfiguration( Eigen::VectorXd& x, confPtr_t q );
    double getPr2ArmDistance( Robot* robot, Eigen::VectorXd& q_i, Eigen::VectorXd& q_f );

private:

    // Delta
    enum CostSpaceDeltaStepMethod m_deltaMethod;
    enum CostSpaceResolutionMethod m_resolution;

    double m_dmax;
};

namespace GlobalCostSpace 
{
void initialize();
};

double computeIntersectionWithGround(Configuration& conf);
double computeFlatCost(Configuration& conf);
double computeDistanceToObstacles(Configuration& conf);
double computeInCollisionCost(Configuration& conf);
double computeCollisionSpaceCost(Configuration& conf);
double computeLocalpathKinematicCost(void* rob, localpath* LP);

extern CostSpace* global_costSpace;

}

#endif

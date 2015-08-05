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
#include "cost_space.hpp"
#include <iostream>

#include "API/project.hpp"
#include "API/Roadmap/compco.hpp"

#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planEnvironment.hpp"

#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "GroundHeight-pkg.h"
#include "Planner-pkg.h"

#include "Collision-pkg.h"

#include <algorithm>

#include <boost/function.hpp>
#include <boost/bind.hpp>

extern void* GroundCostObj;

using namespace Move3D;
using namespace std;

MOVE3D_USING_SHARED_PTR_NAMESPACE

CostSpace* Move3D::global_costSpace(NULL);

//using std::string;
//------------------------------------------------------------------------------

void GlobalCostSpace::initialize()
{
    if (global_costSpace != NULL) {
        return ;
    }

    // initialize the cost function object.
    global_costSpace = new CostSpace();

    std::cout << "Initializing the dummy cost Function" << std::endl;
    global_costSpace->addCost("NoCost",boost::bind(computeFlatCost, _1));
    global_costSpace->setCost("NoCost");

    std::cout << "Initializing the dist to obst costmap cost function" << std::endl;
    global_costSpace->addCost("costDistToObst",boost::bind(computeDistanceToObstacles, _1));
    global_costSpace->setCost("costDistToObst");

    std::cout << "Initializing the collision costmap cost function" << std::endl;
    global_costSpace->addCost("costIsInCollision",boost::bind(computeInCollisionCost, _1));
    global_costSpace->setCost("costIsInCollision");

    std::cout << "Initializing the length cost function" << std::endl;
    global_costSpace->addCost("costLength",boost::bind(computeFlatCost, _1));
    global_costSpace->setCost("costLength");

    if( GroundCostObj )
    {
        std::cout << "Initializing the 2d costmap cost function" << std::endl;
        global_costSpace->addCost("costMap2D",boost::bind(computeIntersectionWithGround, _1));
        global_costSpace->setCost("costMap2D");
    }
}

//------------------------------------------------------------------------------
CostSpace::CostSpace() :
    mSelectedCostName("No Cost"),
    mSelectedPathCostName("Default"),
    m_deltaMethod(cs_integral),
    m_resolution(cs_classic)/* m_resolution(cs_pr2_manip)*/
{
    m_dmax = ENV.getDouble(Env::dmax);

    mSelectedPathCost = boost::bind( &CostSpace::path_cost_default, this, _1, _2 );
    mPathFunctions[ mSelectedCostName ] = mSelectedPathCost;
    //m_resolution = cs_pr2_manip;
}

//------------------------------------------------------------------------------
std::string CostSpace::getSelectedCostName()
{
    return mSelectedCostName;
}

//------------------------------------------------------------------------------
std::vector<string> CostSpace::getAllCost()
{
    std::vector<string> functions;
    std::map< string, boost::function<double(Configuration&)> >::iterator it;

    for ( it=mFunctions.begin() ; it != mFunctions.end(); it++ )
        functions.push_back((*it).first);

    return functions;
}

//------------------------------------------------------------------------------
bool CostSpace::setCost(std::string name)
{
    if( mFunctions.find(name) != mFunctions.end() )
    {
        mSelectedCostName = name;
        mSelectedCost = mFunctions[name];
        setPathCost( name );
        return true;
    }
    else
    {
        typedef std::map< std::string, boost::function<double(Configuration&) > >::const_iterator MapIterator;

        cout << name << " is : " << mFunctions.count(name) << endl;

        for (MapIterator iter = mFunctions.begin();
             iter != mFunctions.end(); iter++)
            cout << "Key: " << iter->first << endl; // << "Values:" << endl;

        std::cout << "Warning : in CostSpace::setCost(string name), could not find a cost function named " << name  << std::endl;
        return false;
    }
}

void CostSpace::addCost(std::string name, boost::function<double(Configuration&)> f)
{
    if( mFunctions.find(name) == mFunctions.end() )
    {
        mFunctions[name] = f;
    }
    else
    {
        std::cout << "Warning : in CostSpace::addCost, replacing the cost function named " << name << "by another." << std::endl;
        mFunctions[name] = f;
    }
}


//------------------------------------------------------------------------------
// Select the cost function with the given name in the map
bool CostSpace::setPathCost(std::string name)
{
    if( mPathFunctions.find(name) != mPathFunctions.end() )
    {
        mSelectedPathCostName = name;
        mSelectedPathCost = mPathFunctions[name];
        return true;
    }
    else
    {
        std::cout << "Warning : in CostSpace::setPathCost(string name), could not find a cost function named " << name  << std::endl;
        return false;
    }
}

// Register a new cost function.
void CostSpace::addPathCost(std::string name, boost::function<double(LocalPath&,int&)> f)
{
    if( mPathFunctions.find(name) == mPathFunctions.end() )
    {
        mPathFunctions[name] = f;
    }
    else
    {
        std::cout << "Warning : in CostSpace::addPathCost, replacing the cost function named " << name << "by another." << std::endl;
        mPathFunctions[name] = f;
    }
}

//------------------------------------------------------------------------------
void CostSpace::deleteCost(string name)
{
    std::map< string, boost::function<double(Configuration&)> >::iterator
            it = mFunctions.find(name);

    if(it != mFunctions.end())
    {
        mFunctions.erase (it);
    }
    else
    {
        std::cout << "Warning : in CostSpace::deleteCost, the cost function " << name << " does not exist." << std::endl;
    }
}

//------------------------------------------------------------------------------
double CostSpace::cost(Configuration& conf)
{
    if(!mSelectedCost.empty())
    {
        double cost = mSelectedCost(conf);
        return cost;
    }
    else
    {
        std::cout << "Warning : CostSpace::cost(Configuration& conf) called, but \
                     the cost function has not been set." << std::endl;
                     return(1.0);
    }
}

//------------------------------------------------------------------------------
/**
 * ComputeDeltaStepCost
 * Compute the cost of a portion of path */
double CostSpace::deltaStepCost(double cost1, double cost2, double length)
{
    const double epsilon = 0.002;
    //double alpha;
    const double kb = 0.00831;
    const double temp = 310.15;

    if ( ENV.getBool(Env::isCostSpace) )
    {
        switch (m_deltaMethod)
        {
        case cs_mechanical_work:
        {
            double cost;
            if (cost2 > cost1)
            {
                cost = length * epsilon + cost2 - cost1;
            }
            else
            {
                cost = epsilon * length;
            }
            return cost;
        }

        case cs_integral:
        case cs_visibility:
        {
            double powerOnIntegral = 1.0;
            double cost = std::pow( ( (cost1 + cost2) / 2 ), powerOnIntegral ) * length;

            // Warning, length added for dynamic shortcut (HRI)
            // cost += epsilon*length;

            return cost;
        }

        case cs_average:
            return (cost1 + cost2) / 2.;

        case cs_max:
            return std::max( cost1, cost2 );

        case cs_boltzman_cost:

            if (cost2 > cost1)
                return 1;
            return 1/exp(ENV.getInt(Env::maxCostOptimFailures)*(cost2-cost1)/(kb*temp));

        default:
            std::cout << "Warning: " << __PRETTY_FUNCTION__ <<  std::endl;
        }
    }
    //no cost function
    return length;
}
//----------------------------------------------------------------------
void CostSpace::setNodeCost( Node* node, Node* parent )
{
    //------------------------------------------------------
    // New node cost
    //------------------------------------------------------
    if ( parent != NULL )
    {
        confPtr_t q_tmp = parent->getConfiguration()->copy();

        // Compute the sum of cost of the node
        // TODO fix task space bug (should remove the copy)
        //    if( set_sum_cost )
        //    {
        //      LocalPath path( parent->getConfiguration()->copy(), node->getConfiguration() );
        //      node->sumCost() = parent->sumCost() + path.cost();
        //    }

        // Min and max cost of the compco
        node->getConnectedComponent()->getCompcoStruct()->minCost
                = std::min(node->getConfiguration()->cost(),
                           node->getConnectedComponent()->getCompcoStruct()->minCost );

        node->getConnectedComponent()->getCompcoStruct()->maxCost
                = std::max(node->getConfiguration()->cost(),
                           node->getConnectedComponent()->getCompcoStruct()->maxCost );

        if ( !parent->getConfiguration()->equal(*q_tmp) )
        {
            cout << "Configuration was modified" << endl;
        }
    }
}

/*if (p3d_GetCostMethodChoice() == URMSON_TRANSITION)
 {
 node->getConnectedComponent()->getCompcoStruct()->maxUrmsonCost
 = MAX(NodePt->sumCost +
 p3d_ComputeUrmsonCostToGoal(_Graph->getGraphStruct(),NodePt) ,
 node->getConnectedComponent()->getCompcoStruct()->maxUrmsonCost);
 }	*/

//jnt->getName() : right-Arm1(6) , index_dof : 16
//jnt->getName() : right-Arm2(7) , index_dof : 17
//jnt->getName() : right-Arm3(8) , index_dof : 18
//jnt->getName() : right-Arm4(9) , index_dof : 19
//jnt->getName() : right-Arm5(10) , index_dof : 20
//jnt->getName() : right-Arm6(11) , index_dof : 21
//jnt->getName() : right-Arm7(12) , index_dof : 22

//jnt->getName() : virtual_object_right(32) , index_dof : 37
//jnt->getName() : virtual_object_right(32) , index_dof : 38
//jnt->getName() : virtual_object_right(32) , index_dof : 39
//jnt->getName() : virtual_object_right(32) , index_dof : 40
//jnt->getName() : virtual_object_right(32) , index_dof : 41
//jnt->getName() : virtual_object_right(32) , index_dof : 42

void CostSpace::getPr2ArmConfiguration( Eigen::VectorXd& x, confPtr_t q )
{ 
    x[0] = (*q)[16];
    x[1] = (*q)[17];
    x[2] = (*q)[18];
    x[3] = (*q)[19];
    x[4] = angle_limit_PI((*q)[20]);
    x[5] = (*q)[21];
    x[6] = angle_limit_PI((*q)[22]);

    x[7] = (*q)[37];
    x[8] = (*q)[38];
    x[9] = (*q)[39];
    x[10] = angle_limit_PI((*q)[40]);
    x[11] = angle_limit_PI((*q)[41]);
    x[12] = angle_limit_PI((*q)[42]);
}

double CostSpace::getPr2ArmDistance( Robot* robot, Eigen::VectorXd& q_i, Eigen::VectorXd& q_f )
{
    p3d_jnt** joints = static_cast<p3d_rob*>(robot->getP3dRobotStruct())->joints;
    const int joints_id[] = { 6,7,8,9,10,11,12,32 };
    double dist=0.0,ljnt=0.0;

    int k=0;

    printf("\n");
    for (int i=0; i<8; i++)
    {
        p3d_jnt* jntPt = joints[joints_id[i]];

        cout << "jntPt->name : " << jntPt->name << " , " << jntPt->dist << endl;

        for (int j=0; j<jntPt->dof_equiv_nbr; j++)
        {
            if (p3d_jnt_is_dof_angular(jntPt, j))
            {
                if (p3d_jnt_is_dof_circular(jntPt, j))
                    dist = fabs(jntPt->dist*dist_circle(q_i[k], q_f[k]));
                else
                    dist = fabs(jntPt->dist*(q_f[k] - q_i[k]));
            }
            else{
                dist = fabs(q_f[k] - q_i[k]);
            }

            if (std::isnan(dist)) {
                printf("Distance computation error !!!\n");
                return P3D_HUGE;
            }

            dist = SQR(dist);

            printf(" dist[%d] = %f\n", jntPt->index_dof + j, dist );

            ljnt += dist;
            k++;
        }
    }
    return std::sqrt(ljnt);
}

//----------------------------------------------------------------------
double CostSpace::path_cost_default( LocalPath& path, int& nb_test )
{
    confPtr_t q_tmp_begin = path.getBegin()->copy();
    confPtr_t q_tmp_end   = path.getEnd()->copy();

    double cost = 0.0;

    nb_test = 0;

    if (ENV.getBool(Env::isCostSpace) && mSelectedCostName != "costLength" )
    {
        int nStep;
        double deltaStep;

        //        if( m_resolution == cs_classic )
        //        {
        deltaStep = path.getResolution();
        nStep = path.getParamMax() / deltaStep;
        //        }

        // cout << "nStep : " << nStep << endl;

        confPtr_t q;

        double currentCost, prevCost, deltaCost;
        double currentParam = 0.0;

        prevCost = path.configAtParam(0.0)->cost();

        for (int i=0; i<int(nStep); i++)
        {
            q = path.configAtParam( currentParam );

            currentCost = this->cost(*q);

            deltaCost = deltaStepCost( prevCost, currentCost, deltaStep );

            if( m_deltaMethod == cs_max )
            {
                if( deltaCost > cost )
                    cost = deltaCost;
            }
            else {
                cost += deltaCost;
            }
            prevCost = currentCost;
            currentParam += deltaStep;
        }

        nb_test = nStep;
    }
    else
    {
        cost = path.getParamMax();
    }

    if( !q_tmp_begin->equal(*path.getBegin()) )
    {
        cout << "Warning => begin was modified by local path cost computation" << endl;
    }
    if( !q_tmp_end->equal(*path.getEnd()) )
    {
        cout << "Warning => end was modified by local path cost computation" << endl;
    }

    return cost;
}

double CostSpace::cost( LocalPath& path, int& nb_test )
{
    if(!mPathFunctions.empty())
    {
        // cout << "path cost name : " << mSelectedPathCostName << endl;
        double cost = mSelectedPathCost( path, nb_test );
        return cost;
    }
    else
    {
        return path_cost_default( path, nb_test );
    }
}

//! The jacobian is of m by n -> nb features by active dofs
//! column per dof
//! row per feature
Eigen::VectorXd CostSpace::getJacobian( const Configuration& q, std::vector<int> active_dofs )
{
    const double eps = 1e-3;

    const std::vector<int>& dofs = active_dofs;
    if( active_dofs.empty() )
    {
        for( int i=0; i< q.getEigenVector().size(); i++ )
            active_dofs.push_back( i );
    }

    Eigen::VectorXd q_0 = q.getEigenVector( dofs );
    Eigen::VectorXd J = Eigen::VectorXd::Zero( dofs.size() );

    Configuration q_tmp( q );
    q_tmp.getRobot()->setAndUpdate( q_tmp );
    double c_0 = cost( q_tmp );

    for( int j=0; j<q_0.size(); j++ ) // For each colomn
    {
        int dof = j;
        // int dof = active_dofs_[ j ];
        // cout << "dof : " << dof << endl;

        Eigen::VectorXd q_1 = q_0;
        q_1[dof] = q_0[ dof ] + eps;

        q_tmp.setFromEigenVector( q_1, dofs );
        q_tmp.getRobot()->setAndUpdate( q_tmp );
        double c_1 = cost( q_tmp );

        // J.col(j) = ( c_1 - c_0 ) / eps;
        J(j) = ( c_1 - c_0 ) / eps;
    }

//    cout << "J : " << endl << J.transpose() << std::scientific << endl;
    return J;
}

//! The jacobian is of m by n -> nb features by active dofs
//! column per dof
//! row per feature
Eigen::MatrixXd CostSpace::getHessian( const Configuration& q, std::vector<int> active_dofs )
{
    const double eps = 1e-3;
    const double eps_sq = (eps*eps);

    const std::vector<int>& dofs = active_dofs;
    if( active_dofs.empty() )
    {
        for( int i=0; i< q.getEigenVector().size(); i++ )
            active_dofs.push_back( i );
    }

    Eigen::VectorXd q_0 = q.getEigenVector( dofs );
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero( dofs.size(), dofs.size() );

    Configuration q_tmp( q );
    q_tmp.getRobot()->setAndUpdate( q_tmp );
    double c_0 = cost( q_tmp );


    for( int i=0; i<q_0.size(); i++ ) // For each row

        for( int j=0; j<q_0.size(); j++ ) // For each colomn
        {
            int dof_i = i;
            int dof_j = j;
            // int dof = active_dofs_[ j ];
            // cout << "dof : " << dof << endl;

            Eigen::VectorXd q_1 = q_0;
            q_1[dof_i] = q_0[ dof_i ] + eps;
            q_1[dof_j] = q_0[ dof_j ] + eps;

            Eigen::VectorXd q_2 = q_0;
            q_2[dof_i] = q_0[ dof_i ] + eps;

            Eigen::VectorXd q_3 = q_0;
            q_3[dof_j] = q_0[ dof_j ] + eps;

            q_tmp.setFromEigenVector( q_1, dofs );
            q_tmp.getRobot()->setAndUpdate( q_tmp );
            double c_1 = cost( q_tmp );

            q_tmp.setFromEigenVector( q_2, dofs );
            q_tmp.getRobot()->setAndUpdate( q_tmp );
            double c_2 = cost( q_tmp );

            q_tmp.setFromEigenVector( q_3, dofs );
            q_tmp.getRobot()->setAndUpdate( q_tmp );
            double c_3 = cost( q_tmp );

            H(i, j) = ( c_1 - c_2 - c_3 + c_0 ) / eps_sq;
        }

//    cout << "J : " << endl << J.transpose() << std::scientific << endl;
    return H;
}

//----------------------------------------------------------------------
// Basic cost functions
//----------------------------------------------------------------------

double Move3D::computeFlatCost(Configuration& conf)
{
    return 1.0;
}

extern void* GroundCostObj;

double Move3D::computeIntersectionWithGround(Configuration& conf)
{
    double cost(0);
    if(GroundCostObj)
    {
        GHintersectionVerticalLineWithGround(GroundCostObj,
                                             conf.getConfigStruct()[6],
                                             conf.getConfigStruct()[7],
                                             &cost);
    }
    //cout << "Ground Cost = " << cost << endl;
    return(cost);
}

double Move3D::computeDistanceToObstacles(Configuration& conf)
{
    if( conf.isInCollision() )
    {
        return 10000;
    }
    Robot* robot = conf.getRobot();
    robot->setAndUpdate(conf);
    double cost = p3d_GetMinDistCost( static_cast<p3d_rob*>(robot->getP3dRobotStruct()) );
    return cost;
}

double Move3D::computeInCollisionCost(Configuration& conf)
{
    Robot* robot = conf.getRobot();
    confPtr_t qActual = robot->getCurrentPos();

    double cost = 0.1;

    if( conf.isInCollision() )
    {
        cost = 20.0;
    }

    robot->setAndUpdate(*qActual);
    return cost;
}

double Move3D::computeCollisionSpaceCost(Configuration& conf)
{
    double cost = 0.1;
    if( global_optimizer.get() != NULL ) {
        cost = PlanEnv->getDouble(PlanParam::trajOptimObstacWeight)*global_optimizer->getCollisionSpaceCost( conf );
    }
    return cost;
}

double Move3D::computeLocalpathKinematicCost(void* rob, p3d_localpath* LP)
{
    if (LP == NULL) {
        return 1;
    }

    p3d_rob* robotPt = (p3d_rob*)rob;
    Robot* currRob = global_Project->getActiveScene()->getRobotByNameContaining( robotPt->name );
    LocalPath path(currRob,LP);
    double cost = path.cost();
    cout << "Kinematic cost = " << cost << endl;
    return cost;
}

//----------------------------------------------------------------------
void CostSpace::initMotionPlanning(p3d_graph* graph, p3d_node* start, p3d_node* goal)
{
    Robot* robot = global_Project->getActiveScene()->getRobotByName( graph->rob->name );
    start->temp = ENV.getDouble(Env::initialTemperature);
    start->comp->temperature = ENV.getDouble(Env::initialTemperature);
    start->nbFailedTemp = 0;

    p3d_SetGlobalNumberOfFail(0);

    Configuration qStart( robot, start->q );
    p3d_SetNodeCost( graph, start, qStart.cost() );
    p3d_SetCostThreshold( start->cost );

#ifdef P3D_PLANNER
    p3d_SetInitCostThreshold( start->cost );
#else
    printf("P3D_PLANNER not compiled in %s in %s",__PRETTY_FUNCTION__,__FILE__);
#endif

    if ( ENV.getBool(Env::expandToGoal) && (goal != NULL))
    {
        goal->temp					= ENV.getDouble(Env::initialTemperature);
        goal->comp->temperature	= ENV.getDouble(Env::initialTemperature);
        start->temp				= ENV.getDouble(Env::initialTemperature);
        goal->nbFailedTemp = 0;
        //    Ng->NbDown = 0;

        Configuration qGoal( robot, goal->q );
        p3d_SetNodeCost( graph, goal, qGoal.cost());
        p3d_SetCostThreshold(MAX(qStart.cost(), qGoal.cost() ));

        //        p3d_SetCostThreshold(MAX(
        //								p3d_GetNodeCost(this->getStart()->getNodeStruct()),
        //								p3d_GetNodeCost(this->getGoal()->getNodeStruct()) ));

        p3d_SetAverQsQgCost(
                    ( graph->search_start->cost
                      +graph->search_goal->cost) / 2.);
    }
    else
    {
#ifdef P3D_PLANNER
        p3d_SetCostThreshold( start->cost );
        p3d_SetInitCostThreshold(start->cost );
        p3d_SetAverQsQgCost( graph->search_start->cost );
#else
        printf("P3D_PLANNER not compiled in %s in %s",__PRETTY_FUNCTION__,__FILE__);
#endif
    }
}

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
#include "HRICS_human_prediction_cost_space.hpp"

#include "API/project.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"


using namespace std;
using namespace HRICS;
using namespace Move3D;

HumanPredictionCostSpace* global_humanPredictionCostSpace = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Cost Function

double HRICS_getPredictionOccupancyCost(Configuration& q)
{
    //    return 1.0;
    //    return global_humanPredictionCostSpace->getCostFromActiveJoints(q);
    return global_humanPredictionCostSpace->getCost(q);
    //    return global_humanPredictionCostSpace->getCurrentOccupationCost(q);
}


//----------------------------------------------------------------------
//----------------------------------------------------------------------

HumanPredictionCostSpace::HumanPredictionCostSpace( Robot* robot, WorkspaceOccupancyGrid* occup_grid ) : m_robot(robot), m_ws_occupancy(occup_grid)
{
    cout << "Create HumanPredictionCostSpace" << endl;

    m_surface_sampler = new BodySurfaceSampler( 0.50 );

    sampleRobotPoints();
    setActiveJoints();
}

HumanPredictionCostSpace::~HumanPredictionCostSpace()
{

}

double HumanPredictionCostSpace::getCurrentOccupationCost(Configuration& q) const
{
    Robot* robot = q.getRobot();
    robot->setAndUpdate(q); // TODO remove this when necessary

    double cost = 0.0;

    int nb_points = 0;

    double factor = 10.0;

    for(int i=0; i<int(m_active_joints.size()); i++)
    {
        p3d_obj* obj = m_active_joints[i]->getP3dJointStruct()->o;

//        cout << "Active joint : "  << m_active_joints[i]->getName() << endl;

        if( obj )
        {
            //cout << "compute cost for joint : " << jnt->getName() ;
            Eigen::Transform3d T = m_active_joints[i]->getMatrixPos();
            PointCloud& pc = m_surface_sampler->getPointCloud( obj );

            for( int j=0; j<int(pc.size()); j++ )
            {
                cost += (factor*m_ws_occupancy->geCurrentOccupancy( T*pc[j] ));
                nb_points++;
            }
        }
    }

//    cout << " , cost : " << cost << endl;
//    exit(0);
    return cost;
}

double HumanPredictionCostSpace::getCost(Configuration& q) const
{
    Robot* robot = q.getRobot();
    robot->setAndUpdate(q); // TODO remove this when necessary

    double cost = 0.0;

    int nb_points = 0;

    for(int i=0; i<int(m_active_joints.size()); i++)
    {
        p3d_obj* obj = m_active_joints[i]->getP3dJointStruct()->o;

        if( obj )
        {
            //cout << "compute cost for joint " << jnt->getName() << endl;
            Eigen::Transform3d T = m_active_joints[i]->getMatrixPos();
            PointCloud& pc = m_surface_sampler->getPointCloud( obj );

            for( int j=0; j<int(pc.size()); j++ )
            {
                cost += m_ws_occupancy->getOccupancyCombination( T*pc[j] );
                //cost += m_ws_occupancy->getOccupancy( T*pc[j] );
                nb_points++;
            }
        }
    }


    cout << "cost : " << cost << endl;



    //cout << "occupancy computed for " << nb_points << endl;
    //cout << "HumanPredictionCostSpace cost : " << cost << endl;
    return cost;
}

double HumanPredictionCostSpace::getCostFromActiveJoints(Configuration& q) const
{
    Robot* robot = q.getRobot();
    //    robot->setAndUpdate(q);

    double cost=0.0;

    for(int i=0; i<int(m_active_joints.size()); i++)
    {
        cost += 10*m_ws_occupancy->getOccupancyCombination( m_active_joints[i]->getVectorPos() );
    }

    return cost;
}

void HumanPredictionCostSpace::draw()
{
    //    if( PlanEnv->getBool(PlanParam::drawSampledPoints) )
    draw_sampled_points();
}

void HumanPredictionCostSpace::sampleRobotPoints()
{
    m_surface_sampler->sampleRobotBodiesSurface( m_robot );
}

void HumanPredictionCostSpace::draw_sampled_points()
{
    if( !m_robot->getUseLibmove3dStruct() ){
        return;
    }

    for(int i=0; i<int(m_active_joints.size()); i++)
    {
        p3d_obj* obj = m_active_joints[i]->getP3dJointStruct()->o;

        if( obj )
        {
            m_surface_sampler->getPointCloud( obj ).drawAllPoints( m_active_joints[i]->getMatrixPos() );
        }
    }
}

void HumanPredictionCostSpace::setActiveJoints()
{
    if( m_robot->getName() == "PR2_ROBOT" )
    {
        // Set the planner joints
        m_active_joints.clear();
        // right arm
        //        m_active_joints.push_back( 6 );
        //        m_active_joints.push_back( 7 );

        m_active_joints.push_back( m_robot->getJoint( 8 ) );
        m_active_joints.push_back( m_robot->getJoint( 9 ) );
        m_active_joints.push_back( m_robot->getJoint( 10 ) );
        m_active_joints.push_back( m_robot->getJoint( 11 ) );
        m_active_joints.push_back( m_robot->getJoint( 12 ) );
        //        m_active_joints.push_back( 14 );
        //        m_active_joints.push_back( 15 );
    }

    int nb_points = 0;

    for(int i=0; i<int(m_active_joints.size()); i++)
    {
        p3d_obj* obj = m_active_joints[i]->getP3dJointStruct()->o;

        if( obj )
        {
            PointCloud& pc = m_surface_sampler->getPointCloud( obj );
            nb_points += pc.size();
        }
    }

    cout << "HumanPredictionCostSpace => add " << nb_points << " to the cost function" << endl;

    //    for( int i=0;i<int(m_active_joints.size());i++)
    //    {
    //        cout << m_robot->getJoint( m_active_joints[i] )->getName() << endl;
    //    }
}


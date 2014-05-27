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
#include "cspace.hpp"
#include "localpath.hpp"
//#include "riehl_localpaths.hpp" // PathSegments
//#include "mechanical_cost_functions.hpp" // 2d costmap cost function
//#include "mltrrt_cost_based_selection.hpp" // clearance cost
#include "cost_space.hpp"
#include <cmath>

#include <boost/math/special_functions/factorials.hpp>

#include <libmove3d/include/P3d-pkg.h>

using namespace Move3D;

//double CSpace::traj_cost(p3d_traj* traj)
//{
//    double cost(0.);
//    p3d_localpath *lp;
//    if (traj)
//    {
//        lp = traj->courbePt;
//        while (lp)
//        {
//            LocalPath path(m_robot, lp);
//            cost += this->lp_cost(path.getBegin(), path.getEnd());
//            lp = lp->next_lp;
//        }
//    }
//    return(cost);
//}

CSpaceCostMap2D::CSpaceCostMap2D()
{
    m_connection_radius_flag = true;
}

CSpaceCostMap2D::~CSpaceCostMap2D()
{
}

double CSpaceCostMap2D::q_cost(confPtr_t q)
{
    return(computeIntersectionWithGround(*q));
}

double CSpaceCostMap2D::lp_cost(confPtr_t q1, confPtr_t q2)
{
    LocalPath p(q1, q2);
    //    PathSegments segments(p.getParamMax(), m_cost_step);
    //    double cost(0.);
    //    double cost_q_prev(this->q_cost(q1));
    //    //printf("path length, step : %f, %f\n", p.getParamMax(), m_cost_step);
    //    for(unsigned i(1); i < segments.nbPoints(); ++i)
    //    {
    //        double cost_q_next(this->q_cost(p.configAtParam(segments[i])));
    //        //cost += std::max(cost_q_prev, cost_q_next) * segments.step();
    //        cost += std::max(cost_q_next - cost_q_prev, 0.0) + segments.step() * 0.01;
    //        //printf("cost: %f\n", cost);
    //        cost_q_prev = cost_q_next;
    //    }
    int nb_test=0;
    return global_costSpace->cost(p,nb_test);
}

double CSpaceCostMap2D::volume()
{
    assert( m_robot );
    // this means that there is 1 joint + the unused(legacy) joint 0
    assert( m_robot->getP3dRobotStruct()->njoints == 1);
    assert( m_robot->getP3dRobotStruct()->joints[1]->type == P3D_PLAN );

    double vmax_0, vmin_0, vmax_1, vmin_1;

    m_robot->getJoint( 1 )->getDofBounds( 0, vmin_0, vmax_0 );
    m_robot->getJoint( 1 )->getDofBounds( 1, vmin_1, vmax_1 );

    return( fabs( ( vmax_0 - vmin_0 ) * (vmax_1 - vmin_1 ) ) );

    //    return(fabs((m_c_robot->joints[1]->dof_data[0].vmax - m_c_robot->joints[1]->dof_data[0].vmin) *
    //                (m_c_robot->joints[1]->dof_data[1].vmax - m_c_robot->joints[1]->dof_data[1].vmin)));
}

double CSpaceCostMap2D::unit_sphere()
{
    return(M_PI);
}

unsigned CSpaceCostMap2D::dimension()
{
    return(2);
}

//-----------------------------------------------------------
//-----------------------------------------------------------
ArmCSpace::ArmCSpace( const std::vector<Joint*>& joints ) : joints_(joints)
{
    m_connection_radius_flag = true;
}

ArmCSpace::~ArmCSpace()
{
}

double ArmCSpace::q_cost(confPtr_t q)
{
    return(global_costSpace->cost(*q));
}

double ArmCSpace::lp_cost(confPtr_t q1, confPtr_t q2)
{
    LocalPath p(q1, q2);
    int nb_test;
    return global_costSpace->cost( p, nb_test );
}

double ArmCSpace::volume()
{
    // PR2
    //  jnt->getName() : right-Arm1(6) , index_dof : 16
    //  jnt->getName() : right-Arm2(7) , index_dof : 17
    //  jnt->getName() : right-Arm3(8) , index_dof : 18
    //  jnt->getName() : right-Arm4(9) , index_dof : 19
    //  jnt->getName() : right-Arm5(10) , index_dof : 20
    //  jnt->getName() : right-Arm6(11) , index_dof : 21
    //  jnt->getName() : right-Arm7(12) , index_dof : 22

    double volume=1.0;
    double vmin,vmax;

    // for( int i=6; i<=12; i++) // PR2
    for( size_t i=0; i<joints_.size(); i++ )
    {
        Joint* jntPt = joints_[i];
        jntPt->getDofBounds( 0, vmin, vmax );
        volume *= jntPt->getDist() * fabs( vmax - vmin );
    }

    return volume;
}

double ArmCSpace::unit_sphere()
{
    double n = joints_.size();
    return pow( 2, (n+1) / 2 )* pow( M_PI, (n-1) / 2 ) / boost::math::double_factorial<double>(n);
}

unsigned ArmCSpace::dimension()
{
    return( joints_.size() );
}

//-------------------------------------------------------------
//-------------------------------------------------------------
GenericCSpace::GenericCSpace(path_cost_mode mode) :
    m_mode(mode)
{
    m_connection_radius_flag = false;
}

GenericCSpace::~GenericCSpace()
{
}

double GenericCSpace::q_cost(confPtr_t q)
{
    return( global_costSpace->cost(*q) );
}

double GenericCSpace::lp_cost(confPtr_t q1, confPtr_t q2)
{
    LocalPath p(q1, q2);
    //    PathSegments segments(p.getParamMax(), m_cost_step);
    //    double cost(0.);
    //    double cost_q_prev(this->q_cost(q1));
    //    switch(m_mode)
    //    {
    //    case integral:
    //    {
    //        for(unsigned i(1); i < segments.nbPoints(); ++i)
    //        {
    //            double cost_q_next(this->q_cost(p.configAtParam(segments[i])));
    //            cost += (cost_q_next + cost_q_prev) * 0.5 * segments.step();
    //            cost_q_prev = cost_q_next;
    //        }
    //    }
    //    case work:
    //    {
    //        for(unsigned i(1); i < segments.nbPoints(); ++i)
    //        {
    //            double cost_q_next(this->q_cost(p.configAtParam(segments[i])));
    //            cost += std::max(cost_q_next - cost_q_prev, 0.0) + 0.01 * segments.step();
    //            cost_q_prev = cost_q_next;
    //        }
    //    }
    //    default: {}
    //    }
    int nb_test;
    return global_costSpace->cost(p,nb_test);
}

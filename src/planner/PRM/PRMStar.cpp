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
#include "PRMStar.hpp"

// C++ Implementation: sPRM

#include "PRMStar.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Device/robot.hpp"

#include "planner/planEnvironment.hpp"

#include <iostream>
#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/P3d-pkg.h>

using namespace Move3D;
using std::cout;
using std::endl;

MOVE3D_USING_SHARED_PTR_NAMESPACE

static bool print_lower_connect = false;

PRMStar::PRMStar( Robot* R, Graph* G) : sPRM(R,G)
{
    cout << __PRETTY_FUNCTION__ << endl;

    cspace_ = NULL;

    if( _Robot->getNumberOfJoints() == 2 && _Robot->getP3dRobotStruct()->joints[1]->type == P3D_PLAN )
    {
        cspace_ = new CSpaceCostMap2D();
        initCSpace();
        cout << "plannar CS space" << endl;
    }

    if( _Robot->getName() == "PR2_ROBOT" )
    {
        std::vector<Joint*> joints;
        for( int i=6;i<=12;i++) joints.push_back( _Robot->getJoint(i) );
        cspace_ = new ArmCSpace( joints );
        initCSpace();
    }
}

PRMStar::~PRMStar()
{

}

void PRMStar::initCSpace()
{
    cspace_->set_step( radius_ );
    cspace_->set_cost_step( radius_ );
    cspace_->set_robot( _Robot );
}

double PRMStar::computeRadius()
{
    if( cspace_ == NULL )
    {
        return radius_;
    }

    return rrgBallRadius();
}

double PRMStar::rrgBallRadius()
{
    if( cspace_->get_connection_radius_flag() )
    {
        double inv_d = 1.0 / cspace_->dimension();
        double gamma_rrg = 2 * pow(1.0 + inv_d, inv_d) * pow( cspace_->volume() / cspace_->unit_sphere(), inv_d);
        double nb_nodes = _Graph->getNumberOfNodes();

        double radius = gamma_rrg * pow((log(nb_nodes)/nb_nodes), inv_d);
        //double radius = std::min(cspace_->get_step(), radius );

        if( print_lower_connect )
            cout << "radius : " << radius << endl;

        return radius;
    }
    else
    {
        return cspace_->get_step();
    }
}

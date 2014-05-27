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
// C++ Implementation: sPRM

#include "sPRM.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

#include "planner/planEnvironment.hpp"

#include <iostream>
#include <libmove3d/p3d/env.hpp>

using namespace Move3D;
using std::cout;
using std::endl;

MOVE3D_USING_SHARED_PTR_NAMESPACE

sPRM::sPRM( Robot* R, Graph* G ) : PRM( R, G )
{
    cout << "sPRM::sPRM() with robot : " << R->getName() << endl;
    cout << "Max connecting nodes : " << ENV.getInt(Env::maxConnect) << endl;

    dist_choice_ = ENV.getInt(Env::DistConfigChoice);
    radius_ = ENV.getDouble(Env::extensionStep)*ENV.getDouble(Env::dmax);

    std::vector<double> dimensions(6);

    dimensions[0] = 1.0; // Dimensions
    dimensions[1] = 1.0;
    dimensions[2] = 1.2;

    dimensions[3] = 0.5; // Offset
    dimensions[4] = -0.3;
    dimensions[5] = 1;

    _Robot->initObjectBox( dimensions );
}

sPRM::~sPRM()
{

}

double sPRM::computeRadius()
{
    return ENV.getDouble(Env::extensionStep) * ENV.getDouble(Env::dmax);
}

void sPRM::postPocess()
{
    int K = _Graph->getNumberOfNodes(); // All nodes
    radius_ = computeRadius() * PlanEnv->getDouble( PlanParam::starRadius );

    cout << " -------------------------------- " << endl;
    cout << " nb of nodes in the graph is : " << K << endl;
    cout << " nb of edges in the graph is : " << _Graph->getNumberOfEdges() << endl;
    cout << " radius used is " << radius_ << endl;

    for( int i=0; i<int(_Graph->getNumberOfNodes()); i++)
    {
        Node* node_tmp = _Graph->getNode(i);
        confPtr_t q = node_tmp->getConfiguration();

        std::vector<Node*> near_nodes = _Graph->KNearestWeightNeighbour( q, K, radius_, dist_choice_, node_tmp );

        // cout << "Nearest neighbours : " << near_nodes.size() << endl;

        for( int j=0; j<int(near_nodes.size()); j++)
        {
            LocalPath path( near_nodes[j]->getConfiguration(), q );

            bool is_valid = true;
            if( PlanEnv->getBool(PlanParam::trajComputeCollision) )
            {
                is_valid = path.isValid();
            }

            if ( is_valid  )
            {
                // cout << "lp is valid" << endl;
                _Graph->mergeComp( near_nodes[j], node_tmp, path.getParamMax(), false );
            }
        }
    }

    cout << "Graph has got " << _Graph->getNumberOfEdges() << " edges !!!" << endl;
}

void sPRM::expandOneStep()
{
    confPtr_t q = _Robot->shoot();

    _Robot->setAndUpdate( *q );

    bool is_hand_in_box = true;
//    Eigen::Vector3d pos = _Robot->getJoint( "rPalm" )->getVectorPos();
//    is_hand_in_box = _Robot->isInObjectBox(pos);

    if( is_hand_in_box && q->setConstraintsWithSideEffect() && (!q->isInCollision())  )
    {
        _Graph->addNode( new Node( _Graph, q ) );

        m_nbConscutiveFailures = 0;
        m_nbAddedNode ++;
    }
    else {
        m_nbConscutiveFailures++;
    }
}

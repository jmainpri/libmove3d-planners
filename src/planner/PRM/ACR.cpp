//
// C++ Implementation: acr
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "ACR.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

#include "planner/planEnvironment.hpp"

#include <iostream>

#include "../p3d/env.hpp"

#include "Planner-pkg.h"

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

ACR::ACR(Robot* R, Graph* G)
    : PRM(R,G)
{
    cout << "ACR::ACR() with robot : " << R->getName() << endl;
    cout << "Max connecting nodes : " << ENV.getInt(Env::maxConnect) << endl;
}

ACR::~ACR()
{

}

void ACR::expandOneStep()
{
    confPtr_t q = _Robot->shoot();

    if ( q->setConstraintsWithSideEffect() && (!q->isInCollision()))
    {
        Node* node_new = new Node( _Graph, q );

        _Graph->addNode( node_new );
        _Graph->linkNode( node_new );

        m_nbConscutiveFailures = 0;

        int K = _Graph->getNumberOfNodes();
        //int K = m_K_Nearest;
        double radius = ENV.getDouble(Env::extensionStep)*p3d_get_env_dmax();

        vector<Node*> near_nodes = _Graph->KNearestWeightNeighbour( q, K, radius, false, ENV.getInt(Env::DistConfigChoice) );

        for (int i=0; i<int(near_nodes.size()); i++)
        {
            LocalPath path( near_nodes[i]->getConfiguration(), q );

            bool is_valid = true;
            if( PlanEnv->getBool(PlanParam::trajComputeCollision) )
            {
                is_valid = path.isValid();
            }

            if ( is_valid  )
            {
                _Graph->linkNodeAndMerge( near_nodes[i], node_new, true );
            }
        }
    }

    m_nbAddedNode ++;

    if (ENV.getBool(Env::drawExploration))
    {
        (*_draw_func)();
    }
}

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

sPRM::sPRM(Robot* R, Graph* G)
    : PRM(R,G)
{
    cout << "sPRM::sPRM() with robot : " << R->getName() << endl;
    cout << "Max connecting nodes : " << ENV.getInt(Env::maxConnect) << endl;

    dist_choice_ = ENV.getInt(Env::DistConfigChoice);
    radius_ = ENV.getDouble(Env::extensionStep)*ENV.getDouble(Env::dmax);
}

sPRM::~sPRM()
{

}

void sPRM::postPocess()
{
    int K = _Graph->getNumberOfNodes(); // All nodes

    cout << " -------------------------------- " << endl;
    cout << " nb of node in the graph is : " << K << endl;
    cout << " radius used is " << radius_ << endl;

    for( int i=0; i<int(_Graph->getNumberOfNodes()); i++)
    {
        Node* node_tmp = _Graph->getNode(i);
        confPtr_t q = node_tmp->getConfiguration();

        std::vector<Node*> near_nodes = _Graph->KNearestWeightNeighbour( q, K, radius_, dist_choice_, node_tmp );

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
                _Graph->mergeComp( near_nodes[j], node_tmp, path.getParamMax(), false );
            }
        }
    }
}

void sPRM::expandOneStep()
{
    confPtr_t q = _Robot->shoot();

    if ( q->setConstraintsWithSideEffect() && (!q->isInCollision()))
    {
        _Graph->addNode( new Node( _Graph, q ) );

        m_nbConscutiveFailures = 0;
        m_nbAddedNode ++;
    }
    else {
        m_nbConscutiveFailures++;
    }
}

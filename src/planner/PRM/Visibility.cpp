//
// C++ Implementation: vis_prm
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "Visibility.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Roadmap/compco.hpp"

#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/Planner-pkg.h>
#include <libmove3d/include/Graphic-pkg.h>

using namespace std;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

Vis_PRM::Vis_PRM(Robot* R, Graph* G) : PRM(R,G)
{
    m_nbOfExpand = 0;
    cout << " New Visibility PRM "  << endl;
}

Vis_PRM::~Vis_PRM()
{

}

/**
 * Returns vector of nodes that are orphan
 */
vector<Node*> Vis_PRM::isOrphanLinking( Node* N, int& link )
{
    vector<Node*> vect;
    double dist = 0;

    //  for (int i=0; i<int(_Graph->getNumberOfNodes()); i++) {
    //    _Graph->getNode(i)->distMultisol(N);
    //  }
    // sort(nodes.begin(), nodes.end(), &compareNodes);

    for (int j=0; j<int(_Graph->getNumberOfCompco()); j++)
    {
        vector<Node*>& nodes = _Graph->getConnectedComponent(j)->getNodes();

        for (int i=0; i<int(nodes.size()); i++)
        {
            if( N == nodes[i] ) continue;

            if (_Graph->areNodesLinked( nodes[i], N, dist ) /*&& (dist < ENV.getDouble(Env::dist) || !ENV.getBool(Env::useDist))*/)
            {
                link++;
                vect.push_back( nodes[i] );
                break;
            }
        }
    }
    return vect;
}


/**
 * Is Linking Orphan node
 */
bool Vis_PRM::linkOrphanLinking( Node* node, int type, unsigned int& total_nodes_added, int& nb_fail )
{
    int link = 0;
    int nodes_added =0;

    // All Orphan nodes
    vector<Node*> vect = isOrphanLinking( node, link );

    if ((type == 1 || type == 2) && (link > 1))
    {
        //_Graph->insertNode(N);
        _Graph->addNode(node); total_nodes_added++; nodes_added++;
        nb_fail = 0;

        for (int i=0; i<int(vect.size()); i++)
        {
            _Graph->linkNodeAndMerge( vect[i], node, ENV.getBool(Env::isCostSpace) );
        }

        return true;
    }
    else if ((type == 0 || type == 2) && (link == 0))
    {
        _Graph->addNode( node );
        total_nodes_added++; nodes_added++;
        nb_fail = 0;
        return true;
    }
    else
    {
        if( nodes_added > 0 )
        {
            throw string("Erases a created node");
        }
        nb_fail++;
        node->deleteCompco();
        delete node->getNodeStruct();
        return false;
    }
}


/**
 * Create Orphans Linking node

int Vis_PRM::createOrphansLinking(unsigned int nb_node, int type)
{
    unsigned int nodes_added = 0;
    int nb_try = 0;

    while ((*_stop_func)() &&  nb_try < ENV.getInt(Env::NbTry) && (_Graph->getGraphStruct()->ncomp > 1 || !type))
    {
        if ( !(_Graph->getNumberOfNodes() < nb_node) )
        {
            return nodes_added;
        }

        createOneOrphanLinking( type, nodes_added, nb_try );
    }
    return nodes_added;
}

 */

/**
 * Create One Orphan Linking
 */
void Vis_PRM::createOneOrphanLinking( int type, unsigned int& nodes_added, int& nb_fail )
{
    confPtr_t q = _Robot->shoot();

    if ( q->setConstraintsWithSideEffect() && !q->isInCollision() )
    {
        Node* N = new Node( _Graph, q );

        bool add_node = linkOrphanLinking( N, type, nodes_added, nb_fail );

        if( add_node && ENV.getBool(Env::drawGraph) )
        {
            g3d_draw_allwin_active();
        }
    }
}

/**
 * Expand one step
 * of the PRM process
 */
void Vis_PRM::expandOneStep()
{
    createOneOrphanLinking( 2, m_nbAddedNode, m_nbConscutiveFailures );
}

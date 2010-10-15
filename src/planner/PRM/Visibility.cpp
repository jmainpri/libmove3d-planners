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

#include "env.hpp"

using namespace std;
using namespace tr1;

#include "Planner-pkg.h"

Vis_PRM::Vis_PRM(Robot* R, Graph* G)
        : PRM(R,G)
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
vector<Node*> Vis_PRM::isOrphanLinking(Node* N, int & link)
{
    vector<Node*> vect;
    double dist = 0;
	
	p3d_list_node* NodeScan = _Graph->getGraphStruct()->nodes;
	
	while (NodeScan != NULL) {//update all nodes in the list
		NodeScan->N->dist_Nnew = p3d_APInode_dist_multisol(
									_Graph->getGraphStruct(), 
									N->getNodeStruct(), 
									NodeScan->N);//compute the distance between N[i] and the node in the list
		
//		NodeScan->N->dist_Nnew = p3d_dist_config(_Graph->getRobot()->getRobotStruct(), 
//												 N->getNodeStruct()->q,
//												 NodeScan->N->q);
		
		NodeScan = NodeScan->next;
	}
	p3d_order_node_list(_Graph->getGraphStruct()->nodes);
	_Graph->sortNodesByDist(N);
						
	//cout << "----------------------------------------------" << link << endl;
    
	for (int j=1; j<=_Graph->getGraphStruct()->ncomp; j++)
    {
        for (unsigned int i=0; i < _Graph->getNumberOfNodes() ; i++)
        {
            if (_Graph->getNode(i)->getNodeStruct()->numcomp == j)
            {
				//cout << _Graph->getNode(i)->dist(N) << endl;
                if (_Graph->getNode(i)->isLinkable(N, &dist) /*&& 
					(dist < ENV.getDouble(Env::dist) || !ENV.getBool(Env::useDist))*/)
                {
                    link++;
                    vect.push_back(_Graph->getNode(i));
                    break;
                }
            }
        }
    }
    return vect;
}


/**
 * Is Linking Orphan node
 */
bool Vis_PRM::linkOrphanLinking(Node* N, int type, unsigned int & ADDED, int & nb_fail)
{
	int link = 0;
	
	int NodesAdded =0;
	// All Orphan nodes
	vector<Node*> vect = this->isOrphanLinking(N,link);
	
	//	cout << "Size of linked nodes " << vect.size() << endl;
	//	cout << "Link = " << link << endl;
	if ((type == 1 || type == 2) && (link > 1))
	{
		//_Graph->insertNode(N);
		_Graph->addNode(N); ADDED++; NodesAdded++;
		nb_fail = 0;
		
		//_Graph->MergeComp(vect[0], N, vect[0]->dist(N));
		vect[0]->connectNodeToCompco(N, 0);
		//_Graph->addEdges(N,vect[0],N->dist(vect[0]));
		for (unsigned int k = 1; k < vect.size(); k++)
		{
			//N->merge(vect[k]);
			///_Graph->addEdges(N,,N->dist(vect[k]));
			//p3d_create_edges(_Graph->getGraphStruct(),
			//							 vect[k]->getNodeStruct(),
			//							 N->getNodeStruct(),
			//							 N->dist(vect[k]));
			
			//_Graph->MergeComp(vect[k], N, vect[k]->dist(N));
			vect[k]->connectNodeToCompco(N, 0);
			
		}
		return true;
	}
	else if ((type == 0 || type == 2) && (link == 0))
	{
		_Graph->insertNode(N);
		ADDED++; NodesAdded++;
		nb_fail = 0;
		return true;
	}
	else
	{
		if(NodesAdded > 0)
		{
			throw string("Erases a created node"); 
		}
		nb_fail++;
		N->deleteCompco();
		delete N->getNodeStruct();
		return false;
	}
}


/**
 * Create Orphans Linkin node
 */
int Vis_PRM::createOrphansLinking(unsigned int nb_node, int type)
{
	unsigned int ADDED = 0;
	int nb_try = 0;
	
	while ((*_stop_func)() &&  nb_try < ENV.getInt(Env::NbTry) && (_Graph->getGraphStruct()->ncomp > 1 || !type))
	{
		if (!(_Graph->getNumberOfNodes() < nb_node)) 
		{
			return ADDED;
		}
		
		createOneOrphanLinking(type, ADDED, nb_try);
	}
	return ADDED;
}

/**
 * Create One Orphan Linking
 */
void Vis_PRM::createOneOrphanLinking(int type, unsigned int & ADDED, int & nb_fail)
{
    shared_ptr<Configuration> q = _Robot->shoot();
	
	if ( q->setConstraintsWithSideEffect() && !q->isInCollision() ) 
	{
		Node* N = new Node(_Graph,q);
		
		if(linkOrphanLinking( N, type, ADDED, nb_fail) && ENV.getBool(Env::drawGraph) )
		{

			(*_draw_func)();

		}
	}
}

/**
 * Expand one step 
 * of the PRM process
 */
void Vis_PRM::expandOneStep()
{
	createOneOrphanLinking(2,
						   m_nbAddedNode, 
						   m_nbConscutiveFailures);
	
	//cout << _nbConscutiveFailures << endl;
}

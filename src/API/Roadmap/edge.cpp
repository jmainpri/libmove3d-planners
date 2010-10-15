//
// C++ Implementation: edge
//
// Description:
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include "Roadmap/edge.hpp"
#include "Roadmap/graph.hpp"
#include "Roadmap/BGL_Graph.hpp"

#include <tr1/memory>

#include "Localpath-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

Edge::Edge(Graph* G, unsigned int i, unsigned int j) : 
	m_is_BGL_Descriptor_Valid(false),
	m_is_LocalPath_Computed(false)
	
{
	m_Graph = G;
	
	m_Source = m_Graph->getNode(i);
  m_Target = m_Graph->getNode(j);
	
	m_Long = m_Source->dist(m_Target);
	
	Edge* tmpEdge = new Edge(m_Graph,m_Source,m_Target,m_Long);
	m_Edge = tmpEdge->m_Edge;
}

//constructor and destructor
Edge::Edge(Graph* G, p3d_edge* E) : 
	m_is_BGL_Descriptor_Valid(false),
	m_is_LocalPath_Computed(false)
{
    m_Edge = E;
    m_Graph = G;
    m_Robot = G->getRobot();
    m_Long = m_Edge->longueur;
    m_Source = m_Graph->getNode(E->Ni);
    m_Target = m_Graph->getNode(E->Nf);
}

//constructor and destructor
//Edge::Edge(cppm_Graph* G, p3d_edge* E)
//{
//    m_Edge = E;
//    //m_Graph = G;
//    m_Robot = G->getRobot();
//    m_Long = m_Edge->longueur;
//    m_Source =	G->getNode(E->Ni);
//    m_Target =		G->getNode(E->Nf);
//}

Edge::Edge(Graph* G, Node* N1, Node* N2, double Long) :
	m_is_BGL_Descriptor_Valid(false),
	m_is_LocalPath_Computed(false)
{
	int *ikSol = NULL;
	
	m_Edge = new p3d_edge;
	
	m_Edge->Ni = N1->getNodeStruct();
	m_Edge->Nf = N2->getNodeStruct();
	
//	m_Edge->num = _Graph->getNumberOfEdges();
	
	m_Edge->path = p3d_local_planner_multisol(
																						G->getRobot()->getRobotStruct(),
																						N1->getConfiguration()->getConfigStruct(),
																						N2->getConfiguration()->getConfigStruct(),
																						ikSol);
	
	m_Edge->planner = p3d_local_get_planner();
	
	//voir pour la longueur
	m_Edge->longueur = Long;
	m_Edge->sens_edge = 1;
	m_Edge->visible = 0;
	m_Edge->unvalid = 0;
	m_Edge->for_cycle = 0;
	
	
	m_Graph = G;
	m_Robot = G->getRobot();
//	p3d_SetEdgeCost(m_Robot->getRobotStruct(),m_Edge);	
//	Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
	LocalPath path(m_Robot,m_Edge->path);
	m_Edge->cost = path.cost();
	
	m_Long = Long;
	m_Source = N1;
	m_Target = N2;
	
	m_is_BGL_Descriptor_Valid = false;
}

Edge::~Edge()
{
	m_Edge->path->destroy(m_Robot->getRobotStruct(), m_Edge->path );
	delete m_Edge;
}

//Accessors
p3d_edge* Edge::getEdgeStruct()
{
    return m_Edge;
}

Graph* Edge::getGraph()
{
    return m_Graph;
}

Robot* Edge::getRobot()
{
    return m_Robot;
}

double Edge::longueur()
{
    return m_Long;
}

Node* Edge::getSource()
{
    return m_Source;
}

Node* Edge::getTarget()
{
    return m_Target;
}

double Edge::cost()
{
	return getLocalPath()->cost();
}

shared_ptr<LocalPath> Edge::getLocalPath()
{
	if( !m_is_LocalPath_Computed )
	{
		shared_ptr<LocalPath> ptrLP(new LocalPath(m_Source->getConfiguration(),
																							m_Target->getConfiguration()));
		m_path = ptrLP;
		
		m_is_LocalPath_Computed = true;
	}

	return m_path;	
}

BGL_Edge Edge::getDescriptor()
{
	if (!m_is_BGL_Descriptor_Valid) 
	{
		m_BGL_Descriptor = m_Graph->findEdgeDescriptor(this);
		
		m_is_BGL_Descriptor_Valid = true;
	}

	return m_BGL_Descriptor;
}

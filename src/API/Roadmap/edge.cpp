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

#include "Localpath-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

#undef USE_LOCALPATH_PLANNER

Edge::Edge(Graph* G, unsigned int i, unsigned int j) :
    m_is_cost_computed(false),
    m_is_BGL_Descriptor_Valid(false),
    m_is_LocalPath_Computed(false)

{
    m_Graph = G;

    m_Source = m_Graph->getNode(i);
    m_Target = m_Graph->getNode(j);

    m_Long = m_Source->dist(m_Target);

    double cost = m_Long;
    double length = 0.0;

    Edge* tmpEdge = new Edge( m_Graph, m_Source, m_Target, false, cost, true, length );

    m_Edge = tmpEdge->m_Edge;
}

//constructor and destructor
Edge::Edge(Graph* G, p3d_edge* E) : 
    m_is_cost_computed(false),
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

//! Builds an edge from two nodes
//! when cost or length are not set to be computed
//! the value given as argument are set to the localpath structure
Edge::Edge(Graph* G, Node* N1, Node* N2, bool compute_length, double& length, bool compute_cost, double& cost) : 
    m_is_cost_computed(false),
    m_Source( N1 ),
    m_Target( N2 ),
    m_Long( length ),
    m_is_BGL_Descriptor_Valid(false),
    m_is_LocalPath_Computed(false)
{  
    m_Graph = G;
    m_Robot = G->getRobot();

    m_Edge = new p3d_edge;

    m_Edge->Ni = N1->getNodeStruct();
    m_Edge->Nf = N2->getNodeStruct();

    // See how it's done for the connected componnents
    //	m_Edge->num = _Graph->getNumberOfEdges();
    m_Edge->path = NULL;

#ifdef USE_LOCALPATH_PLANNER

        int *ikSol = NULL;

        m_Edge->path = p3d_local_planner_multisol( static_cast<p3d_rob*>( G->getRobot()->getP3dRobotStruct()),
                                                   N1->getConfiguration()->getConfigStruct(),
                                                   N2->getConfiguration()->getConfigStruct(),
                                                   ikSol);

        m_Edge->planner = p3d_local_get_planner();
#endif

    // Compute the length from the associated localpath
    if( compute_length ) {
        m_Long = getLocalPath()->getParamMax();
        length = m_Long;
    }

    // Compute the cost from the associated localpath
    if( compute_cost )
        cost = getLocalPath()->cost();

    // cout << "edge cost : " << cost << endl;
    
    m_Edge->cost = cost;

    //voir pour la longueur
    m_Edge->longueur = m_Long;
    m_Edge->sens_edge = 1;
    m_Edge->visible = 0;
    m_Edge->unvalid = 0;
    m_Edge->for_cycle = 0;
}

Edge::~Edge()
{
    if( m_Edge->path ) {
        m_Edge->path->destroy( (p3d_rob*) m_Robot->getP3dRobotStruct(), m_Edge->path );
    }

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

double Edge::length()
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
//    if( !m_is_cost_computed )
        m_Edge->cost = getLocalPath()->cost();

    //  if( m_Edge->cost == 0.0 ) {
    //    cout << "cost 0" << endl;
    //  }
    m_is_cost_computed = true;
    return m_Edge->cost;
}

pathPtr_t Edge::getLocalPath()
{
    if( !m_is_LocalPath_Computed )
    {
        pathPtr_t pathPtr(new LocalPath(m_Source->getConfiguration(), m_Target->getConfiguration()));
        m_path = pathPtr;

        m_is_LocalPath_Computed = true;
    }

    return m_path;
}

void Edge::setLocalPath(pathPtr_t pathPtr)
{
    m_path = pathPtr;
    m_is_LocalPath_Computed = true;
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

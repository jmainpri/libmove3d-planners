#include "GraphState.hpp"
#include <vector>
#include "P3d-pkg.h"

using namespace std;
using namespace API;

GraphState::GraphState()
{

}


GraphState::GraphState(p3d_node* node)
{
    _GraphNode=node;
//    cout << "node " << _GraphNode << endl;
}



vector<State*> GraphState::getSuccessors()
{
   p3d_list_node* list = _GraphNode->neighb;
   std::vector<State*> nieghboorStates;

    for(int i=0;i<_GraphNode->nneighb;i++)
    {
            p3d_node* ptrNode = list->N;
            nieghboorStates.push_back(new GraphState( ptrNode ));
            list = list->next;
    }

    return nieghboorStates;
}

bool GraphState::equal(State* other)
{
    bool equal(false);
    GraphState* state = dynamic_cast<GraphState*>(other);
    equal = p3d_equal_config(XYZ_ROBOT,_GraphNode->q,state->_GraphNode->q);
//    cout << "equal ?" << equal <<  endl;
//    if(equal)
//    {
//        p3d_print_node(XYZ_GRAPH,_GraphNode);
//        p3d_print_node(XYZ_GRAPH,state->_GraphNode);
//    }
    return equal;
}


bool GraphState::isLeaf()
{
//    cout << "is Leaf" << endl;
    bool isLeaf(false);
    isLeaf = p3d_equal_config(XYZ_ROBOT,_GraphNode->q,XYZ_GRAPH->last_node->N->q);
//    print_config(XYZ_ROBOT,XYZ_GRAPH->last_node->N->q);
    return isLeaf;
}

void GraphState::print()
{
    print_config(XYZ_ROBOT,_GraphNode->q);
    cout << "------------------------------------------"<< endl;
}

void GraphState::setClosed(std::vector<State*>& closedStates,std::vector<State*>& openStates)
{
//    cout << "set closed" << endl;
    _GraphNode->closed = true;
}

bool GraphState::isColsed(std::vector<State*>& closedStates)
{
    return _GraphNode->closed;
}


void GraphState::setOpen(std::vector<State*>& openStates)
{
//     cout << "set opened" << endl;
    _GraphNode->opened = true;
}

bool GraphState::isOpen(std::vector<State*>& openStates)
{
    return _GraphNode->opened;
}

double GraphState::computeLength(State *parent)
{
    GraphState* preced = dynamic_cast<GraphState*>(parent);

    double dist = p3d_dist_config(XYZ_ROBOT, _GraphNode->q, preced->_GraphNode->q);

    if(!ENV.getBool(Env::isCostSpace))
    {
        return (preced->g() + dist);
    }
    else
    {
        double cost = preced->g() + p3d_ComputeDeltaStepCost(
															 _GraphNode->cost,
															 preced->_GraphNode->cost, 
															 dist);
//        cout << cost << endl;
        return cost;
    }
}


double GraphState::computeHeuristic(State *parent)
{
    double dist = p3d_dist_config(XYZ_ROBOT, _GraphNode->q, XYZ_ROBOT->ROBOT_GOTO);

    if(!ENV.getBool(Env::isCostSpace))
    {
        return dist;
    }
    else
    {
        return 0.000001*dist;
    }
}

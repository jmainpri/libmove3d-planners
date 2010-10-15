#ifndef GRAPHSTATE_HPP
#define GRAPHSTATE_HPP

#include "AStar/State.h"
#include "Planner-pkg.h"
/**
  * @ingroup CPP_API
  * @defgroup SEARCH Graph search
  * @brief Astar and Dijsktra
  */

 /**
   * @ingroup SEARCH
   * @brief Graph state interface for the AStar class
   */

class GraphState : public API::State
{
public:
    GraphState();
    GraphState(p3d_node* n);

    std::vector<State*> getSuccessors();

    p3d_node* getGraphNode() { return _GraphNode; }

    bool equal(State* other);
    bool isLeaf();
    void print();

    void setClosed(std::vector<State*>& closedStates,std::vector<State*>& openStates);
    bool isColsed(std::vector<State*>& closedStates);

    void setOpen(std::vector<State*>& openStates);
    bool isOpen(std::vector<State*>& openStates);

protected:
    double computeLength(API::State *parent);       /* g */
    double computeHeuristic(API::State *parent);    /* h */

private:
    p3d_node* _GraphNode;
};

#endif // GRAPHSTATE_HPP

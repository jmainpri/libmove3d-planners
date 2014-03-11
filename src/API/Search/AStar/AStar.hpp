/*
 * A_Star.h
 *
 * This header file contains declerations of four classes
 *
 * 1) Tree_Element
 * 2) Queue_Element
 * 3) Prioritize_Queue_Elements
 * 4) A_Star
 *
 * */

#ifndef A_STAR_H
#define A_STAR_H

#include "API/Search/AStar/State.hpp"

#include <queue>
#include <vector>

namespace Move3D
{
/**
   * @ingroup SEARCH
   * This class is the node class
   * to implement the search tree
   */
class TreeNode
{
public:
    TreeNode() : _Parent(NULL) {}
    TreeNode(Move3D::State*, TreeNode*);
    ~TreeNode() {}
    
    TreeNode* getParent() const { return _Parent;}
    Move3D::State* getState() const { return _State;}
    
private:
    TreeNode *_Parent;
    Move3D::State *_State;
};


/**
   * @ingroup SEARCH
   * Basic block to be used in
   * the priority queue.
   */
class QueueElement
{
public:
    QueueElement() : _Node(NULL) {}
    QueueElement(TreeNode* te) : _Node(te) {}
    ~QueueElement() {}
    
    TreeNode* getTreeNode() { return _Node; }
    
    friend class PrioritizeQueueElements;
    
private:
    TreeNode* _Node;
};

/**
   * @ingroup SEARCH
   * Function used for sorting tree nodes
   * in the priority queue
   */
class PrioritizeQueueElements
{
public:
    int operator()(QueueElement &x, QueueElement &y)
    {
        return x.getTreeNode()->getState()->f() > y.getTreeNode()->getState()->f();
    }
};


/**
   * @ingroup SEARCH
   * @brief This class keeps a pointer to the A-star search tree, an instant
   *  of priority_queue of "Queue_Element"s. Solve returns a vector of
   *  states
   */
class AStar
{
public:
    AStar() :
        _Goal(NULL),
        _GoalIsDefined(false)
    {}
    
    AStar(Move3D::State* goal) :
        _Goal(goal),
        _GoalIsDefined(true)
    {}
    
    ~AStar() {}
    
    std::vector<Move3D::State*>  solve(Move3D::State* initial_state);
    
private:
    Move3D::State* _Goal;
    void setGoal(Move3D::State* goal) { _Goal = goal; }
    
    bool _GoalIsDefined;
    bool isGoal(Move3D::State* state);
    
    std::vector<Move3D::State*>  getSolution(QueueElement qEl);
    
    void cleanStates();
    
    TreeNode *_Root;                        /* root of the A-star tree */
    TreeNode *_SolutionLeaf;                /* keeps the solution leaf after solve is called */
    
    std::priority_queue <QueueElement, std::vector<QueueElement>, PrioritizeQueueElements> _OpenSet;
    
    std::vector<Move3D::State*> _Solution;           /* This array is allocated when solve is called */
    std::vector<Move3D::State*> _Explored;
    
    enum {NOT_FOUND,FOUND} _AStarState;     /* keeps if a solution exists after solve is called */
};
}
#endif


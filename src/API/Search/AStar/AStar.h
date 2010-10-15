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

#include "State.h"
#include <queue>
#include <vector>

namespace API
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
        TreeNode(API::State*, TreeNode*);
        ~TreeNode() {}

        TreeNode* getParent() const { return _Parent;}
        API::State* getState() const { return _State;}

    private:
        TreeNode *_Parent;
        API::State *_State;
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

        AStar(API::State* goal) :
                _Goal(goal),
                _GoalIsDefined(true)
        {}

        ~AStar() {}

        std::vector<API::State*>  solve(API::State* initial_state);

    private:
        API::State* _Goal;
        void setGoal(API::State* goal) { _Goal = goal; }

        bool _GoalIsDefined;
        bool isGoal(API::State* state);

        std::vector<API::State*>  getSolution(QueueElement qEl);

        void cleanStates();

        TreeNode *_Root;                        /* root of the A-star tree */
        TreeNode *_SolutionLeaf;                /* keeps the solution leaf after solve is called */

        std::priority_queue <QueueElement, std::vector<QueueElement>, PrioritizeQueueElements> _OpenSet;

        std::vector<API::State*> _Solution;           /* This array is allocated when solve is called */
        std::vector<API::State*> _Explored;

        enum {NOT_FOUND,FOUND} _AStarState;     /* keeps if a solution exists after solve is called */
    };
}
#endif


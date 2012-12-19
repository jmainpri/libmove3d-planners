#include <iostream>
#include "State.hpp"
#include "AStar.hpp"

#include "Util-pkg.h"

using namespace std;
using namespace API;

/**
 * Tree Element Class
 */
TreeNode::TreeNode(State *st, TreeNode *prnt=NULL)
{
  if(st!=NULL)
  {
    _State = st;
    _Parent = prnt;
  }
  else
  {
    cerr << "Error in TreeNode::TreeNode(State *st, TreeNode *prnt=NULL)\n";
    exit(1);
  }
}



/**
 * A Star
 * backwards solution
 */
vector<State*> AStar::getSolution(QueueElement q_tmp)
{
  _AStarState = FOUND;
  TreeNode *solution_leaf = q_tmp.getTreeNode();
  TreeNode *t_tmp = solution_leaf;
  
  while(t_tmp)
  {
    _Solution.push_back(t_tmp->getState());
    t_tmp = t_tmp->getParent();
  }
  
  return _Solution;
}

/**
 *
 */
bool AStar::isGoal(State* state)
{
  //    cout << "AStar:: is Goal"<< endl;
  if(_GoalIsDefined)
  {
    return _Goal->equal(state);
  }
  else
  {
    return(false);
  }
}

void AStar::cleanStates()
{
  for(unsigned i=0;i<_Explored.size();i++)
  {
    _Explored[i]->reset();
  }
}

/**
 * A Star
 * solving function (main)
 */
vector<State*> AStar::solve(State* initialState)
{
  double tu, ts;
  cout << "start solve" << endl;
  ChronoOn();
  
  //    _Explored.reserve(20000);
  
  _AStarState = NOT_FOUND;
  _SolutionLeaf = NULL;
  
  // initialState->computeCost(NULL);
  _Root = new TreeNode(initialState);
  
  vector<State*> closedSet;
  vector<State*> openSet;
  openSet.push_back(initialState);
  
  _OpenSet.push(* new QueueElement(_Root));
  _Explored.push_back(initialState);
  
  QueueElement q_tmp;
  
  while( !_OpenSet.empty() )
  {
    q_tmp = _OpenSet.top();
    _OpenSet.pop();
    
    State* currentState = q_tmp.getTreeNode()->getState();
    // cout << "State = "<< currentState << endl;
    currentState->setClosed(openSet,closedSet);
    
    /* The solution is found */
    if(currentState->isLeaf() || this->isGoal(currentState) )
    {
      cleanStates();
      ChronoPrint("");
      ChronoTimes(&tu, &ts);
      ChronoOff();
      cout << "Number of explored states = " << _Explored.size() << endl;
      return getSolution(q_tmp);
    }

    TreeNode* parent = q_tmp.getTreeNode()->getParent();
    State* parent_state = NULL;
    if(parent != NULL) {
        parent_state = parent->getState();
    }
    vector<State*> branchedStates = currentState->getSuccessors(parent_state);
    
    for(unsigned int i=0; i<branchedStates.size(); i++)
    {
      if((branchedStates[i] != NULL) && (branchedStates[i]->isValid()))
      {
        if(!((parent != NULL) && (parent->getState()->isValid()) && (parent->getState()->equal(branchedStates[i]))))
        {
          if(!(branchedStates[i]->isColsed(closedSet)))
          {
            if(!(branchedStates[i]->isOpen(openSet)))
            {
              branchedStates[i]->computeCost(currentState,_Goal);
              branchedStates[i]->setOpen(openSet);
              
              _Explored.push_back(branchedStates[i]);
              _OpenSet.push(*new QueueElement(new TreeNode(branchedStates[i],(q_tmp.getTreeNode()))));
            }
          }
        }
      }
    }
  }
  
  cleanStates();
  ChronoPrint("");
  ChronoTimes(&tu, &ts);
  ChronoOff();
  
  if(NOT_FOUND==_AStarState)
  {
    cerr << "The Solution does not exist\n";
    return _Solution;
  }
  
  return _Solution;
}

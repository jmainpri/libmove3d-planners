/*
 * TreePlanner.hpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#ifndef TREEPLANNER_HPP_
#define TREEPLANNER_HPP_

#include "planner.hpp"

/**
  @ingroup NEW_CPP_MODULE
  @defgroup Diffusion
  @brief Tree planner module
  \image html RRT_graph2.png
  */

/**
  @ingroup Diffusion
  */
class TreePlanner : public Planner {

public:
	/**
	 * Constructor
	 */
	TreePlanner(Robot* R, Graph* G);

	/**
	 * Destructor
	 */
	~TreePlanner();

	/**
	 * Initializes Planner
	 */
	virtual int init();

	/**
	 * Checks out the Stop condition
	 */
	virtual bool checkStopConditions();

	/**
	 * Checks out the Pre-conditions
	 * - start and goal are not in collision
	 * - start and goal are different configurations
	 */
	virtual bool preConditions();

	/**
	 * Tries to connect a node to the other
	 * connected component of the graph
	 *
	 * @param currentNode The node that will be connected
	 * @param ComNode The Connected Component
	 */
	virtual bool connectNodeToCompco(Node* N, Node* CompNode);

	/**
	 * Main function to connect to the other Connected Component
	 */
	virtual bool connectionToTheOtherCompco(Node* toNode);

	/**
	 * Expands tree from component fromComp
	 * to component toComp
	 * @param fromComp the starting connex component
	 * @param toComp the arriving connex component
	 * @return the number of node created
	 */
	virtual int expandOneStep(Node* fromComp, Node* toComp) = 0 ;

	/**
	 * Main function of the Tree process
	 * @return the number of Nodes added to the Graph
	 */
	virtual unsigned int run();

	/**
	 * Returns number of consecutive failure
	 * during plannification
	 */
	unsigned int getNumberOfConsecutiveFail()
	{
		return m_nbConscutiveFailures;
	};

	/**
	 * Returns number of expansion
	 * during plannification
	 */
	unsigned int getNumberOfExpansion()
	{
		return m_nbExpansion;
	};
	
	/**
	 * Returns number of expansion failure
	 * during plannification
	 */
	unsigned int getNumberOfFailedExpansion()
	{
		return m_nbFailedExpansion;
	};
	
	/**
	 * Returns number the initial number of nodes
	 * of plannification
	 */
	unsigned int getNumberOfInitialNodes()
	{
		return m_nbInitNodes;
	};

protected:

	unsigned int m_nbConscutiveFailures;
	unsigned int m_nbExpansion;
	unsigned int m_nbFailedExpansion;
	unsigned int m_nbInitNodes;

};

#endif /* TREEPLANNER_HPP_ */

/*
 * Transition-RRT.hpp
 *
 *  Created on: Nov 20, 2012
 *      Author: jmainpri, ddevaurs
 */

#ifndef TRANSITIONRRT_HPP_
#define TRANSITIONRRT_HPP_

#include "planner/Diffusion/RRT.hpp"

class ConnectedComponent;


/**
 * Expansion procedure of the T-RRT algorithm.
 * @ingroup Diffusion
 */
class TransitionExpansion : public RRTExpansion
{
public:
    //! Constructor.
	TransitionExpansion() : RRTExpansion() {};

	//! Constructor (with graph).
    TransitionExpansion(Graph * G) : RRTExpansion(G) {};

    //! Destructor.
    virtual ~TransitionExpansion() {};

    //! Compare the given cost to the current minimum and maximum costs, and update them if necessary.
    void updateMinMaxCost(double cost);

    //! Initialize the T-RRT expansion with the given initial and goal nodes.
    void initialize(Node * init, Node * goal);

    //! Transition test between fromConf and toConf, with temperature tuning in the tempComp component.
    bool transitionTest(confPtr_t fromConf, confPtr_t toConf, ConnectedComponent * tempComp);

    /**
     * Sample a direction for the expansion process (taking into account the goal bias, if relevant).
     * @param directionNode: contains the goal node when relevant
     * @return the configuration toward which the tree will be expanded
     */
    confPtr_t sampleExpansionDirection(Node * directionNode);

    /**
     * T-RRT Extend.
     * @param fromNode: node from which the expansion is performed
     * @param directionConf: configuration toward which the expansion is going
     * @param directionNode: node toward which the expansion is going (when relevant)
     * @return the number of created nodes
     */
    unsigned extend(Node * fromNode, confPtr_t directionConf, Node * directionNode);

    /**
     * T-RRT Connect.
     * @param fromNode: node from which the expansion is performed
     * @param directionConf: configuration toward which the expansion is going
     * @param directionNode: node toward which the expansion is going (when relevant)
     * @return the number of created nodes
     */
    unsigned connect(Node * fromNode, confPtr_t directionConf, Node * directionNode);

    /**
     * T-RRT Expand process: call the appropriate expansion method.
     * @param fromNode: node from which the expansion is performed
     * @param directionConf: configuration toward which the expansion is going
     * @param directionNode: node toward which the expansion is going (when relevant)
     * @param method: Extend or Connect
     * @return the number of created nodes
     */
    unsigned expandProcess(Node * fromNode, confPtr_t directionConf, Node * directionNode,
            Env::expansionMethod method);

// private:
protected:
    //! initial node
    Node * initNode;

    //! goal node
    Node * goalNode;

    //! cost of the highest-cost node in the tree
    double maxCost;

    //! cost of the lowest-cost node in the tree
    double minCost;
};


/**
 * The T-RRT algorithm.
 * @ingroup Diffusion
 */
class TransitionRRT: public RRT
{
public:
    //! Constructor.
    TransitionRRT(Robot * R, Graph * G) : RRT(R, G) {
        std::cout << "Construct the Transition-based RRT" << std::endl;
    }

    //! Destructor.
    virtual ~TransitionRRT() {};

    /**
     * Initialize the T-RRT.
     * @return the number of nodes added to the graph
     */
    unsigned init();

    /**
     * Perform a single expansion step of T-RRT, growing the connected component containing fromNode.
     * @return the number of created nodes
     */
    unsigned expandOneStep(Node * fromNode);

    /**
     * Try to connect a given node to a given component.
     * @return: TRUE if the connection was successful.
     */
    bool connectNodeToComp(Node * node, Node * compNode);

    /**
     * Main function of the T-RRT.
     * @return the number of nodes added to the graph
     */
    unsigned run();
};

#endif /* TRANSITIONRRT_HPP_ */

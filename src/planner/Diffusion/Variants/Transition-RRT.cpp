/*
 * Transition-RRT.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: jmainpri, ddevaurs
 */

#include "Planner-pkg.h"

#include "API/Grids/PointCloud.hpp"
#include "API/Roadmap/compco.hpp"
#include "API/Roadmap/graph.hpp"

#include "Transition-RRT.hpp"

using namespace std;
using namespace Move3D;

/**
 * ****************************************************
 *      Transition Expansion
 * ****************************************************
 */

/**
 * Compare the given cost to the current minimum and maximum costs, and update them if necessary.
 */
void TransitionExpansion::updateMinMaxCost(double cost)
{
    if (cost > maxCost)
        maxCost = cost;
    if (cost < minCost)
        minCost = cost;
}


/**
 * Initialize the T-RRT expansion with the given initial and goal nodes.
 */
void TransitionExpansion::initialize(Node * init, Node * goal)
{
    initNode = init;
    goalNode = goal;

    maxCost = initNode->cost();
    minCost = initNode->cost();

    if (goalNode)
        this->updateMinMaxCost(goalNode->cost());
}


/**
 * Transition test between fromConf and toConf, with temperature tuning in the tempComp component.
 */
bool TransitionExpansion::transitionTest(confPtr_t fromConf, confPtr_t toConf, ConnectedComponent * tempComp)
{
    // failure, when cost is higher than maximal value
    if ((ENV.getDouble(Env::costMax) > 0.) && (toConf->cost() > ENV.getDouble(Env::costMax)))
        return false;

    // success, for down-hill or flat slopes
    if (toConf->cost() <= fromConf->cost())
        return true;

    // Metropolis test
    double temperature = tempComp->getTemperature();
    if (exp((fromConf->cost() - toConf->cost()) / temperature) > 0.5)
    {
        // if success, decrease the temperature
        double costRange = max(1., maxCost - minCost);
        temperature /= pow(2, (toConf->cost() - fromConf->cost()) / (0.1 * costRange));
        tempComp->setTemperature(max(temperature, ENV.getDouble(Env::initialTemperature)));
        return true;
    }
    else
    {
        // if failure, increase the temperature
        tempComp->setTemperature(temperature * pow(2, ENV.getDouble(Env::temperatureRate)));
        return false;
    }
}


/**
 * Sample a direction for the expansion process (taking into account the goal bias, if relevant).
 * @param directionNode: contains the goal node when relevant
 * @return the configuration toward which the tree will be expanded
 */
confPtr_t TransitionExpansion::sampleExpansionDirection(Node * directionNode)
{
    // bias toward the goal configuration
    if (ENV.getBool(Env::isGoalBiased) && p3d_random(0., 1.) <= ENV.getDouble(Env::Bias))
    {
        directionNode = goalNode;
        return goalNode->getConfiguration();
    }
    // or random sampling in the entire C-Space
    else
        return m_Graph->getRobot()->shoot();
}


/**
 * T-RRT Extend.
 * @param fromNode: node from which the expansion is performed
 * @param directionConf: configuration toward which the expansion is going
 * @param directionNode: node toward which the expansion is going (when relevant)
 * @return the number of created nodes
 */
unsigned TransitionExpansion::extend(Node * fromNode, confPtr_t directionConf, Node * directionNode)
{
    int nbCreatedNodes(0);
    double extStep = this->step();
    confPtr_t fromConf = fromNode->getConfiguration();
    ConnectedComponent * fromComp = fromNode->getConnectedComponent();
    LocalPath directionPath(fromNode->getConfiguration(), directionConf);
    double dirPathLength = directionPath.getParamMax();
    double k = (dirPathLength == 0.) ? 1. : min(1., extStep / dirPathLength );  // interpolation factor

    // Refinement control:
    // (k == 1. && !directionNode) means that we are potentially about to add a new refinement node;
    // in that case, if the number of refinement nodes already present in the component is too high
    // (i.e. more than 10% of the total number of nodes), the expansion attempt is canceled
    if (ENV.getBool(Env::refinementControl) && (k == 1. && !directionNode) &&
            (fromComp->getNumberOfRefinementNodes() > 0.1 * fromComp->getNumberOfNodes()))
        return 0;

    // construct an extension path (starting from fromConf) toward directionConf
    LocalPath extensionPath(fromConf, (k == 1.) ? directionConf : directionPath.configAtParam(extStep));

    // check cost, and then collisions
    if (ENV.getBool(Env::costBeforeColl))
    {
        // perform the transition test
        if (!this->transitionTest(fromConf, extensionPath.getEnd(), initNode->getConnectedComponent()))
            return 0;

        // check whether the extension path is valid (i.e. without collision)
        if (!extensionPath.isValid())
            return 0;
    }
    // OR check collisions, and then cost
    else
    {
        // check whether the extension path is valid (i.e. without collision)
        if (!extensionPath.isValid())
            return 0;

        // perform the transition test
        // TODO DD Check whether it is more efficient to have a temperature local to each component
        // if (!this->transitionTest(fromConf, extensionPath.getEnd(), fromNode->getConnectedComponent()))
        if (!this->transitionTest(fromConf, extensionPath.getEnd(), initNode->getConnectedComponent()))
            return 0;
    }

    // if everything succeeded, add a new node to the graph
    Node * extensionNode = this->addNode(fromNode, extensionPath, k, directionNode, nbCreatedNodes);
    this->updateMinMaxCost(extensionNode->cost());

    // Refinement control:
    // if a new refinement node has been built, update the number of refinement nodes of the component
    if (ENV.getBool(Env::refinementControl) && (k == 1. && nbCreatedNodes > 0))
        fromComp->updateNumberOfRefinementNodes(nbCreatedNodes);

    return nbCreatedNodes;
}


/**
 * T-RRT Connect.
 * @param fromNode: node from which the expansion is performed
 * @param directionConf: configuration toward which the expansion is going
 * @param directionNode: node toward which the expansion is going (when relevant)
 * @return the number of created nodes
 */
unsigned TransitionExpansion::connect(Node * fromNode, confPtr_t directionConf, Node * directionNode)
{
    int nbCreatedNodes(0);
    double extStep = this->step();
    confPtr_t fromConf = fromNode->getConfiguration();
    ConnectedComponent * fromComp = fromNode->getConnectedComponent();
    LocalPath directionPath(fromNode->getConfiguration(), directionConf);
    double dirPathLength = directionPath.getParamMax();
    double k = (dirPathLength == 0.) ? 1. : min(1., extStep / dirPathLength );  // interpolation factor

    // Refinement control:
    // (k == 1. && !directionNode) means that we are potentially about to add a new refinement node;
    // in that case, if the number of refinement nodes already present in the component is too high
    // (i.e. more than 10% of the total number of nodes), the expansion attempt is canceled
    if (ENV.getBool(Env::refinementControl) && (k == 1. && !directionNode) &&
            (fromComp->getNumberOfRefinementNodes() > 0.1 * fromComp->getNumberOfNodes()))
        return 0;

    bool success(true);
    confPtr_t currentConf = fromConf;
    confPtr_t newConf;
    double prop(k);
    double oldProp(0.);

    // attempt a connection to directionConf
    while (success && prop <= 1.)
    {
        // check whether the extension path is valid (i.e. without collision)
        newConf = (prop == 1.) ? directionConf : directionPath.configAtParam(prop * dirPathLength);
        LocalPath extensionPath(currentConf, newConf);
        if (!extensionPath.isValid())
            success = false;

        // perform the transition test
        if (success && this->transitionTest(currentConf, newConf, initNode->getConnectedComponent()))
        {
            currentConf = newConf;
            oldProp = prop;
            if (prop < 1.)
                prop = min(1., prop + k);
            else
                prop = 2.;
        }
        else
            success = false;
    }

    // if "reached" or "advanced"
    if (success || prop > k)
    {
        LocalPath connectPath(fromConf, currentConf);
        Node * connectionNode = this->addNode(fromNode, connectPath, oldProp, directionNode, nbCreatedNodes);

        this->updateMinMaxCost(connectionNode->cost());

        // Refinement control:
        // if a new refinement node has been built, update the number of refinement nodes of the component
        if (ENV.getBool(Env::refinementControl) && (k == 1. && nbCreatedNodes > 0))
            fromComp->updateNumberOfRefinementNodes(nbCreatedNodes);
    }

    return nbCreatedNodes;
}


/**
 * T-RRT Expand process: call the appropriate expansion method.
 * @param fromNode: node from which the expansion is performed
 * @param directionConf: configuration toward which the expansion is going
 * @param directionNode: node toward which the expansion is going (when relevant)
 * @param method: Extend or Connect
 * @return the number of created nodes
 */
unsigned TransitionExpansion::expandProcess(Node * fromNode, confPtr_t directionConf, Node * directionNode,
                                            Env::expansionMethod method)
{
    switch (method)
    {
    case Env::Extend:
        return this->extend(fromNode, directionConf, directionNode);

    case Env::Connect:
        return this->connect(fromNode, directionConf, directionNode);

    default:
        cerr << "*** ERROR: TransitionExpansion::expandProcess --> method not implemented !!! ***" << endl;
        return 0;
    }
}


/**
 * ****************************************************
 *      Transition RRT
 * ****************************************************
 */

/**
 * Initialize the T-RRT.
 * @return the number of nodes added to the graph
 */
unsigned TransitionRRT::init()
{
    // initialize the Tree Planner
    unsigned nbAddedNodes = TreePlanner::init();

    // create the T-RRT expansion
    _expan = new TransitionExpansion(_Graph);
    ((TransitionExpansion *) _expan)->initialize(_Start, _Goal);

    // set the temperature of the start component
    _Start->getConnectedComponent()->setTemperature(ENV.getDouble(Env::initialTemperature));

    // TODO DD Check whether it is more efficient to have a temperature local to each component
    /*if (ENV.getBool(Env::expandToGoal) && _Goal)
        _Goal->getConnectedComponent()->setTemperature(ENV.getDouble(Env::initialTemperature));*/

    return nbAddedNodes;
}


/**
 * Perform a single expansion step of T-RRT, growing the connected component containing fromNode
 * @return the number of created nodes
 */
unsigned TransitionRRT::expandOneStep(Node * fromNode)
{
    // sample the expansion direction (i.e. the configuration toward which the expansion will be performed)
    // N.B.: if the expansion is performed toward a node than already exists (e.g. the goal node),
    //       this node becomes the dirNode and dirConf simply points to its configuration
    Node * dirNode(NULL);
    confPtr_t dirConf = ((TransitionExpansion *) _expan)->sampleExpansionDirection(dirNode);

    // select, from the component to be grown, the node from which the expansion will be attempted
    Node * expandNode = fromNode->getConnectedComponent()->nearestWeightNeighbour( dirConf, ENV.getInt(Env::DistConfigChoice) );

    // directed expansion
    unsigned nbCreatedNode = _expan->expandProcess(expandNode, dirConf, dirNode, ENV.getExpansionMethod());

    return nbCreatedNode;
}


/**
 * Try to connect a given node to a given component.
 * @return: TRUE if the connection was successful.
 */
bool TransitionRRT::connectNodeToComp(Node * node, Node * compNode)
{
    confPtr_t fromConf = node->getConfiguration();
    confPtr_t toConf = compNode->getConfiguration();
    LocalPath path(fromConf, toConf);
    double pathLength = path.getParamMax();
    double extensionStep = _expan->step();

    // N.B.: the connection is attempted only if the path length is less than a specified threshold
    // (which is some multiple of the extension step)
    if (pathLength <= ENV.getDouble(Env::minimalFinalExpansionGap) * extensionStep)
    {
        // check whether the path is valid (i.e. collision-free)
        // but only if collisions have to be checked before the cost condition
        if (!ENV.getBool(Env::costBeforeColl))
            if (!path.isValid())
                return false;

        // cost condition = does the connection path follow a downhill slope in the cost space?
        double k = (pathLength == 0.) ? 1. : min(1., extensionStep / pathLength);  // interpolation factor
        double proportion(k);
        confPtr_t currentConf = fromConf;
        confPtr_t newConf = (proportion == 1.) ? toConf : path.configAtParam(proportion * pathLength);

        while ((proportion <= 1.) && (newConf->cost() <= currentConf->cost()))
        {
            if (proportion < 1.) {
                proportion = min(1., proportion + k);
                currentConf = newConf;
                newConf = (proportion == 1.) ? toConf : path.configAtParam(proportion * pathLength);
            }
            else
                proportion = 2.;
        }

        // if the cost condition was satisfied
        if (proportion > 1.)
        {
            // check whether the path is valid (i.e. collision-free)
            // but only if collisions have to be checked after the cost condition
            if (ENV.getBool(Env::costBeforeColl))
                if (!path.isValid())
                    return false;

            // if everything has succeeded, perform the connection in the graph
            return _Graph->linkNodeAndMerge(node, compNode, true);
        }
        // if the cost condition was not satisfied
        else
            return false;
    }
    else
        return false;
}


/**
 * Main function of the T-RRT.
 * @return the number of nodes added to the graph
 */
unsigned TransitionRRT::run()
{
    double ts(0.);
    m_time = 0.;
    ChronoOn();

    // check the pre-conditions
    if (!this->preConditions()) {
        m_fail = true;
        cout << "Planning stopped: pre-conditions are not satisfied." << endl;
        return 0;
    }

    unsigned nbCurCreatedNodes(0);  // number of nodes created by one expansion step
    unsigned nbTotCreatedNodes(0);  // total number of nodes created so far
    Node * fromNode = _Start;       // node belonging to the component that will be expanded
    Node * toNode = _Goal;          // node belonging to the component toward which to expand (if relevant)

    // T-RRT loop: start by checking the stopping conditions
    while (!this->checkStopConditions())
    {
        // TODO DD check whether balancing is useful
        // do not expand, in the case of a balanced Bi-directional T-TRRT, when components are unbalanced
        if (!(ENV.getBool(Env::biDir) && ENV.getBool(Env::expandBalanced)
              && (fromNode->getConnectedComponent()->getNumberOfNodes()
                  > toNode->getConnectedComponent()->getNumberOfNodes() + 2)))
        {
            ++m_nbExpansion;

            // perform a single expansion step of T-RRT
            nbCurCreatedNodes = this->expandOneStep(fromNode);

            // success, if some nodes have been created
            if (nbCurCreatedNodes > 0)
            {
                nbTotCreatedNodes += nbCurCreatedNodes;
                m_nbConscutiveFailures = 0;

                // attempt a connection toward the goal / the other component (when relevant)
                if (ENV.getBool(Env::expandToGoal))
                {
                    // Bi-directional T-RRT
                    if (ENV.getBool(Env::biDir))
                    {
                        // look for the node whose configuration is the closest to that of the last node
                        // within the connected component to which toNode belongs
                        Node * closestNode = _Graph->nearestWeightNeighbour(toNode,
                                                                            _Graph->getLastNode()->getConfiguration(),
                                                                            ENV.getInt(Env::DistConfigChoice));
                        this->connectNodeToComp(_Graph->getLastNode(), closestNode);
                    }
                    // mono-directional T-RRT (with goal)
                    else
                        this->connectNodeToComp(_Graph->getLastNode(), _Goal);
                }

                // redraw the scene (when relevant)
                if (!ENV.getBool(Env::drawDisabled) && ENV.getBool(Env::drawExploration))
                    _draw_func();
            }
            // failure, otherwise
            else {
                m_nbFailedExpansion++;
                m_nbConscutiveFailures++;
            }
        }

        // in the case of the Bi-directional T-RRT, swap the roles of the connected components
        if (ENV.getBool(Env::biDir))
            swap(fromNode, toNode);

        ChronoTimes(&m_time, &ts);
    }

    ENV.setInt(Env::nbQRand, m_nbExpansion);
    ChronoOff();

    return nbTotCreatedNodes;
}

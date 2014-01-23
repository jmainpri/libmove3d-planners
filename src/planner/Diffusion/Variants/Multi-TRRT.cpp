/*
 *  Multi-TRRT.c
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 09/06/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "Multi-TRRT.hpp"

/*
 * RRT-Transition.cpp
 *
 *  Created on: Jul 27, 2009
 *      Author: jmainpri
 */

#include "planner/cost_space.hpp"
#include "planner/Diffusion/Variants/Transition-RRT.hpp"

#include "Roadmap/node.hpp"
#include "Roadmap/graph.hpp"
#include "Roadmap/compco.hpp"

#include "planEnvironment.hpp"

#ifdef HRI_COSTSPACE
#include "hri_costspace/HRICS_Workspace.hpp"
#endif

#include "Planner-pkg.h"

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

vector<confPtr_t> multi_rrt_configs;



MultiTRRT::MultiTRRT(Robot* R, Graph* G) : MultiRRT(R,G)
{
    cout << "Multi-Transition-RRT Constructor" << endl;
    m_current_traj = NULL;
}

MultiTRRT::~MultiTRRT()
{

}

class ConfigurationComparator
{	
public:

    bool operator()(shared_ptr<Configuration> first, shared_ptr<Configuration> second)
    {
        return ( first->cost() < second->cost() );
    }

} ConfigurationComparatorObject;


unsigned MultiTRRT::init()
{
    int added = TreePlanner::init();

    cout << "Graph Number of Nodes : " << _Graph->getNumberOfNodes() << endl;

    multi_rrt_configs.clear();

    //  for (int i=0; i<3; i++) {
    //
    //    confPtr_t q;
    //    while (true) {
    //      q = _Robot->shoot();
    //      if( !q->isInCollision() ){
    //        break;
    //      }
    //    }
    //    multi_rrt_configs.push_back( q  );
    //  }
    //  confPtr_t q = _Robot->getCurrentPos();
    //  (*q)[16] = -0.02981;
    //  (*q)[17] = 1.338;
    //  (*q)[18] = -1.347;
    //  (*q)[19] = -1.342;
    //  (*q)[20] = -0.5286;
    //  (*q)[21] = -0.9295;
    //  (*q)[22] = -2.888;
    //
    //  (*q)[37] = 0.4164;
    //  (*q)[38] = -0.9488;
    //  (*q)[39] = 0.6429;
    //  (*q)[40] = 2.37;
    //  (*q)[41] = -0.003307;
    //  (*q)[42] = 0.7441;
    //  multi_rrt_configs.push_back( q  );

    //  confPtr_t conf = _Robot->getCurrentPos();
    //  (*conf)[6] = -1.188;
    //  (*conf)[7] = 0.674;
    //  (*conf)[8] = 0;
    //  (*conf)[9] = -0;
    //  (*conf)[10] = 0;
    //  (*conf)[11] = 0.3789;
    //  (*conf)[12] = 0.15;
    //  (*conf)[13] = 0;
    //  (*conf)[14] = 0;
    //  (*conf)[15] = 0;
    //  (*conf)[16] = -0.02708;
    //  (*conf)[17] = -0.2262;
    //  (*conf)[18] = -0.5061;
    //  (*conf)[19] = -1.052;
    //  (*conf)[20] = -2.604;
    //  (*conf)[21] = -1.096;
    //  (*conf)[22] = -1.278;
    //  (*conf)[23] = 0;
    //  (*conf)[24] = 0;
    //  (*conf)[25] = 0;
    //  (*conf)[26] = 1.396;
    //  (*conf)[27] = 1.484;
    //  (*conf)[28] = -1.571;
    //  (*conf)[29] = 0;
    //  (*conf)[30] = 0;
    //  (*conf)[31] = 1.571;
    //  (*conf)[32] = 0;
    //  (*conf)[33] = 0;
    //  (*conf)[34] = 0;
    //  (*conf)[35] = 0;
    //  (*conf)[36] = 0;
    //  (*conf)[37] = -0.5284;
    //  (*conf)[38] = 0.9366;
    //  (*conf)[39] = 1.315;
    //  (*conf)[40] = 1.571;
    //  (*conf)[41] = 0.7716;
    //  (*conf)[42] = -2.993e-06;
    //  (*conf)[43] = -0.922;
    //  (*conf)[44] = 0.4451;
    //  (*conf)[45] = 0.5657;
    //  (*conf)[46] = 1.942;
    //  (*conf)[47] = -1.074;
    //  (*conf)[48] = 0.3455;
    //  multi_rrt_configs.push_back( conf );

    for (int i=0; i<int(multi_rrt_configs.size()); i++)
    {
        Node* newNode = new Node( _Graph, multi_rrt_configs[i] );
        _Graph->addNode(newNode );
    }

    _expan = new MultiTransitionExpansion(this->getActivGraph());
    dynamic_cast<MultiTransitionExpansion*>(_expan)->rrt = this;

    vector<Node*> nodes = _Graph->getNodes();

    for (int i=0; i<int(nodes.size()); i++)
    {
        initalizeRoot( nodes[i], i );
    }

    cout << "start MultiTRRT with " << m_Roots.size() << " trees" << endl;
    return added;
}

void MultiTRRT::initalizeRoot( Node* rootNode, int id )
{
    m_temperature.push_back( ENV.getDouble(Env::initialTemperature) );

    std::vector<Node*> nodes;
    nodes.push_back( rootNode );
    m_trees.push_back( nodes );

    global_costSpace->setNodeCost( rootNode, NULL );
}

Node* MultiTRRT::closestNeighInTree( Node* node, int id )
{
    confPtr_t q = node->getConfiguration();

    double current_dist;
    double best_score = numeric_limits<double>::max();
    Node* BestNodePt = NULL;

    for (int i=0; i<int(m_trees[id].size()); i++)
    {
        node = m_trees[id][i];

        // We take into account only the undiscarded nodes
        if (!node->getNodeStruct()->IsDiscarded)
        {
            current_dist = q->dist( *node->getConfiguration() );

            if ( current_dist < best_score)
            {
                best_score = current_dist;
                BestNodePt = node;
            }
        }
    }

    return BestNodePt;
}

bool MultiTRRT::testAddEdge(Node* source, Node* target)
{
    LocalPath path( source->getConfiguration(), target->getConfiguration() );

    if( path.isValid() )
    {
        if( path.getParamMax() <= _expan->step() )
        {
            _Graph->addEdges( source, target, false, path.getParamMax(), false, path.cost() );
            return true;
        }
    }

    double minumFinalCostGap = ENV.getDouble(Env::minimalFinalExpansionGap);

    if( ENV.getBool(Env::costExpandToGoal) && (path.getParamMax() <= (minumFinalCostGap*_expan->step())) &&
            _expan->expandToGoal( source, target->getConfiguration() ))
    {
        _Graph->addEdges( source, target, false, path.getParamMax(), false, path.cost() );
        return true;
    }
    return false;
}

bool MultiTRRT::connectNodeToCompco( Node* node, Node* compNode )
{
    bool success = false;

    for (int i=0; i<int(m_trees.size()); i++)
    {
        if( i == m_tree_id )
            continue;

        Node* cosest_node = closestNeighInTree( node, i );

        if ( node->getConnectedComponent() == cosest_node->getConnectedComponent() )
        {
            if( testAddEdge( node, cosest_node) )
            {
                success=true;
            }
            continue;
        }

        double minumFinalCostGap = ENV.getDouble(Env::minimalFinalExpansionGap);

        //	int SavedIsMaxDis = FALSE;
        int nbCreatedNodes=0;

        LocalPath path( node->getConfiguration(), cosest_node->getConfiguration() );

        if(!ENV.getBool(Env::costBeforeColl))
        {
            if( path.isValid() )
            {
                if( path.getParamMax() <= _expan->step() )
                {
                    int nbCreatedNodes=0;

                    _expan->addNode( node ,path, 1.0 ,cosest_node, nbCreatedNodes );
                    cout << "Path Valid Connected" << endl;
                    success = true;
                    continue;
                }

                if( ENV.getBool(Env::costExpandToGoal) && (path.getParamMax() <= (minumFinalCostGap*_expan->step())) &&
                        _expan->expandToGoal( node, cosest_node->getConfiguration() ))
                {
                    int nbCreatedNodes=0;

                    cout << "attempting connect " << node->getConfiguration()->cost() <<  " to " << cosest_node->getConfiguration()->cost() << endl;
                    _expan->addNode(node,path,1.0,cosest_node,nbCreatedNodes);
                    success = true;
                    continue;
                }
            }
        }
        else
        {
            if( path.getParamMax() <= _expan->step() )
            {
                //			cout << "path.getParamMax() <= _expan->step()" << endl;
                //			cout << "path.getParamMax() = " << path.getParamMax() << endl;
                //			cout << "_expan->step()     = " << _expan->step() << endl;

                if ( path.getParamMax() == 0.0 ) {
                    node->print();
                    compNode->print();
                }

                if( path.isValid() )
                {
                    _expan->addNode( node ,path, 1.0, cosest_node, nbCreatedNodes );
                    cout << "Path Valid Connected" << endl;
                    success = true;
                    continue;
                }
            }

            // Warning
            // The expansion to goal happens if only if the gap is inferior to some
            // multiple of the expansion step
            if( ENV.getBool(Env::costExpandToGoal) && (path.getParamMax() <= (minumFinalCostGap*_expan->step())) &&
                    _expan->expandToGoal(node,cosest_node->getConfiguration() ))
            {
                if( path.isValid() )
                {
                    _expan->addNode(node,path,1.0,cosest_node,nbCreatedNodes);
                    cout << "attempting connect " << node->getConfiguration()->cost() << " to " << cosest_node->getConfiguration()->cost() << endl;
                    success = true;
                    continue;
                }
            }
        }
    }
    return success;
}

void MultiTRRT::extractTrajectory()
{
    API::Trajectory* traj = _Graph->extractAStarShortestPathsTraj( _q_start, _q_goal );

    if( traj )
    {
        if( m_current_traj == NULL || (*m_current_traj) != (*traj) )
        {
            TrajectoryStatistics stat;

            double cost = traj->costStatistics(stat);

            cout << "time : " << getTime() << " , traj cost : " << cost << endl;
            //    traj->costDeltaAlongTraj();
            //    traj->replaceP3dTraj();

            //m_convergence_rate.push_back( std::make_pair( getTime(), stat )  );

            if( m_current_traj == NULL )
                m_current_traj = new API::Trajectory( _Graph->getRobot() );

            // replace current trajectory by new trajectory
            (*m_current_traj) = (*traj);

            if(  (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) )
                g3d_draw_allwin_active();
        }
    }
    else {
        cout << "no traj" << endl;
    }

    delete traj;
}

unsigned int MultiTRRT::run()
{
    shared_ptr<Configuration> tmp = _Robot->getCurrentPos();

    //	cout << "ENV.getInt(Env::maxNodeCompco) = " << ENV.getInt(Env::maxNodeCompco) << endl;
    if(!preConditions())
    {
        return 0;
    }

    int NbCurCreatedNodes = 0;
    int NbTotCreatedNodes = 0;

    if( m_trees.size() == 0 ) {
        cout << "No trees" << endl;
        return 0;
    }

    int min_size=numeric_limits<int>::max();
    for (int i=0; i<int(m_trees.size()); i++) {
        if(int(m_trees[i].size()) < min_size ) {
            min_size = int(m_trees[i].size());
        }
    }

    m_tree_id = 0;

    m_nbConscutiveFailures = 0;

    while (!checkStopConditions())
    {
        if( !ENV.getBool(Env::expandBalanced) || (int(m_trees[m_tree_id].size()) < (min_size + 2)))
        {
            dynamic_cast<MultiTransitionExpansion*>(_expan)->m_tree_id = m_tree_id;
            dynamic_cast<MultiTransitionExpansion*>(_expan)->m_temperature = m_temperature[m_tree_id];

            // cout << "expandOneStep for tree :  " << m_tree_id << endl;
            NbCurCreatedNodes = expandOneStep( m_trees[m_tree_id][0], NULL );

            if (NbCurCreatedNodes > 0)
            {
                m_trees[m_tree_id].push_back( _Graph->getLastNode() );

                if(ENV.getBool(Env::drawExploration))
                    (*_draw_func)();

                NbTotCreatedNodes += NbCurCreatedNodes;

                // cout << "NbTotCreatedNodes  = "  << NbTotCreatedNodes << endl;
                m_nbConscutiveFailures = 0;

                // If it expands towards a goal
                // Tries to link with local method
                if( connectNodeToCompco(_Graph->getLastNode(), NULL ) )
                {
                    //cout << "connected" << endl;
                    //return (NbTotCreatedNodes);

                    if( PlanEnv->getBool( PlanParam::rrtExtractShortestPath ) )
                    {
                        extractTrajectory();
                    }
                }
            }
            else
            {
                m_nbConscutiveFailures++;
            }

            m_temperature[m_tree_id] = dynamic_cast<MultiTransitionExpansion*>(_expan)->m_temperature;

            min_size=numeric_limits<int>::max();
            for (int i=0; i<int(m_trees.size()); i++) {
                if(int(m_trees[i].size()) < min_size ) {
                    min_size = int(m_trees[i].size());
                }
            }
        }

        m_tree_id++;
        if( m_tree_id >= int(m_trees.size()) )
            m_tree_id = 0;
    }

    if (ENV.getBool(Env::drawExploration))
    {
        (*_draw_func)();
    }

    _Robot->setAndUpdate(*tmp);

    return (NbTotCreatedNodes);
}


MultiTransitionExpansion::MultiTransitionExpansion()
{

}

MultiTransitionExpansion::MultiTransitionExpansion(Graph* grah) : RRTExpansion(grah)
{

}

Node* MultiTransitionExpansion::getExpansionNode( Node* compNode, confPtr_t direction, int distance)
{  
    double current_dist;
    double best_score = numeric_limits<double>::max();
    Node* BestNodePt = NULL;

    vector< vector<Node*> >& trees = rrt->getTrees();

    for (int i=0; i<int(trees[m_tree_id].size()); i++)
    {
        Node* node = trees[m_tree_id][i];

        current_dist = direction->dist( *node->getConfiguration() );

        if ( current_dist < best_score)
        {
            best_score = current_dist;
            BestNodePt = node;
        }
    }

    //  cout << " BestNodePt :  " << BestNodePt->getId() << endl;
    return BestNodePt;
}

bool MultiTransitionExpansion::expandToGoal(Node* expansionNode, confPtr_t directionConfig)
{
    bool extensionSucceeded(true);
    double param(0);
    //double temperature = expansionNode->getCompcoStruct()->temperature;
    double extensionCost(0.);

    confPtr_t fromConfig = expansionNode->getConfiguration();
    confPtr_t toConfig;

    LocalPath directionLocalPath( fromConfig, directionConfig );

    double expansionCost = fromConfig->cost();

    double paramMax = directionLocalPath.getParamMax();

    // Perform extension toward directionConfig
    // Additional nodes creation in the nExtend case, but without checking for expansion control
    for (int i = 1; param < paramMax; i++)
    {
        param = ((double) i) * step();

        if (param > paramMax)
            toConfig = directionConfig;
        else
            toConfig = directionLocalPath.configAtParam(param);

        if (ENV.getBool(Env::isCostSpace))
        {
            extensionCost = toConfig->cost();

            if (!(expansionCost >= extensionCost))
                return false;
        }
        else
            return true;

        expansionCost = extensionCost;
        fromConfig = toConfig;
    }

    return extensionSucceeded;
}

bool MultiTransitionExpansion::costTestSucceeded( Node* previousNode, double currentCost, double temperature )
{
    double ThresholdVal;
    bool success(false);

    //new simplified test for down hill slopes
    if (currentCost <= previousNode->cost() )
    {
        return true;
    }

    //Metropolis criterion (ie Boltzman probability)
    ThresholdVal = exp((previousNode->cost() - currentCost) / temperature);

    success = p3d_random(0., 1.) < ThresholdVal;

    //  if (ENV.getBool(Env::printTemp))
    //  {
    //    cout << temperature << "\t" << previousNode->cost() << "\t"<< currentCost << endl;
    //	}

    //	if (previousNode->equalCompco(m_initNode))
    //	{
    //		ENV.setDouble(Env::temperatureStart, temperature);
    //	}
    //	else
    //	{
    //		ENV.setDouble(Env::temperatureGoal, temperature);
    //	}
    return success;
}

bool MultiTransitionExpansion::transitionTest( Node& fromNode, LocalPath& extensionLocalpath )
{
    // Transition test for cost spaces, increase temperature in case of failure
    double extensionCost = extensionLocalpath.getEnd()->cost();

    if ( costTestSucceeded( &fromNode, extensionCost, m_temperature ) )
    {
        return true;
    }
    else
    {
        // cout << "Failed : Cost invalid" << endl;
        return false;
    }
}

void MultiTransitionExpansion::adjustTemperature(bool accepted, Node* node, double& temperature )
{
    if (accepted)
    {
        temperature /= 2.0;
    }
    else
    {
        double factor = exp(log(2.) / ENV.getDouble(Env::temperatureRate));
        temperature *= factor ;
    }

    //	if (node->equalCompco(m_initNode))
    //	{
    //		ENV.setDouble(Env::temperatureStart,node->getConnectedComponent()->getCompcoStruct()->temperature);
    //	}
    //	else
    //	{
    //		ENV.setDouble(Env::temperatureGoal,node->getConnectedComponent()->getCompcoStruct()->temperature);
    //	}
}

unsigned MultiTransitionExpansion::expandProcess(Node* expansionNode, MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig, Node* directionNode,
                                                 Env::expansionMethod method)
{
    //cout << "Expansion node cost = " <<  expansionNode->cost() << endl;
    //cout << "Expansion node cost = " <<  expansionNode->getNodeStruct()->cost << endl;

    bool valid(true);
    bool failed(false);
    int nbCreatedNodes(0);

    double extensionCost;

    Node* fromNode = expansionNode;
    Node* extensionNode = NULL;

    // Perform extension toward directionConfig
    // Additional nodes creation in the nExtend case, but without checking for expansion control
    LocalPath directionLocalpath(fromNode->getConfiguration(), directionConfig);


    double pathDelta = directionLocalpath.getParamMax() == 0. ? 1.
                                                              : MIN(1., step() / directionLocalpath.getParamMax() );

    LocalPath extensionLocalpath(directionLocalpath.getBegin(), pathDelta == 1.
                                 && directionNode ? directionNode->getConfiguration()
                                                  : directionLocalpath.configAtParam(pathDelta
                                                                                     * directionLocalpath.getParamMax()));

    // Expansion control
    // Discards potential nodes that are to close to the graph
    if (ENV.getBool(Env::expandControl) && !expandControl(directionLocalpath,*expansionNode))
    {
        //		cout << "Failed expandControl test in tree(" << m_tree_id << ") " << __func__ << endl;
        return 0;
    }

    extensionCost = extensionLocalpath.getEnd()->cost();

    // Transition test and collision check
    //
    if (ENV.getBool(Env::costBeforeColl))
    {
        if (ENV.getBool(Env::isCostSpace))
        {
            if (!transitionTest(*fromNode, extensionLocalpath))
            {
                failed = true;
                //				cout << "Failed transition test in tree(" << m_tree_id << ") " << __func__ << endl;
            }
        }
        if (!failed)
        {
            if (!extensionLocalpath.isValid())
            {
                valid = false;
                failed = true;
            }
        }
    }
    else
    {
        if (!extensionLocalpath.isValid())
        {
            valid = false;
            failed = true;
        }
        if (!failed)
        {
            if (ENV.getBool(Env::isCostSpace))
            {
                if (!transitionTest(*fromNode, extensionLocalpath))
                {
                    failed = true;
                }
            }
        }
    }

    // Add node to graph if everything succeeded
    if (!failed)
    {
        extensionNode = addNode(fromNode, extensionLocalpath,
                                pathDelta, directionNode,
                                nbCreatedNodes);

        if ( ( directionNode != NULL )&&( extensionNode == directionNode ))
        {
            // Components were merged
            cout << "Connected in Transition" << __func__ << endl;
            return 0;
        }

        if ( extensionCost > expansionNode->getConfiguration()->cost())
        {
            //cout << "extensionCost = " << extensionCost << " , "
            adjustTemperature( true, extensionNode, m_temperature );
        }
    }
    else
    {
        if(valid)
        {
            adjustTemperature( false, fromNode, m_temperature );
        }
        this->expansionFailed(*expansionNode);
    }

    //	if (ENV.getBool(Env::isCostSpace) && ENV.getInt(Env::costMethodChoice)
    //			== MAXIMAL_THRESHOLD)
    //	{
    //		p3d_updateCostThreshold();
    //	}

    return nbCreatedNodes;
}

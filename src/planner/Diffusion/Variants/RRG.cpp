/*
 *  RRG-RRT.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 10/03/14.
 *  Copyright 2014 WPI. All rights reserved.
 */

#include "RRG.hpp"

#include "planEnvironment.hpp"

#include "API/ConfigSpace/configuration.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Roadmap/compco.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>

#include <libmove3d/include/Planner-pkg.h>
#include <libmove3d/include/Graphic-pkg.h>

using namespace Move3D;
using std::cout;
using std::endl;

MOVE3D_USING_SHARED_PTR_NAMESPACE

const bool print_exploration = true;
const bool print_lower_connect = false;

/*!
 * Constructors with default values
 */
RRGExpansion::RRGExpansion(Graph* G) : RRTExpansion(G)
{
    cout << __PRETTY_FUNCTION__ << endl;

    m_K_Nearest = 10;
    m_RrgRadiusFactor = 1.0;
    m_step = step();

    m_compute_edge_cost = false;

//    m_nb_rewiring = 0;
    m_iteration = 0;
//    m_biasRatio = 5;
//    m_ith_on_traj = -1;
//    m_ith_trajectory = -1;

    m_cspace = NULL;

    Robot* rob = m_Graph->getRobot();

    if( rob->getNumberOfJoints() == 2 && rob->getP3dRobotStruct()->joints[1]->type == P3D_PLAN )
    {
        m_cspace = new CSpaceCostMap2D();
        initCSpace();
        cout << "plannar CS space" << endl;
    }

    if( rob->getName() == "PR2_ROBOT" )
    {
        std::vector<Joint*> joints;
        for( int i=6; i<=12; i++) joints.push_back( rob->getJoint(i) );
        m_cspace = new ArmCSpace( joints );
        initCSpace();

        std::vector<double> dimensions(6);

        dimensions[0] = 1.0; // Dimensions
        dimensions[1] = 1.0;
        dimensions[2] = 1.2;

        dimensions[3] = 0.5; // Offset
        dimensions[4] = -0.3;
        dimensions[5] = 1;

        rob->initObjectBox( dimensions );
    }

    if( rob->getName() == "kukaArm_ROBOT" )
    {
        std::vector<Joint*> joints;
        for( int i=3; i<=9; i++) joints.push_back( rob->getJoint(i) );
        m_cspace = new ArmCSpace( joints );
        initCSpace();
    }
}

/*!
 * Destructor
 */
RRGExpansion::~RRGExpansion()
{

}

void RRGExpansion::setInitAndGoal( confPtr_t q_init, confPtr_t q_goal )
{
    m_q_init = q_init;
    m_q_goal = q_goal;
}

void RRGExpansion::initCSpace()
{
    cout << "m_step is  : " << m_step << endl;

    m_cspace->set_step( m_step );
    m_cspace->set_cost_step( m_step );
    m_cspace->set_robot( m_Graph->getRobot() );
}

double RRGExpansion::rrgBallRadius()
{
    if( m_cspace == NULL )
    {
        return 5*step();
    }

    if( m_cspace->get_connection_radius_flag() )
    {
        double inv_d = 1.0 / m_cspace->dimension();
        double gamma_rrg = 2 * pow( 1.0 + inv_d, inv_d ) * pow( m_cspace->volume() / m_cspace->unit_sphere(), inv_d );
        double nb_nodes = m_Graph->getNumberOfNodes();
        double radius = gamma_rrg * pow((log(nb_nodes)/nb_nodes), inv_d);
        //double radius = std::min( m_cspace->get_step(), radius );

        if( print_lower_connect ) {
//            cout << "m_cspace->dimension() : " << m_cspace->dimension() << endl;
//            cout << "m_cspace->volume() : " << m_cspace->volume() << endl;
//            cout << "m_cspace->unit_sphere() : " << m_cspace->unit_sphere() << endl;
            cout << "radius : " << radius << endl;
        }

        return radius;
    }
    else
    {
        return( m_cspace->get_step() );
    }
}

/**
 * Compco
 */
void RRGExpansion::setInitialCompco( ConnectedComponent* compco )
{
    m_compco = compco;
}

/*!
 * Expand the localpath
 */
bool RRGExpansion::expandToGoal(Node* expansionNode,
                                MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig)
{
    return false;
}

/*!
 * Connect expansion method
 */
int RRGExpansion::connectExpandProcess( Node* expansionNode, confPtr_t directionConfig, Node* directionNode)
{
    cout << "RRGExpansion::connectExpandProcess Not implemented" << endl;
    return 0;
}

confPtr_t RRGExpansion::getExpansionDirection( Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode )
{
    confPtr_t q;

    m_iteration++;

    // the entire CSpace
    q = m_Graph->getRobot()->shoot( samplePassive );

    return q;
}


/*!
 * Extend expansion method
 *
 * The expansion node is the node from the
 * graph that has been selected by the voronoi bias
 */
int RRGExpansion::extendExpandProcess( Node* expansion_node, confPtr_t direction_config, Node* directionNode )
{
    bool failed(false);
    int nbCreatedNodes(0);

    pathPtr_t extensionLocalpath( new LocalPath( expansion_node->getConfiguration(), direction_config ) );

    double radius = rrgBallRadius() * PlanEnv->getDouble( PlanParam::starRadius );

    if( *expansion_node->getConfiguration() == *direction_config )
    {
        failed=true;
    }
    else
    {
        // Perform extension toward directionConfig
        // Construct a smaller path from direction path
        extensionLocalpath = getExtensiontPath( expansion_node->getConfiguration(), direction_config, radius );
    }

    confPtr_t q_new = extensionLocalpath->getEnd();

    // Check that endeffecor is in box
    if( m_Graph->getRobot()->getName() == "PR2_ROBOT" )
    {
        m_Graph->getRobot()->setAndUpdate( *q_new );
        failed |= ( !m_Graph->getRobot()->isInObjectBox( m_Graph->getRobot()->getJoint( "rPalm" )->getVectorPos() ) );
    }

    Node* node_new = NULL;

    bool is_valid = true;
    if( !failed && PlanEnv->getBool( PlanParam::trajComputeCollision ) )
    {
        is_valid = extensionLocalpath->isValid();
    }

    if ( !failed && is_valid )
    {
        // Create new node and add it to the graph
        node_new = new Node( m_Graph, q_new );

        m_Graph->addNode( node_new );
        node_new->merge( expansion_node, m_compute_edge_cost ); // Performs the add edge

        nbCreatedNodes++;

        int K = m_Graph->getNumberOfNodes();

        // Get connected component
        ConnectedComponent* compco = ENV.getBool(Env::biDir) ? expansion_node->getConnectedComponent() : m_compco ;

        // Get near nodes in the connected component
        std::vector<Node*> near_nodes = compco->KNearestWeightNeighbour( q_new, K, radius, ENV.getInt(Env::DistConfigChoice) );

        if( print_exploration )
        {
            cout << " -- number of neighboors : " << near_nodes.size() ;
            cout << " -- radius : " << radius ;
            cout << " -- dist : " << q_new->dist( *expansion_node->getConfiguration() );
            cout << endl;
        }

        for (int i=0; i<int(near_nodes.size()); i++)
        {
            if( near_nodes[i] == node_new )
                continue;

            LocalPath path( near_nodes[i]->getConfiguration(), q_new );

            is_valid = true;
            if( PlanEnv->getBool( PlanParam::trajComputeCollision ) )
            {
                is_valid = path.isValid();
            }

            if ( is_valid  )
            {
                double path_cost = 0.0;

                if ( PlanEnv->getBool(PlanParam::orientedGraph) )
                    m_Graph->addEdges( near_nodes[i], node_new, false, path.getParamMax(), m_compute_edge_cost, path_cost );
                else
                    m_Graph->addEdge( near_nodes[i], node_new, false, path.getParamMax(), m_compute_edge_cost, path_cost );
            }
        }

        m_last_added_node = node_new;

        // Merge compco with minNode and set as parent
//        node_new->merge( node_min );
//        node_new->sumCost() = min_sum_of_cost;
//        node_new->parent() = node_min;
//        node_min->isLeaf() = false;
    }
    else
    {
        failed = true;
    }

    // Add node to graph if everything succeeded
    if (!failed)
    {
        // Components were merged
        if(( directionNode != NULL )&&( node_new == directionNode ))
        {
            cout << "Connected in Transition" << __PRETTY_FUNCTION__ << endl;
            return 0;
        }
    }
    else
    {
        expansionFailed( *expansion_node );
    }

    return nbCreatedNodes;
}

/*!
 * expandProcess
 */
unsigned RRGExpansion::expandProcess( Node* expansionNode, confPtr_t directionConfig, Node* directionNode, Env::expansionMethod method )
{
    switch( method )
    {
    // case Env::Connect:
    //  return connectExpandProcess( expansionNode, directionConfig, directionNode );

    case Env::Connect:
    case Env::Extend:

        return extendExpandProcess( expansionNode, directionConfig, directionNode );

    default:
        std::cerr << "Error : expand process not implemented" << endl;
        return 0;
    }
}


/**
 * Constructor from a WorkSpace object
 * @param WS the WorkSpace
 */
RRG::RRG( Robot* R, Graph* G ) : RRT( R, G )
{
    cout << __PRETTY_FUNCTION__ << endl;
    m_current_traj = NULL;
}

/**
 * Destructor
 */
RRG::~RRG()
{
    saveConvergenceToFile();
    delete m_current_traj;
}

/**
 * Initialzation of the plannificator
 * @return the number of node added during the init phase
 */
unsigned RRG::init()
{
    int added = TreePlanner::init();

    _expan = new RRGExpansion(_Graph);
    dynamic_cast<RRGExpansion*>(_expan)->setInitAndGoal( _q_start, _q_goal );
    dynamic_cast<RRGExpansion*>(_expan)->setInitialCompco( _Start->getConnectedComponent() );

    return added;
}

/**
 *
 */
bool RRG::connectNodeToCompco( Node* node, Node* compNode )
{
    LocalPath path( node->getConfiguration(), compNode->getConfiguration() );

    if( PlanEnv->getBool( PlanParam::rrtExtractShortestPath ) || path.getParamMax() <= PlanEnv->getDouble(PlanParam::starFinish)*_expan->step() )
    {
        if ( path.getParamMax() == 0.0 && print_exploration )
        {
            cout << "path.getParamMax() == 0.0 in " << __PRETTY_FUNCTION__ << endl;
            node->print();
            compNode->print();
        }

        if( path.isValid() )
        {
            //_expan->addNode( node, path, 1.0, nearestNode, nbCreatedNodes );
            //m_Graph->addEdges( node, nearestNode, false, path.getParamMax(), false, path.cost() );
            _Graph->linkNodeAndMerge( node, compNode, true );

            compNode->parent() = node;
            compNode->isLeaf() = true;

            cout << "Path Valid Connected" << endl;
            return true;
        }
        else {
            return false;
        }
    }

    return false;
}

void RRG::saveConvergenceToFile()
{
    std::ostringstream oss;
    std::ofstream s;

    oss << getenv("HOME_MOVE3D") << "/statFiles/convergence_rrt_star_" << std::setfill('0') << std::setw(4) << m_runId << ".csv";

    const char *res = oss.str().c_str();

    s.open(res);
    cout << "Opening save file : " << res << endl;

    s << "TIME" << ";";
    s << "LENGTH" << ";";
    s << "MAX" << ";";
    s << "AVERAGE" << ";";
    s << "INTEGRAL" << ";";
    s << "MECHA WORK" << ";";
    s << endl;

    for (int i=0; i<int(m_convergence_rate.size()); i++)
    {
        s << m_convergence_rate[i].first << ";";
        s << m_convergence_rate[i].second.length << ";";
        s << m_convergence_rate[i].second.max << ";";
        s << m_convergence_rate[i].second.average << ";";
        s << m_convergence_rate[i].second.integral << ";";
        s << m_convergence_rate[i].second.mecha_work << ";";
        s << endl;
    }

    s.close();
    cout << "Closing save file" << endl;
}

void RRG::extractTrajectory()
{
// Move3D::Trajectory* traj = _Graph->extractBestAStarPathSoFar( _q_start, _q_goal );
    Move3D::Trajectory* traj = _Graph->extractAStarShortestPathsTraj( _q_start, _q_goal );

    if( traj )
    {
        if( m_current_traj == NULL || (*m_current_traj) != (*traj) )
        {
            TrajectoryStatistics stat;

            double cost = traj->costStatistics(stat);

            cout << "time : " << getTime() << " , traj cost : " << cost << endl;
            //    traj->costDeltaAlongTraj();
            traj->replaceP3dTraj();

            m_convergence_rate.push_back( std::make_pair( getTime(), stat )  );

            if( m_current_traj == NULL )
                m_current_traj = new Move3D::Trajectory( _Graph->getRobot() );

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

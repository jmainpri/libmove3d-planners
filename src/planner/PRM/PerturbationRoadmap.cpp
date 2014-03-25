//
//  PerturbationRoadmap.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 01/02/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#include "PerturbationRoadmap.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Roadmap/compco.hpp"

#include "trajectoryOptim.hpp"

#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/planEnvironment.hpp"

#include "p3d/env.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"

using namespace std;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

PerturbationRoadmap::PerturbationRoadmap(Robot* R, Graph* G)  : PRM(R,G)
{
    m_K_Nearest = 3;
    m_traj = NULL;
    m_sampled_on_traj = false;
    m_descent = true;
    m_max_dist_to_traj = std::numeric_limits<double>::min();
    m_use_rejection = false;

    m_std_dev_trans = 1000.0;
    m_std_dev_rot =3.14/10;

    m_radian_steps = 1;
    m_transl_steps = 1;
}

PerturbationRoadmap::~PerturbationRoadmap()
{

}

void PerturbationRoadmap::getTranslationBounds()
{
    p3d_jnt* joint = static_cast<p3d_jnt*>( _Robot->getJoint(1)->getP3dJointStruct() );

    if( joint->type == P3D_PLAN || joint->type == P3D_FREEFLYER )
    {
        Eigen::Vector2d size;
        size[0] = joint->dof_data[0].vmax - joint->dof_data[0].vmin;
        size[1] = joint->dof_data[1].vmax - joint->dof_data[1].vmin;
        m_transl_max = size.norm();
    }
    else {
        m_transl_max = 0.0;
    }

    cout << "m_transl_max : " << m_transl_max << endl;
}

unsigned PerturbationRoadmap::init()
{
    int added = PRM::init();

    cout << "Init PerturbationRoadmap" << endl;

    getTranslationBounds();

    if( m_traj ) {
        delete m_traj;
        m_traj = NULL;
    }

    //  if( !traj_optim_initScenario()) {
    //    cout << "Error!!!" << endl;
    //    return added;
    //  }

    Move3D::Trajectory T( _Robot->getCurrentTraj() );

    if( T.getNbOfPaths() == 0 ) {
        T =  Move3D::Trajectory( traj_optim_create_sraight_line_traj() );
    }

    m_qi = T.getBegin();
    m_qf = T.getEnd();
    addTrajectory( T );

    m_traj = new Move3D::CostOptimization( T );

    return added;
}

void PerturbationRoadmap::addTrajectory( const Move3D::Trajectory& T )
{
    double range_max = T.getRangeMax();
    m_delta = range_max/20;

    Node* node_1 = m_main_compco = _Start;
    Node* node_2;

    for (double s=m_delta; s<(range_max-m_delta); s+=m_delta)
    {
        node_2 = new Node( _Graph, T.configAtParam(s), true);
        _Graph->addNode( node_2 );

        if ( PlanEnv->getBool(PlanParam::orientedGraph) )
            _Graph->addEdges( node_1, node_2, false, node_1->getConfiguration()->dist(*node_2->getConfiguration()), true, 0 );
        else
            _Graph->addEdge( node_1, node_2, false, node_1->getConfiguration()->dist(*node_2->getConfiguration()), true, 0 );

        node_1 = node_2;
    }
    node_2 = new Node( _Graph, T.getEnd(), true);
    _Graph->addNode( node_2 );

    if ( PlanEnv->getBool(PlanParam::orientedGraph) )
        _Graph->addEdges( node_1, node_2, false, node_1->getConfiguration()->dist(*T.getEnd()), true, 0 );
    else
        _Graph->addEdge( node_1, node_2, false, node_1->getConfiguration()->dist(*T.getEnd()), true, 0 );

    _Graph->rebuildCompcoFromBoostGraph();
}

Node* PerturbationRoadmap::getClosestOnTraj( confPtr_t q_rand )
{
    vector<Node*> nodes = _Graph->extractAStarShortestNodePaths( m_qi, m_qf );
    if( nodes.empty() ) {
        return NULL;
    }

    Graph::sortNodesByDist( nodes, q_rand );
    return nodes[0];
}

double PerturbationRoadmap::distToTraj( confPtr_t q_new )
{
    vector<Node*> nodes = _Graph->extractAStarShortestNodePaths( m_qi, m_qf );
    if( nodes.empty() ) {
        return 0.0;
    }

    Graph::sortNodesByDist( nodes, q_new );
    return nodes[0]->getNodeStruct()->dist_Nnew;
}

//! Find the best cycle in the graph
//! the best cycle is composed of two nodes that are at a distance close to dist
//! and that maximise path length in the graph
bool PerturbationRoadmap::findBestCycle( confPtr_t q_new, double dist, std::vector<Node*>& vect_conf )
{
    const int nbNodes = 10;

    vector< pair<double,Node*> > nearNodesQueue;

    bool firstNode = true;
    nearNodesQueue.push_back( make_pair( numeric_limits<double>::max(), (Node*)NULL ) );

    for (int i=0; i<int(_Graph->getNumberOfNodes()); i++)
    {
        double current_dist = _Graph->getNode(i)->dist( q_new ) - dist;

        // If better than last node of the Queue add to the Queue (rewrite better code)
        if ( current_dist < nearNodesQueue.back().first )
        {
            if ( firstNode )
            {
                nearNodesQueue.clear(); firstNode = false;
            }
            nearNodesQueue.push_back( make_pair( current_dist, _Graph->getNode(i) ) );

            sort( nearNodesQueue.begin(), nearNodesQueue.end() );

            if( int(nearNodesQueue.size()) > nbNodes )
            {
                nearNodesQueue.resize( nbNodes );
            }
        }
    }

    vect_conf.resize(2);
    vect_conf[0] = NULL;
    vect_conf[1] = NULL;

    double longest_path = 0.0;

    // For all pair of nodes in the graph
    for (int i=0; i<int(nearNodesQueue.size()); i++)
    {
        for (int j=0; j<int(nearNodesQueue.size()); j++)
        {
            if( j <= i ) continue;

            confPtr_t q_source = nearNodesQueue[i].second->getConfiguration();
            confPtr_t q_target = nearNodesQueue[j].second->getConfiguration();

            vector<Node*> path = _Graph->extractAStarShortestNodePaths( q_source, q_target );

            for (int k=0; k<int(path.size()-1); k++)
            {
                double dist_in_graph = path[k]->dist( path[k+1] );

                if( dist_in_graph > longest_path )
                {
                    longest_path = dist_in_graph;
                    vect_conf[0] = nearNodesQueue[i].second;
                    vect_conf[1] = nearNodesQueue[j].second;
                }
            }
        }
    }

    if( vect_conf[0] == NULL || vect_conf[1] == NULL )
    {
        vect_conf.clear();
        return false;
    }

    return true;
}

bool PerturbationRoadmap::testPerturb( confPtr_t q_new, vector<Node*>& vect_nodes )
{  
    bool is_valid = true;
    if ( PlanEnv->getBool(PlanParam::trajComputeCollision) )
    {
        is_valid = !q_new->isInCollision();
    }

    if ( !is_valid ) {
        return false;
    }

    vector< pair< int, shared_ptr<LocalPath> > > edges;
    Node* node_new = NULL;

    for( int i=0;i<int(vect_nodes.size());i++) {

        shared_ptr<LocalPath> path(new LocalPath(vect_nodes[i]->getConfiguration(), q_new));

        // if the path is valid, check for cost
        if ( PlanEnv->getBool(PlanParam::trajComputeCollision) )
        {
            is_valid = path->isValid();
        }

        if ( is_valid )
        {
            edges.push_back(make_pair(i,path));

            if( edges.size() == 1 )
            {
                node_new = new Node( _Graph, q_new, true);
                _Graph->addNode( node_new );
            }
        }
    }

    for( int i=0;i<int(edges.size());i++)
    {
        double max_param = edges[i].second->getParamMax();

        if ( PlanEnv->getBool(PlanParam::orientedGraph) )
            _Graph->addEdges( vect_nodes[edges[i].first], node_new, false, max_param, true, 0 );
        else
            _Graph->addEdge( vect_nodes[edges[i].first], node_new, false, max_param, true, 0 );

        _Graph->rebuildCompcoFromBoostGraph();
        is_valid = true;
    }

    return is_valid;
}

bool PerturbationRoadmap::addPerturbation( confPtr_t q_rand )
{
    Node* node_near = _Graph->nearestWeightNeighbour( m_main_compco, q_rand, false, ENV.getInt(Env::DistConfigChoice));

    confPtr_t q_new = Move3D::CostOptimization::perturbCurrent( node_near->getConfiguration(), q_rand, m_delta, m_descent );

    vector<Node*> nodes = _Graph->KNearestWeightNeighbour( q_new, m_K_Nearest, P3D_HUGE,
                                                           false, ENV.getInt(Env::DistConfigChoice));

    if( int(nodes.size()) < m_K_Nearest )
        return false;

    vector<Node*> nodes_neigh(2);
    nodes_neigh[0] = nodes[1];
    nodes_neigh[1] = nodes[2];

    bool success = testPerturb( q_new, nodes_neigh );
    return success;
}

bool PerturbationRoadmap::expandPerturbation( confPtr_t q_rand )
{
    Node* node_near = _Graph->nearestWeightNeighbour( m_main_compco, q_rand, false, ENV.getInt(Env::DistConfigChoice));
    if (node_near == NULL) {
        return false;
    }

    LocalPath path( node_near->getConfiguration(), q_rand );

    // Dividing the step n times to stay in the bounds
    const double minStep = 2.0;
    const int	max_div = 3;

    confPtr_t q_new;

    double step = m_delta/2.0;
    bool is_q_out_of_bounds = true;
    bool is_q_in_collision = true;
    int ith_div =0;

    if( !m_sampled_on_traj )
    {
        step = path.getParamMax();
    }

    for (; ith_div<max_div && (is_q_out_of_bounds && is_q_in_collision) ; ith_div++)
    {
        q_new = path.configAtParam( step );
        is_q_out_of_bounds = q_new->isOutOfBounds();

        if ( PlanEnv->getBool(PlanParam::trajComputeCollision) )
        {
            if( !is_q_out_of_bounds ) {
                is_q_in_collision = q_new->isInCollision();
            }
        } else {
            is_q_in_collision = false;
        }
        step /= minStep;
    }

    if( is_q_out_of_bounds || is_q_in_collision )
    {
        return false;
    }

    double dist=0.0;

    if( m_use_rejection )
    {
        double dist = distToTraj(q_new);

        if( (dist > m_max_dist_to_traj) && (p3d_random(0.0,1.0) > 0.01) ) {
            //cout << "reject : " << endl;
            return false;
        }
    }

    std::vector<Node*> nodes;

    if( m_sampled_on_traj )
    {
        nodes = _Graph->KNearestWeightNeighbour( q_new, 5, P3D_HUGE,
                                                 false, ENV.getInt(Env::DistConfigChoice));
        //Graph::sortNodesByDist( nodes, q_new );
    }
    else {
        if( !findBestCycle( q_new, node_near->dist(q_new), nodes ) ) {
            cout << "Best cycle NOT found" << endl;
            return false;
        }
        //    else {
        //      cout << "Best cycle IS found" << endl;
        //    }
    }

    // Main function that tests new cycles
    bool success = testPerturb( q_new, nodes );

    if( m_use_rejection )
    {
        if ( success && (dist > m_max_dist_to_traj)) {
            m_max_dist_to_traj = dist;
        }
    }
    return success;
}

confPtr_t PerturbationRoadmap::trajShootConfiguration()
{
    double dist=0.0;

    confPtr_t q1( m_traj->getRandConfAlongTraj( dist, false ));
    confPtr_t q2( new Configuration(*q1));

    p3d_gaussian_config2_specific(q1->getRobot()->getP3dRobotStruct(), q1->getConfigStruct(), q2->getConfigStruct(),
                                  m_std_dev_trans, m_std_dev_rot, false);

    _Robot->setAndUpdate( *q2 );
    return q2;
}

confPtr_t PerturbationRoadmap::getExpansionConfiguration(bool sample_on_traj)
{
    double w = 0.0;

    if( sample_on_traj )
    {
        w = p3d_random( 0.0, 1.0 );
    }

    if( w < 0.2 ) // When sampling on traj do it half of the time
    {
        m_sampled_on_traj = false;
        return _Robot->shoot();
    }
    else {
        m_sampled_on_traj = true;
        return trajShootConfiguration();
    }
}

void PerturbationRoadmap::expandOneStep()
{
    // Get the random direction
    confPtr_t q = getExpansionConfiguration(true);

    // Get the expansion perturbation
    bool valid_perturb = expandPerturbation(q);

    if( valid_perturb ) {

        m_nbConscutiveFailures = 0;
        m_nbAddedNode++;

        //cout << "m_std_dev : " << m_std_dev << endl;
        //m_std_dev_trans = 1000/m_transl_steps;
        m_std_dev_trans = m_transl_max/10;
        m_std_dev_rot = 3.14/m_radian_steps;
        //m_radian_steps += 0.1;
        //m_transl_steps += 0.1;

        // 1000 - 50 = 950, 950/300
        // 15 - 50 = 35, 35/300 => 0.1

        if( m_traj ) {
            delete m_traj;
        }
        Move3D::Trajectory* traj = _Graph->extractAStarShortestPathsTraj( m_qi, m_qf );

        if( traj != NULL ) {

            m_traj = new Move3D::CostOptimization(*traj);
            delete traj;
            cout << "trajectory cost : " << m_traj->cost() << endl;
        }
        else {
            cout << "Trajectory not in graph" << endl;
        }

        if (ENV.getBool(Env::drawExploration)) {

            _Robot->setAndUpdate( *m_qf );

            (*_draw_func)();

            // Wait for window event
            // while ( PlanEnv->getBool(PlanParam::nextIterWaitForGui) );
            // PlanEnv->setBool(PlanParam::nextIterWaitForGui, true );
        }
    }
    else {
        m_nbConscutiveFailures++;
    }
}

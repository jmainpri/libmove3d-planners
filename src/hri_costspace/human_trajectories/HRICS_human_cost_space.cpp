/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#include "HRICS_human_cost_space.hpp"

#include "HRICS_human_simulator.hpp"
#include "HRICS_play_motion.hpp"
#include "API/project.hpp"

#include "planner/cost_space.hpp"
#include "planner/planEnvironment.hpp"

#include "p3d/env.hpp"

#include <boost/bind.hpp>
#include <iomanip>

using namespace Move3D;
using namespace HRICS;
using std::cout;
using std::endl;

HRICS::HumanTrajCostSpace* global_ht_cost_space = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------

bool HRICS_init_human_trajectory_cost()
{
    cout << "---------------------------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;
    cout << "---------------------------------------------" << endl;

    if( global_ht_cost_space == NULL )
    {
        Scene* sce = global_Project->getActiveScene();
        Robot* human1 = sce->getRobotByName( "HERAKLES_HUMAN1" );
        Robot* human2 = sce->getRobotByName( "HERAKLES_HUMAN2" );
        if( human1 == NULL || human2 == NULL )
        {
            cout << "No humans HERAKLES in the the scene" << endl;
            return false;
        }

        global_motionRecorders.push_back( new HRICS::RecordMotion( human1 ) );
        global_motionRecorders.push_back( new HRICS::RecordMotion( human2 ) );

        // std::string foldername = "/home/jmainpri/workspace/move3d/libmove3d/statFiles/collaboration/recorded_motion_01_09_13";
        // Set this bool to false is you want to print file names as they are loaded
//        bool quiet = true;
//        global_motionRecorders[0]->loadCSVFolder( foldername + "/human0", quiet );
//        global_motionRecorders[1]->loadCSVFolder( foldername + "/human1", quiet );


        global_motionRecorders[0]->useOpenRAVEFormat( true );
        global_motionRecorders[1]->useOpenRAVEFormat( true );

//        std::string foldername = "/home/jmainpri/Dropbox/move3d/move3d-launch/matlab/quan_motion";
//        motion_t traj1 = global_motionRecorders[0]->loadFromCSV( foldername + "/[1016#-#1112]#motion_saved_00000_00000.csv" );
//        motion_t traj2 = global_motionRecorders[1]->loadFromCSV( foldername + "/[1016#-#1112]#motion_saved_00001_00000.csv" );
//        global_motionRecorders[0]->storeMotion( traj1 );
//        global_motionRecorders[1]->storeMotion( traj2 );

//        std::string foldername = "/home/jmainpri/Dropbox/move3d/move3d-launch/matlab/kinect_good_motions/good_lib/";
        std::string foldername = "/home/jmainpri/Dropbox/move3d/move3d-launch/matlab/kinect_good_motions/human_one_good/";

        bool quiet = true;

        global_motionRecorders[0]->loadCSVFolder( foldername + "human_one/", quiet, +0.5 );
        global_motionRecorders[1]->loadCSVFolder( foldername + "human_two/", quiet, -0.5 );

        cout << "create human traj cost space" << endl;

        global_ht_cost_space = new HRICS::HumanTrajCostSpace( human2, human1 );

        // Set active joints and joint bounds
        global_ht_simulator = new HRICS::HumanTrajSimulator( global_ht_cost_space );
        global_ht_simulator->init();

        // Define cost functions
        cout << " add cost : " << "costHumanTrajectoryCost" << endl;
        global_costSpace->addCost( "costHumanTrajectoryCost", boost::bind( &HumanTrajCostSpace::cost, global_ht_cost_space, _1) );
    }

    ENV.setBool( Env::isCostSpace, true );
    global_costSpace->setCost( "costHumanTrajectoryCost" );

    if( !global_ht_cost_space->initCollisionSpace() )
        cout << "Error : could not init collision space" << endl;

    cout << " global_ht_cost_space : " << global_ht_cost_space << endl;
    global_activeFeatureFunction = global_ht_cost_space;

    return true;
}

void HRICS_run_human_planning()
{
    HRICS_init_human_trajectory_cost();

    HumanTrajSimulator sim( global_ht_cost_space );
    sim.init();
    sim.run();
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Human Trajectory Cost Space

HumanTrajCostSpace::HumanTrajCostSpace( Robot* active, Robot* passive ) :
    human_active_(active),
    human_passive_(passive),
    smoothness_feat_(),
    dist_feat_( active, passive ),
    visi_feat_(active, passive),
    musc_feat_( active ),
    collision_feat_( active )
{
    cout << "---------------------------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;

    nb_way_points_ = 20;

//    w_ = getFeatures( *human_active_->getCurrentPos() );
//    smoothness_feat_.setActiveDoFs( acti);

    active_dofs_ = std::vector<int>(1,1);

    length_feat_.setActiveDoFs( active_dofs_ );
    length_feat_.setWeights( 0.8 * WeightVect::Ones(length_feat_.getNumberOfFeatures()) );

    smoothness_feat_.setActiveDoFs( active_dofs_ );
    smoothness_feat_.setWeights( WeightVect::Ones(smoothness_feat_.getNumberOfFeatures()) );

    dist_feat_.setActiveDoFs( active_dofs_ );
    dist_feat_.setWeights( w_distance_16 );

    collision_feat_.setActiveDoFs( active_dofs_ );
    collision_feat_.setWeights( WeightVect::Ones(collision_feat_.getNumberOfFeatures()) );

    if(!addFeatureFunction( &length_feat_ ) ){
        cout << "Error adding feature length" << endl;
    }
//    if(!addFeatureFunction( &smoothness_feat_ ) ){
//        cout << "Error adding feature smoothness" << endl;
//    }
//    if(!addFeatureFunction( &collision_feat_ )){
//        cout << "Error adding feature distance collision" << endl;
//    }
    if(!addFeatureFunction( &dist_feat_ )){
        cout << "Error adding feature distance feature" << endl;
    }

    w_ = getWeights();

    cout << "w_ = " << w_.transpose() << endl;

    setAllFeaturesActive();
    printInfo();

    cout << "---------------------------------------------" << endl;
}

HumanTrajCostSpace::~HumanTrajCostSpace()
{

}

void HumanTrajCostSpace::setPassiveConfig( const Configuration& q )
{
    human_passive_->setAndUpdate(q);
}

void HumanTrajCostSpace::setPassiveTrajectory( const motion_t& motion )
{
    Move3D::Trajectory t( human_passive_ );

    for(int i=0;i<int(motion.size());i++)
    {
        t.push_back( motion[i].second->copy() );
    }
    t.cutTrajInSmallLP( nb_way_points_-1 );
    passive_traj_ = t;
    passive_traj_.replaceP3dTraj();
}

//FeatureVect HumanTrajCostSpace::getFeatures(const Configuration& q)
//{
//    FeatureVect vect = dist_feat_.getFeatures(q);
//    return vect;
//}

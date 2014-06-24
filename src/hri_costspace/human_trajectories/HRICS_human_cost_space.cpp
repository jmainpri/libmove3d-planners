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

#include "HRICS_play_motion.hpp"
#include "API/project.hpp"

#include "planner/cost_space.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/planEnvironment.hpp"

#include "p3d/env.hpp"

#include <boost/bind.hpp>
#include <iomanip>

using namespace Move3D;
using namespace HRICS;
using std::cout;
using std::endl;

HRICS::HumanTrajCostSpace* global_ht_cost_space = NULL;
HRICS::HumanTrajSimulator* global_ht_simulator = NULL;

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

        global_motionRecorders[0]->loadCSVFolder( foldername + "human_one/", false, +0.5 );
        global_motionRecorders[1]->loadCSVFolder( foldername + "human_two/", false, -0.5 );

        cout << "create human traj cost space" << endl;

        global_ht_cost_space = new HumanTrajCostSpace( human2, human1 );

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
// Human Trajectory Simulator

HumanTrajSimulator::HumanTrajSimulator( HumanTrajCostSpace* cost_space )  :
    cost_space_( cost_space ),
    human_active_( cost_space->getActiveHuman() ),
    human_passive_( cost_space->getPassiveHuman() ),
    init_scenario_(false)
{
    cout << "Human active : " << human_active_->getName() << endl;
    cout << "Human passive : " << human_passive_->getName() << endl;
}

bool HumanTrajSimulator::init()
{
//    if( !global_motionRecorders[0]->getStoredMotions().empty() )
//    {
//        cout << "Load init and goal from file" << endl;
//        const motion_t& motion_pas = global_motionRecorders[0]->getStoredMotions()[0];
//        const motion_t& motion_act = global_motionRecorders[1]->getStoredMotions()[0];

//        q_init_ = motion_act[0].second;
//        q_goal_ = motion_act.back().second;

//        // Adds the trajectory from the passive robot
//        // to the cost space
//        cost_space_->setPassiveTrajectory( motion_pas );
//    }
//    else
//    {
        q_init_ = human_active_->getInitPos();
        q_goal_ = human_active_->getGoalPos();
//    }

    // Set humans colors
    setHumanColor( human_active_, 3 );
    setHumanColor( human_passive_, 2 );

    // Sets the active robot as active for planning
    global_Project->getActiveScene()->setActiveRobot( human_active_->getName() );

    // Change dofs limit
    // human_active_->se

    // Set init and goal config
    human_active_->setInitPos( *q_init_ );
    human_active_->setGoalPos( *q_goal_ );

    // Set to init pose
    human_active_->setAndUpdate( *q_init_ );

    // Set bounds and dofs
    setActiveJoints();

    // set the pointers to the motion recorder
    motion_recorders_ = global_motionRecorders;

    return true;
}

void HumanTrajSimulator::setReplanningDemonstrations()
{
    std::vector<std::string> good_motions_names;
    good_motions_names.push_back( "[0551-0602]motion_saved_00000_00000.csv" );
    good_motions_names.push_back( "[1186-1245]motion_saved_00000_00001.csv" );
    good_motions_names.push_back( "[1552-1581]motion_saved_00000_00000.csv" );
    good_motions_names.push_back( "[1873-1929]motion_saved_00000_00001.csv" );
    good_motions_names.push_back( "[3191-3234]motion_saved_00000_00000.csv" );
    good_motions_names.push_back( "[3913-3950]motion_saved_00000_00000.csv" );
    good_motions_names.push_back( "[4125-4169]motion_saved_00000_00001.csv" );
    good_motions_names.push_back( "[4422-4476]motion_saved_00000_00000.csv" );
    good_motions_names.push_back( "[4591-4640]motion_saved_00000_00000.csv" );
    good_motions_names.push_back( "[4753-4802]motion_saved_00000_00000.csv" );

    for( size_t k=0; k<good_motions_names.size(); k++)
    {
        for(size_t j=0; j<motion_recorders_[0]->getStoredMotions().size(); j++)
        {
            if( motion_recorders_[0]->getStoredMotionName(j) == good_motions_names[k] )
            {
                human_1_motions_.push_back( motion_recorders_[0]->getStoredMotions()[j] );
                human_2_motions_.push_back( motion_recorders_[1]->getStoredMotions()[j] );
            }
        }
    }
}

std::vector< std::vector<motion_t> > HumanTrajSimulator::getMotions()
{
    std::vector< std::vector<motion_t> > motions;
    motions.push_back( human_1_motions_ );
    motions.push_back( human_2_motions_ );
    return motions;
}

void HumanTrajSimulator::setActiveJoints()
{
    // Get first joint and change bounds
    Joint* joint = human_active_->getJoint( "Pelvis" );

    // take only (x, y, z) components of the base
    double bound_trans = 0.05;
    double bound_rotat = 0.1;
    double dof[6][2];
    for(int i = 0; i < 3; i++) { // Translation bounds
        dof[i][0] = joint->getJointDof(i) - bound_trans;
        dof[i][1] = joint->getJointDof(i) + bound_trans;
        cout << "PELVIS DOF: " << joint->getJointDof(i) << endl;
    }
    for(int i = 3; i < 6; i++) { // Rotation bounds
        dof[i][0] = joint->getJointDof(i) - bound_rotat;
        dof[i][1] = joint->getJointDof(i) + bound_rotat;
    }
    for(int i = 0; i < 6; i++){
        p3d_jnt_set_dof_rand_bounds( joint->getP3dJointStruct(), i, dof[i][0], dof[i][1] );
    }

//    Joint(0), Dof : 6, Pelvis
//    Is dof user : (min = 0.94, max = 1.04)
//    Joint(1), Dof : 7, Pelvis
//    Is dof user : (min = 0.61, max = 0.71)
//    Joint(2), Dof : 8, Pelvis
//    Is dof user : (min = 0.92, max = 1.02)
//    Joint(3), Dof : 9, Pelvis
//    Is dof user : (min = -0.10, max = 0.10)
//    Joint(4), Dof : 10, Pelvis
//    Is dof user : (min = -0.10, max = 0.10)
//    Joint(5), Dof : 11, Pelvis

//    Is dof user : (min = -0.77, max = -0.57)
//    Joint(0), Dof : 12, TorsoX
//    Is dof user : (min = -0.79, max = 0.79)
//    Joint(0), Dof : 13, TorsoY
//    Is dof user : (min = -0.35, max = 0.70)
//    Joint(0), Dof : 14, TorsoZ

//    Is dof user : (min = -0.79, max = 0.79)
//    Joint(0), Dof : 18, rShoulderX
//    Is dof user : (min = -3.14, max = 3.14)
//    Joint(0), Dof : 19, rShoulderZ
//    Is dof user : (min = -3.14, max = 3.14)
//    Joint(0), Dof : 20, rShoulderY

//    Is dof user : (min = -3.14, max = 3.14)
//    Joint(0), Dof : 22, rElbowZ
//    Is dof user : (min = -3.14, max = 3.14)

    active_joints_.push_back( human_active_->getJoint( "Pelvis" )->getId() ); // Pelvis

    active_joints_.push_back( human_active_->getJoint( "TorsoX" )->getId() ); // TorsoX
    active_joints_.push_back( human_active_->getJoint( "TorsoY" )->getId() ); // TorsoY
    active_joints_.push_back( human_active_->getJoint( "TorsoZ" ) ->getId()); // TorsoZ

    active_joints_.push_back( human_active_->getJoint( "rShoulderX" )->getId() ); // rShoulderX
    active_joints_.push_back( human_active_->getJoint( "rShoulderZ" )->getId() ); // rShoulderZ
    active_joints_.push_back( human_active_->getJoint( "rShoulderY" )->getId() ); // rShoulderY
//    active_joints_.push_back( 11 ); // rArmTrans
    active_joints_.push_back( human_active_->getJoint( "rElbowZ" )->getId() ); // rElbowZ
//    active_joints_.push_back(14); // joint name : rWristX
//    active_joints_.push_back(15); // joint name : rWristY
//    active_joints_.push_back(16); // joint name : rWristZ

    // SET COST SPACE ACTIVE DOFS

    std::vector<int> active_dofs;

    for( size_t i=0; i<active_joints_.size(); i++ ) // get all active dofs
    {
        std::vector<unsigned int> jnt_active_dofs = human_active_->getJoint(active_joints_[i])->getDofIndices();
        active_dofs.insert( active_dofs.end(), jnt_active_dofs.begin(), jnt_active_dofs.end() );
    }

    for( size_t i=0; i<cost_space_->getNumberOfFeatureFunctions(); i++ ) // set all features active dofs
    {
        cost_space_->getFeatureFunction(i)->setActiveDoFs( active_dofs );
    }
}

void HumanTrajSimulator::setHumanColor(Robot* human, int color)
{
    if( !human->getUseLibmove3dStruct() )
        return;

    for (int i=1; i<= human->getP3dRobotStruct()->njoints; i++)
    {
        p3d_obj* obj = human->getP3dRobotStruct()->joints[i]->o;

        if(obj == NULL)
            continue;

        for(int j = 0; j < obj->np; j++)
        {
            p3d_poly* poly = obj->pol[j];

            if(poly->color_vect == NULL)
                continue;

            if (         poly->color_vect[0] == 0.80
                      && poly->color_vect[1] == 0.00
                      && poly->color_vect[2] == 0.01) {

                double color_vect[4];

                g3d_get_color_vect(color, color_vect);

                poly->color = color;

                poly->color_vect[0] = color_vect[0];
                poly->color_vect[1] = color_vect[1];
                poly->color_vect[2] = color_vect[2];
                poly->color_vect[3] = color_vect[3];
            }
        }
    }
}

bool HumanTrajSimulator::run()
{
    cout << "run human traj simulator" << endl;

    //    if( !init_scenario_ )
    //    {
    //        traj_optim_initScenario();
    //    }

    int nb_trajectories = 1;
    int max_iter = 100;

    traj_folder_ = std::string(getenv("HOME")) + "/Dropbox/move3d/assets/Collaboration/TRAJECTORIES/";

    for( int i=0;i<nb_trajectories;i++)
    {
        traj_optim_set_use_iteration_limit( true );
        traj_optim_set_iteration_limit( max_iter );
        traj_optim_runStomp(0);

        std::stringstream ss;
        ss << "trajectory" << std::setw(3) << std::setfill( '0' ) << i << ".traj";
        std::string filename = traj_folder_ + ss.str();

        p3d_save_traj( filename.c_str(), human_active_->getP3dRobotStruct()->tcur );

        cout << "save trajectory to : " << filename << endl;
    }
    return true;
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

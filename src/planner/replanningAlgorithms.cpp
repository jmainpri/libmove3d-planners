//
//  replanning.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 19/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "replanningAlgorithms.hpp"
#include "replanningSimulators.hpp"

#include "API/project.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/cost_space.hpp"
#include "planner/Diffusion/Variants/Star-RRT.hpp"

#include "HRICS_costspace.hpp"

#include "LightPlanner-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"

#include "move3d-headless.h"

#include <boost/thread/thread.hpp>
#include <sys/time.h>

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

//! Deactivate all kinematic constraints
void p3d_deactivate_all_cntrts( Robot* r )
{
    p3d_rob* rob = r->getRobotStruct();

    int nb_cntrts = rob->cntrt_manager->ncntrts;
    p3d_cntrt** cntrts = rob->cntrt_manager->cntrts;

    for(int i=0; i<nb_cntrts; i++)
        p3d_desactivateCntrt( rob , cntrts[i] );
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------

//! Initialize replanner by setting the robot, the multi-localpaths available
//! and calling the init function
Replanner::Replanner(Robot* r) : m_robot(r), m_human(NULL), m_useMLP(false)
{
    cout << "new base replanner" << endl;

    if( !init_mlp() )
    {
        cout << "Error initializing multi-localpahts in " << __FILE__ << " at " << __PRETTY_FUNCTION__ << endl;
    }
}

Replanner::~Replanner() 
{

}

void Replanner::setHuman(Robot* hum)
{
    m_human = hum;
}

void Replanner::setSwitchData( confPtr_t qSwitch, int switch_id, double s_switch, double t_rep, double lp_avera_length, double initial_step )
{
    m_qSwitch = qSwitch;
    m_switch_id = switch_id;
    m_s_switch = s_switch;
    m_t_rep = t_rep;
    m_lp_avera_length = lp_avera_length;
    m_initial_step = initial_step;
}

//! Initialize the robot multilocal path
//! disable the display of the main robot
bool Replanner::init_mlp()
{
    m_useMLP = false;

    if (m_robot == NULL)
        return false;

    for (int i=0; i<m_robot->getRobotStruct()->mlp->nblpGp; i++)
    {
        if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "base")) {
            m_BaseMLP = i;
            m_useMLP = true;
        } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "baseSm")) {
            m_BaseSmMLP = i;
            m_useMLP = true;
        } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "head")) {
            m_HeadMLP = i;
            m_useMLP = true;
        } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "upBody")) {
            m_UpBodyMLP = i;
            m_useMLP = true;
        } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "upBodySm")) {
            m_UpBodySmMLP = i;
            m_useMLP = true;
        }
    }

    cout << "Replanner::robot : " << m_robot->getName() << endl;
    cout << "Replanner::m_useMLP : " << m_useMLP << endl;
    m_init = true;
    return true;
}

//! Concat the new portion of the trajectory which is going to be used 
//! The concatanation happens at swith id which has to be computed before
std::pair<bool,API::Trajectory>  Replanner::concat_to_current_traj(const API::Trajectory& newPortion)
{
    pair<bool,API::Trajectory> pair;
    pair.first = false;

    if ( m_robot==NULL ) {
        cout <<  "concat_to_current_traj: robot is NULL.\n";
        return pair;
    }

    API::Trajectory concatTraj = m_CurrentTraj.extractSubTrajectory( 0, m_s_switch, false );

    cout << "concatTraj.getRangeMax() : " << concatTraj.getRangeMax() << endl;
    cout << "newPortion.getNbOfPaths() : " << newPortion.getNbOfPaths() << endl;
    cout << "newPortion.getRangeMax() : " << newPortion.getRangeMax() << endl;

    if( concatTraj.concat( newPortion ) ) {
        pair.first = true;
        pair.second = concatTraj;
    }
    cout << "concatTraj.getRangeMax() : " << concatTraj.getRangeMax() << endl;
    return pair;
}


//! Concat the new portion of the trajectory which is going to be used 
//! The concatanation happens at swith id which has to be computed before
p3d_traj* Replanner::concat_to_current_traj(const vector<p3d_traj*>& trajs)
{
    if ( m_robot==NULL ) {
        cout <<  "concat_to_current_traj: robot is NULL.\n";
        return NULL;
    }
    if ( trajs.empty() || !trajs[0] ) {
        cout <<  "concat_to_current_traj: the trajectory vector is empty.\n" ;
        return NULL;
    }

    // if the CurrentTraj is not empty
    // concat the allready executed part begore m_switch_id to the new trajectory
    int init_id;
    p3d_traj* concatTraj;
    if ( !m_CurrentTraj.isEmpty() ) {
        concatTraj = m_CurrentTraj.extractSubTrajectoryOfLocalPaths(0,m_switch_id).replaceP3dTraj(NULL);
        init_id = 0;
    }
    else {
        concatTraj = trajs[0];
        init_id = 1;
    }

    for (int i=init_id; i<int(trajs.size()); i++) {
        if(trajs[i]) {
            if (p3d_concat_traj( concatTraj, trajs[i]) == TRUE) {
                cout << "Concat traj fails" << endl;
                return NULL;
            }
        }
    }

    return concatTraj;
}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
SimpleReplanner::SimpleReplanner(Robot* r) : Replanner(r)
{

}

SimpleReplanner::~SimpleReplanner()
{

}

//! Before simple (no softmotion) replanning execution, this function initilizes the replanner data
//! First 
bool SimpleReplanner::init()
{
    m_isPlanning = false;

    set_active_joints( ENV.getInt(Env::setOfActiveJoints) );
    init_active_joints();

    // Store initial and final configurations
    m_qSwitch = m_robot->getCurrentPos();
    m_qGoal   = m_robot->getGoalPos();

    m_idRun = 0;

    switch (PlanEnv->getInt(PlanParam::replanningInitMethod))
    {
    case 0: {
        if( !init_create_straightline() ) {
            cout << "Error : could not create straightline" << endl;
            return false;
        }
    }
        break;

    case 1: {
        if( !p3d_run_rrt( m_robot->getRobotStruct() ) ){
            cout << " Error : could initialize with RRT" << endl;
            return false;
        }
        else {
            m_CurrentTraj = p3d_get_last_trajectory();
            cout << "m_CurrentTraj.getRangeMax() : " << m_CurrentTraj.getRangeMax() << endl;
        }
    }
        break;

    default: {
        cout << "No initializiation" << endl;
        return false;
    }
    }

    global_rePlanningEnv->setDrawStep( m_CurrentTraj.getRangeMax()/30 );
    global_rePlanningEnv->store_traj_to_draw( m_CurrentTraj, 0 );
    return true;
}

//! Creates a straight line trajectory
//! uses the main robot of the simulator or uses the current
//! robot if not initialized
bool SimpleReplanner::init_create_straightline()
{
    Robot* rob=NULL;

    if( m_robot != NULL ) {
        rob = m_robot;
    }
    else {
        rob = global_Project->getActiveScene()->getActiveRobot();
    }

    if (!rob) {
        cout << "Robot not initialized in file " << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    confPtr_t q_init( rob->getInitPos() );
    confPtr_t q_goal( rob->getGoalPos() );

    if( q_init->equal( *q_goal ) )
    {
        cout << "Init equal q_goal in file " << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        cout << "for robot : " << rob->getName() << endl;
        q_init->print();
        q_goal->print();
        return false;
    }

    vector<confPtr_t> confs(2);

    confs[0] = q_init;
    confs[1] = q_goal;

    m_CurrentTraj = API::Trajectory( confs );
    m_CurrentTraj.replaceP3dTraj();
    return true;
}

void SimpleReplanner::init_for_manipuation()
{
    if( m_useMLP )
    {
        cout << "Set upbody MLP" << endl;
#ifdef MULTILOCALPATH
        p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
        p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false);
#endif

        // Fix all the joints but unfixes the arm 0
        fixAllJointsWithoutArm( m_robot->getRobotStruct(), 0 );
    }

    // Deactivate all kinematic constraints
    //p3d_deactivate_all_cntrts( m_robot );
}

void SimpleReplanner::init_for_mobile_manip()
{
    if( m_useMLP )
    {
        cout << "Set upbody MLP" << endl;
#ifdef MULTILOCALPATH
        p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
        p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false);
#endif
    }

    // Fix all the joints but unfixes the arm 0
    fixAllJointsWithoutArm( m_robot->getRobotStruct(), 0 );

    // Unfix base joint for planning
    unFixJoint( m_robot->getRobotStruct(), m_robot->getRobotStruct()->baseJnt );

    // Deactivate all kinematic constraints
    //p3d_deactivate_all_cntrts( m_robot );
}

//! Set the DoFs active/passive of the robot for navigation
void SimpleReplanner::init_for_navigation()
{  
    if( m_useMLP )
    {
        cout << "Set base MLP" << endl;
#ifdef MULTILOCALPATH
        p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
        p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_BaseMLP, 1, false);
#endif

        // Fix all joints for planner
        // This only works with manipulation planner
        fixAllJointsExceptBase( m_robot->getRobotStruct() );
    }

    // Deactivate all kinematic constraints
    //p3d_deactivate_all_cntrts( m_robot );

    // Set base to joint to draw
    p3d_set_user_drawnjnt(1);
}

//! Choose what part of the robot are active in the planning phase
//! Three type of planning to chose from
//! Navigation, Manipulation, Mobile-anipulation
void SimpleReplanner::set_active_joints(int type)
{
    switch (type)
    {
    case 0:
        m_ReplanningType = NAVIGATION;
        break;

    case 1:
        m_ReplanningType = MANIPULATION;
        break;

    case 2:
        m_ReplanningType = MOBILE_MANIP;
        break;

    default:
        cout << "Planner type not implemented" << endl;
        break;
    }
}

void SimpleReplanner::init_active_joints()
{
    // Block specific joint sampling
    // also activate localpaths
    switch (m_ReplanningType)
    {
    case NAVIGATION:
        cout << "Init for Navigation" << endl;
        init_for_navigation();
        break;

    case MANIPULATION:
        cout << "Init for Manipulation" << endl;
        init_for_manipuation();
        break;

    case MOBILE_MANIP:
        cout << "Init for Mobile-manipulation" << endl;
        init_for_mobile_manip();
        break;
    }
}

void SimpleReplanner::run()
{
    m_isPlanning = true;

    // Set 0.3 seconds to do the concat traj
    PlanEnv->setBool( PlanParam::planWithTimeLimit, true );
    PlanEnv->setBool( PlanParam::trajWithTimeLimit, true );
    PlanEnv->setDouble( PlanParam::timeLimitSmoothing, m_t_rep-0.3 );
    cout << "m_t_rep : " << m_t_rep << endl;
    cout << "m_s_switch : " << m_s_switch << endl;
    cout << "m_CurrentTraj.getRangeMax() : " << m_CurrentTraj.getRangeMax() << endl;

    API::Trajectory newPortion = m_CurrentTraj.extractSubTrajectory( m_s_switch, m_CurrentTraj.getRangeMax(), false);
    cout << "newPortion.getNbOfPaths() : " << newPortion.getNbOfPaths() << endl;
    cout << "newPortion.getRangeMax() : " << newPortion.getRangeMax() << endl;

    API::CostOptimization optimTrj( newPortion );
    optimTrj.setStep( m_initial_step/PlanEnv->getDouble(PlanParam::MaxFactor) );
    optimTrj.runDeformation( PlanEnv->getInt(PlanParam::smoothMaxIterations), m_idRun++ );

    std::pair<bool,API::Trajectory> concated_traj= concat_to_current_traj( optimTrj );
    m_planningSucceded = concated_traj.first;

    if( m_planningSucceded ) {
        m_CurrentTraj = concated_traj.second;
    }

    if( optimTrj.getBegin() != m_qSwitch ) {
        cout << "Different q switch" << endl;
    }

    //store_traj_to_draw( optimTrj 0.0 );
    cout << "End replanning : " << __PRETTY_FUNCTION__ << endl;
    m_isPlanning = false;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
RRTReplanner::RRTReplanner(Robot* r) : SimpleReplanner(r)
{
    cout << "new star replanner" << endl;
}

RRTReplanner::~RRTReplanner()
{

}

bool RRTReplanner::init()
{
    SimpleReplanner::init();

    ENV.setBool(Env::isCostSpace,false);
    ENV.setBool(Env::useTRRT,false);
    return true;
}

void RRTReplanner::run()
{
    cout << "RRTReplanner::run"<<  endl;
    m_isPlanning = true;
    m_planningSucceded = false;

    // Set 0.3 seconds to do the concat traj
    PlanEnv->setBool( PlanParam::planWithTimeLimit, true );
    PlanEnv->setDouble(PlanParam::timeLimitPlanning, m_t_rep-0.3 );

    API::Trajectory traj(m_CurrentTraj.extractSubTrajectory( m_s_switch, m_CurrentTraj.getRangeMax(), false) );

    double t_init = 0.0;
    double time = 0.0;
    //bool first_call_to_chrono=true;

    m_idRun++;

    // Get the initial time of planning
    timeval tim;
    gettimeofday(&tim, NULL); t_init=tim.tv_sec+(tim.tv_usec/1000000.0);

    p3d_traj* path = p3d_planner_function( m_robot->getRobotStruct(),
                                           traj.getBegin()->getConfigStruct(),
                                           traj.getEnd()->getConfigStruct() );

    gettimeofday(&tim, NULL); time=tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;

    // Set the time left for smoothing
    PlanEnv->setBool( PlanParam::trajWithTimeLimit, true );
    PlanEnv->setDouble( PlanParam::timeLimitSmoothing, m_t_rep-0.3-time );

    if( path != NULL &&
            !PlanEnv->getBool(PlanParam::stopPlanner) &&
            PlanEnv->getBool(PlanParam::withSmoothing) )
    {
        p3d_smoothing_function( m_robot->getRobotStruct(), path, 100, -1 );
    }

    std::pair<bool,API::Trajectory> concated_traj;

    if( path ) {
        // Cutt the new trajectory in lp average piecese
        API::Trajectory path_( m_robot, path );
        path_.cutTrajInSmallLPSimple( traj.getRangeMax() / m_lp_avera_length );
        concated_traj = concat_to_current_traj( path_ );
        m_planningSucceded = concated_traj.first;
    }

    if( m_planningSucceded ) {
        m_CurrentTraj = concated_traj.second;
    }
    else {
        cout << "Replanning failed" << endl;
    }

    //store_traj_to_draw( optimTrj 0.0 );
    cout << "End replanning : " << __PRETTY_FUNCTION__ << endl;
    m_isPlanning = false;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
HandoverReplanner::HandoverReplanner(Robot* r) : SimpleReplanner(r)
{
    cout << "new star replanner" << endl;
}

HandoverReplanner::~HandoverReplanner()
{

}

bool HandoverReplanner::init()
{
    SimpleReplanner::init();

    m_goal_from_list = true;
    m_human = global_Project->getActiveScene()->getRobotByName("HERAKLES_HUMAN1");

    const char* home = getenv("HOME_MOVE3D");
    if( home == NULL ) {
        cout << "ERROR home is not defined for config generator in " << __PRETTY_FUNCTION__ << endl;
    }

    string dir(home);
    string file("/statFiles/OtpComputing/confHerakles.xml");

    m_goal_from_list = ( ENV.getInt(Env::setOfActiveJoints) == 2 );

    handoverGenerator_ = new ConfGenerator( m_robot, m_human );

    if(  HRICS_activeNatu == NULL )
        return false;
    else
        handoverGenerator_->initialize( dir+file, HRICS_activeNatu );

    return true;
}

std::pair<bool,confPtr_t> HandoverReplanner::newGoalFromList()
{
    double best_cost=0.0;
    pair<confPtr_t,confPtr_t> best_handover_conf;
    std::pair<bool,confPtr_t> result;

    // Parse list to find
    // the best feasible hand-over configuration
    result.first = handoverGenerator_->computeHandoverConfigFromList( best_handover_conf, best_cost );

    if( result.first ) {
        //human_model_->setAndUpdate( *best_handover_conf.first );
        //robot_model_->setAndUpdate( *best_handover_conf.second );
        result.second = best_handover_conf.second;
        p3d_update_virtual_object_config_for_arm_ik_constraint(m_robot->getRobotStruct(), 0, best_handover_conf.second->getConfigStruct() );
        //cout << "Hand-over found with cost : " << best_cost << endl;
    }
    return result;
}

std::pair<bool,confPtr_t> HandoverReplanner::newGoalFromIK()
{  
    configPt q = NULL;
    double dt = 0.0;

    timeval tim;
    gettimeofday(&tim, NULL);
    double t_init = tim.tv_sec+(tim.tv_usec/1000000.0);

    Eigen::Vector3d point = m_human->getJoint("rPalm")->getVectorPos();
    point[2] += 0.10;

    // if the generator finds a configuration, return it otherwise return null configuration
    if( handoverGenerator_->computeRobotIkForGrabing( q, point ) )
    {
        gettimeofday(&tim, NULL); double dt = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
        cout << "Valid robotIk computed in : " << dt << " sec" << endl;
        confPtr_t q_rob( new Configuration( m_robot, q ) );
        return std::make_pair( true, q_rob );
    }
    else
    {
        gettimeofday(&tim, NULL); dt = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
        cout << "Could not compute a valid robot IK configuration in : " << dt << "sec" << endl;
        p3d_destroy_config(m_robot->getRobotStruct(), q);
        confPtr_t q_rob( new Configuration( m_robot, NULL ) );
        return std::make_pair( false, q_rob );
    }
}

void HandoverReplanner::run()
{
    cout << "HandoverReplanner::run"<<  endl;
    m_isPlanning = true;
    m_planningSucceded = false;
    double t_init = 0.0;
    double time = 0.0;
    timeval tim;
    gettimeofday(&tim, NULL); t_init=tim.tv_sec+(tim.tv_usec/1000000.0);

    confPtr_t initConfig = m_CurrentTraj.configAtParam( m_s_switch );
    confPtr_t goalConfig = m_CurrentTraj.getEnd();

    m_idRun++;

    std::pair<bool,confPtr_t> goal;

    confPtr_t human_conf_tmp = m_human->getCurrentPos();

    if( m_goal_from_list )
        goal = newGoalFromList();
    else
        goal = newGoalFromIK();

    m_human->setAndUpdate( *human_conf_tmp );

    if( goal.first ) {
        goalConfig = goal.second;
        cout << "New goal found!!!" << endl;
    }

    // Set 0.3 seconds to do the concat traj
    PlanEnv->setBool( PlanParam::planWithTimeLimit, true );
    PlanEnv->setDouble(PlanParam::timeLimitPlanning, m_t_rep-0.3 );

    p3d_traj* path = p3d_planner_function( m_robot->getRobotStruct(),
                                           initConfig->getConfigStruct(),
                                           goalConfig->getConfigStruct() );

    //  Set the time left for smoothing
    gettimeofday(&tim, NULL); time=tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;

    if( path != NULL &&
            !PlanEnv->getBool(PlanParam::stopPlanner) &&
            PlanEnv->getBool(PlanParam::withSmoothing) )
    {
        p3d_smoothing_function( m_robot->getRobotStruct(), path, 100, m_t_rep-0.3-time);
    }

    if( path )
    {
        // Cut the new trajectory in lp average piecese
        API::Trajectory final_traj( m_robot, path );
        cout << "final_traj.getRangeMax() : " << final_traj.getRangeMax() << endl;

        std::pair<bool,API::Trajectory> concated_traj = concat_to_current_traj( final_traj );
        global_rePlanningEnv->store_traj_to_draw( final_traj, 0 );

        m_planningSucceded = concated_traj.first;
        if( !m_planningSucceded ) {
            cout << "Could not concatanate trajectory" << endl;
        }
        else {
            m_CurrentTraj = concated_traj.second;
        }
    }

    if( !m_planningSucceded ) {
        cout << "Replanning failed" << endl;
    }
    cout << "End replanning : " << __PRETTY_FUNCTION__ << endl;
    m_isPlanning = false;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------

StompReplanner::StompReplanner(Robot* r) : SimpleReplanner(r)
{

}

StompReplanner::~StompReplanner()
{

}

bool StompReplanner::init()
{
    return SimpleReplanner::init();
}

void StompReplanner::run()
{
    cout << "StompReplanner::run"<<  endl;
    m_isPlanning = true;
    m_planningSucceded = false;

    m_idRun++;
    
    API::Trajectory newPortion = m_CurrentTraj.extractSubTrajectory( m_s_switch, m_CurrentTraj.getRangeMax(), false);
    if( newPortion.getNbOfPaths() != 0 )
    {
        // Set 0.3 seconds to do the concat traj
        PlanEnv->setBool(PlanParam::trajStompWithTimeLimit,true);
        PlanEnv->setDouble(PlanParam::trajStompTimeLimit, m_t_rep-0.3);

        global_optimizer->setSource( newPortion.getBegin() );

        traj_optim_runStompNoInit( m_idRun, newPortion );

        // Get the new trajectory and store to draw
        API::Trajectory final_traj = global_optimizer->getBestTraj();
        cout << "final_traj.getRangeMax() : " << final_traj.getRangeMax() << endl;
        global_rePlanningEnv->store_traj_to_draw( final_traj, 0 );

        std::pair<bool,API::Trajectory> concated_traj = concat_to_current_traj( final_traj );
        m_planningSucceded = concated_traj.first;
        if( !m_planningSucceded ) {
            cout << "Could not concatanate trajectory" << endl;
        }
        else {
            m_CurrentTraj = concated_traj.second;
        }
    }
    else {
        cout << "newPortion.getNbOfPaths() : 0" << endl;
    }

    if( !m_planningSucceded ) {
        cout << "Replanning failed" << endl;
    }
    cout << "End replanning : " << __PRETTY_FUNCTION__ << endl;
    m_isPlanning = false;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
AStarReplanner::AStarReplanner(Robot* r) : SimpleReplanner(r)
{
    cout << "new star replanner" << endl;
}

AStarReplanner::~AStarReplanner()
{

}

bool AStarReplanner::init()
{
    SimpleReplanner::init();

    m_navigation = new HRICS::Navigation( m_robot );
    //  m_navigation->deactivateCynlinderWithAll();
    return true;
}

void AStarReplanner::run()
{
    cout << "AStarReplanner::run"<<  endl;
    m_isPlanning = true;
    m_planningSucceded = false;
    m_idRun++;

    // Get the initial time of planning
    timeval tim;
    gettimeofday(&tim, NULL); double t_init=tim.tv_sec+(tim.tv_usec/1000000.0);

    API::Trajectory traj( m_CurrentTraj.extractSubTrajectory( m_s_switch, m_CurrentTraj.getRangeMax(), false) );
    m_navigation->reset();

    API::Trajectory* path_ = m_navigation->computeRobotTrajectory( traj.getBegin(), traj.getEnd() );
    //  cout << "path length : " << path_->getRangeMax() << endl;

    gettimeofday(&tim, NULL); double time=tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;

    // Set the time left for smoothing
    PlanEnv->setBool( PlanParam::trajWithTimeLimit, true );
    PlanEnv->setDouble( PlanParam::timeLimitSmoothing, m_t_rep-0.3-time );

    if( path_ != NULL &&
            !PlanEnv->getBool(PlanParam::stopPlanner) &&
            PlanEnv->getBool(PlanParam::withSmoothing) )
    {
        p3d_smoothing_function( m_robot->getRobotStruct(), path_->replaceP3dTraj(NULL), 100, -1 );
    }

    if( path_ )
    {
        // Cut the new trajectory in lp average piecese
        API::Trajectory final_traj = m_robot->getCurrentTraj();
        final_traj.cutTrajInSmallLPSimple( traj.getRangeMax() / m_lp_avera_length );
        std::pair<bool,API::Trajectory> concated_traj = concat_to_current_traj( final_traj );
        cout << "path length : " << concated_traj.second.getRangeMax() << endl;
        global_rePlanningEnv->store_traj_to_draw( *path_, 0 );
        delete path_;

        m_planningSucceded = concated_traj.first;
        if( !m_planningSucceded ) {
            cout << "Could not concatanate trajectory" << endl;
        }
        else {
            m_CurrentTraj = concated_traj.second;
        }
    }

    if( !m_planningSucceded ) {
        cout << "Replanning failed" << endl;
    }
    cout << "End replanning : " << __PRETTY_FUNCTION__ << endl;
    m_isPlanning = false;
}

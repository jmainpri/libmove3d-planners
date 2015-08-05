//  trajectoryOptim.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 01/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//
// généraliser chomp => init codé en dur et ne tenant pas entièrement compte de la GUI
// régler les prob de freeze
// regarder le collision checker : trajectoire valide mais en collision à l'éxécution
// cinématique inverse à fond
// scénario du facteur + costmap

#include "trajectoryOptim.hpp"

#include "API/project.hpp"
#include "API/Device/robot.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "collision_space/collision_space.hpp"
#include "collision_space/collision_space_factory.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"
#include "feature_space/smoothness.hpp"

#include "utils/multilocalpath_utils.hpp"

#include "Chomp/chompTrajectory.hpp"
#include "Chomp/chompOptimizer.hpp"
#include "Chomp/chompParameters.hpp"

#include "Stomp/stompOptimizer.hpp"
#include "Stomp/stompParameters.hpp"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif
#include "Graphic-pkg.h"
#include "move3d-headless.h"

#include <boost/function.hpp>
#include <boost/bind.hpp>

using namespace Move3D;
using std::cout;
using std::endl;

MOVE3D_USING_SHARED_PTR_NAMESPACE

//--------------------------------------------------------
// Variables
//--------------------------------------------------------
static bool m_init = false;
static bool m_plan_for_human = false;

static Robot* m_robot = NULL;

static traj_optim::ScenarioType m_sce;

enum PlanningType { DEFAULT = -1, NAVIGATION = 0, MANIPULATION = 1, MOBILE_MANIP = 2 };
static PlanningType m_planning_type; 

static ChompPlanningGroup* m_chompplangroup= NULL;
static ChompTrajectory* m_chomptraj=NULL;
static ChompParameters* m_chompparams=NULL;

stomp_motion_planner::StompParameters* m_stompparams=NULL;

// External variable
std::vector< std::vector <double> > traj_optim_to_plot;
std::vector< std::vector <double> > traj_optim_convergence;

static bool m_use_iteration_limit=false;
static double m_max_iteration;

static bool m_use_external_trajectory=false;
static Move3D::Trajectory m_external_trajectory;

static bool m_discretize=false;
static double m_discretization=0.0;

static bool m_use_buffer;
static std::vector<Eigen::VectorXd> m_buffer;


//! Create an initial Move3D trajectory
//! it generates a straigt line between the two configuration init 
//! and goal
// --------------------------------------------------------
Move3D::Trajectory traj_optim_create_sraight_line_traj()
{
    if (!m_robot) {
        cout << "robot not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
    }

    confPtr_t q_init( m_robot->getInitPos() );
    confPtr_t q_goal( m_robot->getGoalPos() );

    if( q_init->equal( *q_goal ) )
    {
        cout << "init equal q_goal in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
    }

    std::vector<confPtr_t> confs(2);

    confs[0] = q_init;
    confs[1] = q_goal;

    Move3D::Trajectory T( confs );

    return T;
}

//! Set the type of scenario
//! Depending on global variables
// --------------------------------------------------------
bool traj_optim_set_scenario_type()
{
    if( ENV.getBool(Env::isCostSpace) && ( global_costSpace == NULL )) {
        cout << "Cost space not initialized!!!" << endl;
        return false;
    }

    if( m_planning_type == DEFAULT )
    {
        if( ENV.getBool(Env::isCostSpace)
                && global_costSpace->getSelectedCostName() == "costMap2D" )
        {
            m_sce = traj_optim::CostMap;
        }
        //    else if( ENV.getBool(Env::isCostSpace) &&
        //            ( global_costSpace->getSelectedCostName() == "costIsInCollision" ||
        //             global_costSpace->getSelectedCostName() == "costDistToObst"  ))
        //    {
        //      m_sce = Simple;
        //    }
        else
        {
            m_sce = traj_optim::Default;
        }
    }
    else if( ENV.getBool(Env::isCostSpace) &&
             ( global_costSpace->getSelectedCostName() == "costHumanGrids" ||
               global_costSpace->getSelectedCostName() == "costHumanPredictionOccupancy" ) )
    {
        if( m_planning_type == NAVIGATION )
        {
            m_sce = traj_optim::HumanAwareNav;
        }
        if( m_planning_type == MANIPULATION )
        {
            m_sce = traj_optim::HumanAwareManip;
        }
        if( m_planning_type == MOBILE_MANIP )
        {
            m_sce = traj_optim::HumanAwareMobileManip;
        }
    }
    else if ( ENV.getBool(Env::isCostSpace) &&
              global_costSpace->getSelectedCostName() == "costHumanTrajectoryCost" )
    {
        if( m_planning_type == MANIPULATION )
        {
            cout << "Set human cost function" << endl;
            m_sce = traj_optim::HumanSimulation;
        }
    }
    else if (  PlanEnv->getBool(PlanParam::useLegibleCost) )
    {
        if( m_planning_type == NAVIGATION )
        {
            cout << "Set legible cost function" << endl;
            cout << "Set default" << endl;
            m_sce = traj_optim::LegiblePlane;
        }
    }
    else
    {
        const bool navigation = false;

        if( navigation )
            m_sce = traj_optim::Navigation;
        else
            m_sce = traj_optim::Shelf;
    }

    return true;
}

//! Choose what part of the robot are active in the planning phase
//! Three type of planning to chose from
//! Navigation, Manipulation, Mobile-anipulation
void traj_optim_init_planning_type(int type)
{
    switch (type)
    {
    case 0:
        m_planning_type = NAVIGATION;
        cout << "NAVIGATION planner type" << endl;
        break;

    case 1:
        m_planning_type = MANIPULATION;
        cout << "MANIPULATION planner type" << endl;
        break;

    case 2:
        m_planning_type = MOBILE_MANIP;
        cout << "MOBILE_MANIP planner type" << endl;
        break;

    default:
        m_planning_type = DEFAULT;
        cout << "DEFAULT planner type" << endl;
        break;
    }
}

//****************************************************************
//*   Run Functions 
//****************************************************************

bool traj_optim_resetInit()
{
    m_init = false;
    return true;
}

bool traj_optim_initScenario()
{
    traj_optim_init_planning_type( ENV.getInt(Env::setOfActiveJoints) );

    cout << "init collision space" << endl;

    if(!traj_optim_set_scenario_type()) {
        cout << "Not well initialized" << endl;
        return false;
    }

    // THE ROBOT IS SET HERE !!!!
    m_robot = global_Project->getActiveScene()->getActiveRobot();
    // m_robot = global_Project->getActiveScene()->getRobotByNameContaining("ROBOT");

    if( global_costSpace )
        cout << "COST FUNCTION is : "
             << global_costSpace->getSelectedCostName() << endl;
    else
        cout << "NO COST FUNCTION" << endl;

    if(!traj_optim_init_collision_spaces( m_sce, m_robot )){
        cout << "Not well initialized" << endl;
        return false;
    }

    m_init = true;
    return true;
}

//!
//! Get Initial trajectory
// --------------------------------------------------------
bool traj_optim_InitTraj( Move3D::Trajectory& T )
{
    if( m_use_external_trajectory )
    {
        T = m_external_trajectory;
    }
    else if( PlanEnv->getBool(PlanParam::withCurrentTraj) )
    {
        T = m_robot->getCurrentTraj();
    }
    else
    {
        T = traj_optim_create_sraight_line_traj();
    }

    if( T.getNbOfPaths() == 0 )
        return false;

    int nb_points = 0;

    if( m_discretize )
    {
        nb_points = floor( T.getParamMax() / m_discretization );
    }
    else
    {
        nb_points = PlanEnv->getInt( PlanParam::nb_pointsOnTraj );
    }

    T.cutTrajInSmallLP( nb_points-1 );
    T.replaceP3dTraj();

    cout << "End Init traj" << endl;
    return true;
}

//! Chomp
// --------------------------------------------------------
bool traj_optim_runChomp()
{
    if( !m_init )
    {
        if(!traj_optim_initScenario())
        {
            return false;
        }
    }

    Move3D::Trajectory T(m_robot);

    if( !traj_optim_InitTraj(T) ){
        return false;
    }

    int nb_points = PlanEnv->getInt( PlanParam::nb_pointsOnTraj );

    T.cutTrajInSmallLP( nb_points-1 );
    T.replaceP3dTraj();

    g3d_draw_allwin_active();

    // Create Optimizer
    // ----------------
    m_chompparams = new ChompParameters;
    m_chompparams->init();

    m_chompplangroup = new ChompPlanningGroup( m_robot, traj_optim_get_planner_joints() );
    m_chompplangroup->collision_points_ = traj_optim_get_collision_points();

    m_chomptraj = new ChompTrajectory( T, DIFF_RULE_LENGTH, *m_chompplangroup, PlanEnv->getDouble(PlanParam::trajDuration) );
    m_chomptraj->print();
    cout << "chomp Trajectory has npoints : " << m_chomptraj->getNumPoints() << endl;
    cout << "Initialize optimizer" << endl;

    ChompOptimizer optimizer( m_chomptraj, m_chompparams, m_chompplangroup, traj_optim_get_collision_space() );
    cout << "Optimizer created" << endl;

    optimizer.runDeformation(0,0);
    return true;
}

//! Init Stomp
// --------------------------------------------------------
bool traj_optim_initStomp()
{
    cout << "----------------------------------" << endl;
    cout << " Init Stomp ----------------------" << endl;

    if( !m_init )
    {
        if(!traj_optim_initScenario())
        {
            return false;
        }
    }

    Move3D::Trajectory T(m_robot);

    if( !traj_optim_InitTraj(T) )
    {
        cout << "Error in traj_optim_InitTraj" << endl;
        return false;
    }

    // Save passive dof to generate the Move3D trajectory
    std::vector<confPtr_t> passive_dofs = T.getVectorOfConfiguration();

    //  if( (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) )
    //  {
    //    g3d_draw_allwin_active();
    //  }

    delete m_stompparams;
    delete m_chompplangroup;
    delete m_chomptraj;

    // Create Optimizer
    // ----------------
    m_stompparams = new stomp_motion_planner::StompParameters;
    m_stompparams->init();

    m_chompplangroup = new ChompPlanningGroup( m_robot, traj_optim_get_planner_joints() );
    m_chompplangroup->collision_points_ = traj_optim_get_collision_points();

    m_chomptraj = new ChompTrajectory(
                T, DIFF_RULE_LENGTH,
                *m_chompplangroup, PlanEnv->getDouble(PlanParam::trajDuration) );
    // m_chomptraj->print();

    cout << "Chomp Trajectory has npoints : " << m_chomptraj->getNumPoints() << endl;
    cout << "PlanEnv->getDouble(PlanParam::trajDuration) : " << PlanEnv->getDouble(PlanParam::trajDuration) << endl;
    cout << "Chomp Trajectory uses time : " << m_chomptraj->getUseTime() << endl;

    cout << "Initialize optimizer" << endl;
    global_optimizer.reset( new stomp_motion_planner::StompOptimizer( m_chomptraj, m_stompparams, m_chompplangroup, traj_optim_get_collision_space() ));
    global_optimizer->setSource( T.getBegin() );
    global_optimizer->setSharedPtr( global_optimizer );
    global_optimizer->setPassiveDofs( passive_dofs );

    m_stompparams->init();

    if( PlanEnv->getBool(PlanParam::trajStompWithIterLimit) || m_use_iteration_limit )
    {
        global_optimizer->setUseIterationLimit( true );
        m_stompparams->max_iterations_ = PlanEnv->getInt(PlanParam::stompMaxIteration);
    }

    if( PlanEnv->getBool(PlanParam::trajStompWithTimeLimit) )
    {
        global_optimizer->setUseTimeLimit( true );
        m_stompparams->max_time_ = PlanEnv->getDouble(PlanParam::trajStompTimeLimit);
    }

    if(     (m_sce == traj_optim::HumanAwareManip && m_robot->getName() == "PR2_ROBOT") ||
            (m_sce == traj_optim::HumanSimulation ) ||
            (m_sce == traj_optim::CostMap) )
    {
        global_optimizer->setUseCostSpace( true );
    }

    if( m_use_buffer )
    {
        global_optimizer->setBuffer( m_buffer );

        cout << "SET BUFFER" << endl;
        Move3D::StackedFeatures* fct = dynamic_cast<StackedFeatures*>( global_activeFeatureFunction );
        if( fct != NULL && fct->getFeatureFunction("SmoothnessAll") != NULL )
            static_cast<SmoothnessFeature*>(fct->getFeatureFunction("SmoothnessAll"))->setBuffer( m_buffer );
    }
    else
    {
        global_optimizer->clearBuffer();

        cout << "CLEAR BUFFER" << endl;
        Move3D::StackedFeatures* fct = dynamic_cast<StackedFeatures*>( global_activeFeatureFunction );
        if( fct != NULL && fct->getFeatureFunction("SmoothnessAll") != NULL )
            static_cast<SmoothnessFeature*>(fct->getFeatureFunction("SmoothnessAll"))->clearBuffer();

    }

    // Initialize all data structures
    global_optimizer->initialize();

    cout << "Optimizer created" << endl;

    GlobalCostSpace::initialize();
    // std::cout << "Initializing the collision space function" << std::endl;
    // global_costSpace->addCost("CollisionSpace",boost::bind(computeCollisionSpaceCost, _1));
    // global_costSpace->setCost("CollisionSpace");

    m_robot->setAndUpdate( *m_robot->getInitPos() );

    return true;
}

bool traj_optim_runStomp( int runId )
{
//    cout << "Robot is : " << m_robot->getName() << endl;

    if(!traj_optim_initStomp() )
    {
        cout << "Could not init stomp in : "
             << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    if(!PlanEnv->getBool(PlanParam::trajOptimTestMultiGauss))
    {
        global_optimizer->runDeformation( 0, runId );
        //optimizer->generateNoisyTrajectories();
    }
    else {
        global_optimizer->testMultiVariateGaussianSampler();
    }

    // WHY IS THIS HERE
//    global_optimizer->resetSharedPtr();
    return true;
}

bool traj_optim_runStompNoInit( int runId, const Move3D::Trajectory& traj )
{
    cout << "run stomp no init" << endl;

    if( PlanEnv->getBool(PlanParam::trajStompWithTimeLimit) )
    {
        global_optimizer->setUseTimeLimit( true );
        m_stompparams->max_time_ = PlanEnv->getDouble(PlanParam::trajStompTimeLimit);
    }
    else {
        global_optimizer->setUseTimeLimit( false );
    }

    cout << "set iteration limit" << endl;

    if( m_use_iteration_limit )
    {
        global_optimizer->setUseIterationLimit( true );
        m_stompparams->max_iterations_ = m_max_iteration;
    }

    global_optimizer->setUseOtp( PlanEnv->getBool(PlanParam::trajUseOtp) );
    global_optimizer->initializeFromNewTrajectory( traj );

    cout << "run deformation" << endl;

    global_optimizer->runDeformation( 0, runId );
    return true;
}

bool traj_optim_runStompNoReset(int runId)
{
    traj_optim_switch_cartesian_mode( false );

    Move3D::Trajectory traj( m_robot );

    if( !traj_optim_InitTraj( traj ) ){
        return false;
    }

    //  Move3D::Trajectory traj = m_robot->getCurrentTraj();
    return traj_optim_runStompNoInit( runId, traj );
}

//! Run Stomp
// --------------------------------------------------------
void traj_optim_plan_for_human(bool use)
{
    m_plan_for_human = use;
}

void traj_optim_set_use_iteration_limit(bool use)
{
    m_use_iteration_limit = use;
}

void traj_optim_set_iteration_limit(int max_iter)
{
    m_max_iteration = max_iter;
}

void traj_optim_set_use_extern_trajectory( bool use )
{
    m_use_external_trajectory = use;
}

void traj_optim_set_extern_trajectory( const Move3D::Trajectory& traj )
{
    m_external_trajectory = traj;
}

void traj_optim_set_discretize( bool discretize )
{
    m_discretize = discretize;
}

void traj_optim_set_discretization( double discretization )
{
    m_discretization = discretization;
}

void traj_optim_clear_buffer()
{
    m_use_buffer = false;
}

void traj_optim_set_buffer( const std::vector<Eigen::VectorXd>& buffer )
{
    m_use_buffer = true;
    m_buffer = buffer;
}

// --------------------------------------------------------
// Draw Functions
// --------------------------------------------------------
void traj_optim_draw_collision_points()
{
    if (m_chompplangroup && !m_chompplangroup->collision_points_.empty())
    {
        m_chompplangroup->draw();
    }
}

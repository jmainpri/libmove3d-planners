#include "HRICS_HumanPredictionSimulator.hpp"
#include "HRICS_HumanPredictionCostSpace.hpp"

#include "API/project.hpp"

#include "planner/cost_space.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"

#include "p3d/env.hpp"
#include "Graphic-pkg.h"

#include <boost/bind.hpp>

using namespace HRICS;

HumanPredictionSimulator* global_humanPredictionSimulator = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Initialization

void HRICS_initOccupancyPredictionFramework()
{
    std::vector<double> size = global_Project->getActiveScene()->getBounds();

    Robot* robot = global_Project->getActiveScene()->getRobotByName( "PR2_ROBOT" );
    Robot* human = global_Project->getActiveScene()->getRobotByName( "HERAKLES_HUMAN1" );

    if( robot == NULL || human == NULL )
    {
        cout << "human or robot not defined" << endl;
        return;
    }

    RecordMotion* recorder = new RecordMotion( human );

    if( recorder->loadRegressedFromCSV() )
    {
        cout << "Motions loaded successfully" << endl;
    }
    else {
         cout << "Error loading motions" << endl;
         delete recorder;
         return;
    }

    ClassifyMotion* classifier = new ClassifyMotion();

    if( classifier->load_model() )
    {
        cout << "GMMs loaded successfully" << endl;
    }
    else {
         cout << "Error loading GMMs" << endl;
         delete recorder;
         delete classifier;
         return;
    }

    // Translate Regressed motions
    recorder->translateStoredMotions();

    WorkspaceOccupancyGrid* occupancyGrid = new WorkspaceOccupancyGrid( human, 0.05, size );
    occupancyGrid->setRegressedMotions( recorder->getStoredMotions() );
    occupancyGrid->computeOccpancy();
    occupancyGrid->setClassToDraw(1);

    // Create the prediction costspace
    HumanPredictionCostSpace* predictionSpace = new HumanPredictionCostSpace( robot, occupancyGrid );

    // Create the simulator
    HumanPredictionSimulator* simulator = new HumanPredictionSimulator( robot, human, recorder, classifier, occupancyGrid );
    recorder->loadFolder();
    // Translate all motions
    recorder->translateStoredMotions();
    simulator->loadHumanTrajectory( recorder->getStoredMotions()[30] );

    // GUI global variables
    global_motionRecorder = recorder;
    global_classifyMotion = classifier;
    global_workspaceOccupancy = occupancyGrid;
    global_humanPredictionCostSpace = predictionSpace;
    global_humanPredictionSimulator = simulator;

    // Define cost functions
    global_costSpace->addCost( "costHumanPredictionOccupancy" , boost::bind(HRICS_getPredictionOccupancyCost, _1) );
    global_costSpace->setCost( "costHumanPredictionOccupancy" );
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Simulator

HumanPredictionSimulator::HumanPredictionSimulator( Robot* robot, Robot* human, RecordMotion* recorder, ClassifyMotion* classifier, WorkspaceOccupancyGrid* occupacy_grid )
{
    m_robot = robot;
    m_human = human;
    m_recorder = recorder;
    m_classifier = classifier;
    m_occupacy_grid = occupacy_grid;
    m_current_traj.resize( 0, 0 );
    m_human_increment = 20;
    m_max_stomp_iter = 30;
    m_use_previous_trajectory = true;
}

void HumanPredictionSimulator::loadHumanTrajectory( const motion_t& motion )
{
    m_motion = motion;
}

void HumanPredictionSimulator::setHumanConfig( confPtr_t q )
{
    confPtr_t q_cur = m_human->getCurrentPos();
    (*q_cur)[6] =  (*q)[6];  // Pelvis
    (*q_cur)[7] =  (*q)[7];  // Pelvis
    (*q_cur)[8]=   (*q)[8];  // Pelvis
    (*q_cur)[11] = (*q)[11]; // Pelvis
    (*q_cur)[12] = (*q)[12]; // TorsoX
    (*q_cur)[13] = (*q)[13]; // TorsoY
    (*q_cur)[14] = (*q)[14]; // TorsoZ
    (*q_cur)[18] = (*q)[18]; // rShoulderX
    (*q_cur)[19] = (*q)[19]; // rShoulderZ
    (*q_cur)[20] = (*q)[20]; // rShoulderY
    (*q_cur)[21] = (*q)[21]; // rArmTrans
    (*q_cur)[22] = (*q)[22]; // rElbowZ
    m_human->setAndUpdate(*q_cur);
}

void HumanPredictionSimulator::setMatrixCol(Eigen::MatrixXd& matrix, int j, confPtr_t q)
{
    matrix(0,j) = j;        // Time
    matrix(1,j) = (*q)[6];  // Pelvis
    matrix(2,j) = (*q)[7];  // Pelvis
    matrix(3,j) = (*q)[8];  // Pelvis
    matrix(4,j) = (*q)[11]; // Pelvis
    matrix(5,j) = (*q)[12]; // TorsoX
    matrix(6,j) = (*q)[13]; // TorsoY
    matrix(7,j) = (*q)[14]; // TorsoZ

    matrix(8,j) =  (*q)[18];  // rShoulderX
    matrix(9,j) =  (*q)[19];  // rShoulderZ
    matrix(10,j) = (*q)[20]; // rShoulderY
    matrix(11,j) = (*q)[21]; // rArmTrans
    matrix(12,j) = (*q)[22]; // rElbowZ
}

int HumanPredictionSimulator::classifyMotion( const motion_t& motion )
{
    std::vector<double> likelihood;

    // for all collumns (a configuration per collumn)
    for (int j=1; j<int(motion.size()); j++)
    {
        Eigen::MatrixXd matrix( 13, j );

        for (int jn=0; jn<j; jn++)
        {
            confPtr_t q = motion[jn].second;
            setMatrixCol( matrix, jn, q );
        }

        likelihood = m_classifier->classify_motion( matrix );
        cout << std::max_element(likelihood.begin(),likelihood.end()) - likelihood.begin() << " ";
    }
    cout << endl;

    return std::max_element(likelihood.begin(),likelihood.end()) - likelihood.begin();
}


bool HumanPredictionSimulator::updateMotion()
{
    cout << "updateMotion" << endl;
    int end = m_current_traj.cols() + m_human_increment;

    if( end > int(m_motion.size()) )
    {
        return false;
    }

    m_current_traj.resize( 13, end );

    motion_t motion = m_recorder->invertTranslation( m_motion );

    for (int j=0; j<m_current_traj.cols(); j++)
    {
        confPtr_t q = motion[j].second;
        setMatrixCol( m_current_traj, j, q );
    }
    //cout << cout << m_current_traj << endl;

    setHumanConfig( m_motion[m_current_traj.cols()-1].second );
    return true;
}

void HumanPredictionSimulator::predictVoxelOccupancy()
{
    cout << "predict voxel occpancy" << endl;
    std::vector<double> likelihood = m_classifier->classify_motion( m_current_traj );
    m_occupacy_grid->setLikelihood( likelihood );
}

void HumanPredictionSimulator::loadGoalConfig()
{
    m_goal_config.clear();

    p3d_rob* rob = m_robot->getRobotStruct();

    for( int i=0; i<rob->nconf; i++ )
    {
        std::string name = rob->conf[i]->name;
        cout << "Add goal config : " << name << endl;
        m_goal_config.push_back( confPtr_t(new Configuration( m_robot, rob->conf[i]->q )) );
    }

    for( int i=0; i<rob->nconf; i++ )
    {
        m_robot->setAndUpdate(*m_goal_config[i]);
        g3d_draw_allwin_active();
        sleep(1);
    }

    m_q_start = m_robot->getInitialPosition();

    m_paths.resize( m_goal_config.size() );
}

void HumanPredictionSimulator::runStomp( int iter, int id_goal  )
{
    PlanEnv->setInt( PlanParam::nb_pointsOnTraj, 100 );

    m_robot->setAndUpdate( *m_q_start );
    m_robot->setInitialPosition( *m_q_start );
    m_robot->setGoTo( *m_goal_config[id_goal] );

    if( (iter>0) && (id_goal == m_best_path_id) )
    {
        const API::Trajectory& traj = m_paths[id_goal];
        const double parameter =  m_robot_steps_per_exection*m_robot_step;
        traj_optim_set_use_extern_trajectory( true );
        traj_optim_set_extern_trajectory( traj.extractSubTrajectory( parameter, traj.getRangeMax(), false ) );
    }
    else
    {
        traj_optim_set_use_extern_trajectory( false );
    }

    traj_optim_set_use_iteration_limit(true);
    traj_optim_runStomp(0);

    m_paths[id_goal] = optimizer->getBestTraj();
}

int HumanPredictionSimulator::getBestPathId()
{
    int id = std::min_element( m_paths.begin(), m_paths.end() ) - m_paths.begin();

    cout << "best id : " << id << endl;

    for(int i=0;i<int(m_paths.size());i++)
    {
        cout << "path " << i << " cost : " << m_paths[i].cost() << endl;
    }
    return id;
}

void HumanPredictionSimulator::execute(const API::Trajectory& path, bool to_end)
{
    double s = 0.0;

    path.replaceP3dTraj();

    confPtr_t q;

    if( !to_end )
    {
        for( int i=0; (i<m_robot_steps_per_exection) && (!PlanEnv->getBool(PlanParam::stopPlanner) ); i++ )
        {
            q = path.configAtParam( s );
            m_robot->setAndUpdate(*q);
            g3d_draw_allwin_active();
            usleep(25000);
            s += m_robot_step;
        }
    }
    else{
        while( s<path.getRangeMax() && !PlanEnv->getBool(PlanParam::stopPlanner) )
        {
            m_robot->setAndUpdate(*path.configAtParam(s));
            g3d_draw_allwin_active();
            usleep(25000);
            s += m_robot_step;
        }
    }

    m_q_start = q;
}

void HumanPredictionSimulator::run()
{
    ENV.setBool(Env::drawGraph,false);
    ENV.setBool(Env::drawTraj,true);

    m_robot_steps_per_exection = 20;
    m_robot_step = 0.005;

    loadGoalConfig();

    m_best_path_id = -1;

    for(int i=0;(!PlanEnv->getBool(PlanParam::stopPlanner)) && updateMotion();i++)
    {
        predictVoxelOccupancy();
        g3d_draw_allwin_active();

        for(int j=0;j<int(m_goal_config.size());j++)
        {
            runStomp( i, j);
        }

        m_best_path_id = getBestPathId();

        execute( m_paths[m_best_path_id] );
    }

    if( m_best_path_id != -1 )
    {
        const API::Trajectory& traj = m_paths[m_best_path_id];
        const double parameter =  m_robot_steps_per_exection*m_robot_step;
        execute( traj.extractSubTrajectory( parameter, traj.getRangeMax(), false ), true );
    }
}

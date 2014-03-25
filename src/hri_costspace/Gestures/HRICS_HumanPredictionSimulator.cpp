#include "HRICS_HumanPredictionSimulator.hpp"
#include "HRICS_HumanPredictionCostSpace.hpp"
#include "HRICS_GestParameters.hpp"

#include "API/project.hpp"

#include "planner/cost_space.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planner/TrajectoryOptim/Stomp/run_parallel_stomp.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"

#include "p3d/env.hpp"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"

#include <boost/bind.hpp>

using namespace Move3D;
using namespace HRICS;
using namespace stomp_motion_planner;
using std::cout;
using std::endl;

HumanPredictionSimulator* global_humanPredictionSimulator = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Initialization

void HRICS_initOccupancyPredictionFramework()
{
    cout << "---------------------------------------------------------------" << endl;
    cout << " HRICS_initOccupancyPredictionFramework " << endl;
    cout << "---------------------------------------------------------------" << endl;

    std::vector<double> size = global_Project->getActiveScene()->getBounds();

    Robot* robot = global_Project->getActiveScene()->getRobotByName( "PR2_ROBOT" );
    Robot* human = global_Project->getActiveScene()->getRobotByName( "HERAKLES_HUMAN1" );

    if( robot == NULL || human == NULL )
    {
        cout << "human or robot not defined" << endl;
        return;
    }

    RecordMotion* recorder = new RecordMotion( human );

    if( global_motionRecorders.empty() )
        global_motionRecorders.push_back( recorder );

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

    const std::vector<motion_t>& r_motions = recorder->getStoredMotions();
    if( r_motions.empty() )
    {
        cout << "Could not load regressed motions" << endl;
        return;
    }

    occupancyGrid->setRegressedMotions( r_motions );
    if( !occupancyGrid->computeOccpancy() )
    {
        cout << "Could not compute workspace occupancy" << endl;
        return;
    }

    occupancyGrid->setClassToDraw(1);

    // Create the prediction costspace
    HumanPredictionCostSpace* predictionSpace = new HumanPredictionCostSpace( robot, occupancyGrid );

    // Create the simulator
    HumanPredictionSimulator* simulator = new HumanPredictionSimulator( robot, human, recorder, classifier, occupancyGrid );
    recorder->loadXMLFolder();
    // Translate all motions
    recorder->translateStoredMotions();

    // 28 (25,26,27,28,29) Side by side (class 1) starting in 0
    // 77, Face to face (class 3)
    //cout << "loading trajectory : " << GestEnv->getInt(GestParam::human_traj_id) << endl;
    //simulator->loadHumanTrajectory( recorder->getStoredMotions()[GestEnv->getInt(GestParam::human_traj_id)] );

    // GUI global variables
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
    m_current_human_traj.resize( 0, 0 );

    m_human_increment = 5;
    //m_robot_steps_per_exection = 10; // side by side
    m_robot_steps_per_exection = 17;  // facing
    m_robot_step = 0.005;

    m_use_previous_trajectory = true;
    m_executed_path = Move3D::Trajectory(m_robot);
    m_is_scenario_init = false;

    m_colors.resize(2);

    m_colors[0].push_back( 1.000 ); //1.00000   0.74902   0.16863
    m_colors[0].push_back( 0.749 );
    m_colors[0].push_back( 0.168 );
    m_colors[0].push_back( 0 );

    m_colors[1].push_back( 0.168 ); // 0.16863   1.00000   0.40392
    m_colors[1].push_back( 1.000 );
    m_colors[1].push_back( 0.403 );
    m_colors[1].push_back( 0 );
}

void HumanPredictionSimulator::loadHumanTrajectory( const motion_t& motion )
{
    m_motion = m_recorder->resample(motion,100);
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
            setMatrixCol( matrix, jn, motion[jn].second );
        }

        likelihood = m_classifier->classify_motion( matrix );
        cout << std::max_element(likelihood.begin(),likelihood.end()) - likelihood.begin() << " ";
    }
    cout << endl;

    return std::max_element(likelihood.begin(),likelihood.end()) - likelihood.begin();
}


bool HumanPredictionSimulator::updateMotion()
{
    cout << "updateMotion, collumns : " << m_current_human_traj.cols() <<  ", motion size : " << m_motion.size() << endl;
    int end = m_current_human_traj.cols() + m_human_increment;

    if( end > int(m_motion.size()) )
    {
        return false;
    }

    m_current_human_traj.resize( 13, end );

    motion_t motion = m_recorder->invertTranslation( m_motion );

    int init_motion = std::max(0.0,double(m_current_human_traj.cols()-2*m_human_increment));
    int end_motion = m_current_human_traj.cols();

    for (int j=init_motion; j<end_motion; j++)
    {
        confPtr_t q = motion[j].second;
        setMatrixCol( m_current_human_traj, j, q );
    }
    //cout << cout << m_current_human_traj << endl;

    setHumanConfig( m_motion[m_current_human_traj.cols()-1].second );
    return true;
}

void HumanPredictionSimulator::predictVoxelOccupancy()
{
    cout << "predict voxel occpancy" << endl;
    std::vector<double> likelihood = m_classifier->classify_motion( m_current_human_traj );
    m_occupacy_grid->setLikelihood( likelihood );
    m_occupacy_grid->computeCurrentOccupancy();
}

void HumanPredictionSimulator::loadGoalConfig()
{
    m_goal_config.clear();

    p3d_rob* rob = static_cast<p3d_rob*>( m_robot->getP3dRobotStruct() );

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

    m_q_start = m_robot->getInitPos();

    m_paths.resize( m_goal_config.size() );
}

Move3D::Trajectory HumanPredictionSimulator::setTrajectoryToMainRobot(const Move3D::Trajectory& traj) const
{
    Move3D::Trajectory traj_tmp(m_robot);

    for(int i=0;i<traj.getNbOfViaPoints();i++)
    {
        confPtr_t q(new Configuration(m_robot));
        q->setConfiguration( traj[i]->getConfigStructCopy() );
        traj_tmp.push_back( q );
    }

    return traj_tmp;
}

void HumanPredictionSimulator::runMultipleStomp( int iter )
{
    Scene* sce = global_Project->getActiveScene();
    std::vector<Robot*> robots;
    robots.push_back( sce->getRobotByName("rob1") );
    robots.push_back( sce->getRobotByName("rob2") );
    if( m_goal_config.size() != robots.size() )
    {
        cout << "Error in multiple stomp run" << endl;
        return;
    }
    std::vector<confPtr_t> init_conf;
    init_conf.push_back( robots[0]->getCurrentPos() );
    init_conf.push_back( robots[1]->getCurrentPos() );

    g3d_draw_allwin_active();

    std::vector<Move3D::Trajectory> stomp_trajs( m_goal_config.size() );
    std::vector<confPtr_t> q_init;

    for(int g=0;(!PlanEnv->getBool(PlanParam::stopPlanner)) && g<int(m_goal_config.size());g++)
    {
        robots[g]->setAndUpdate( *m_q_start );

        if( iter>0 )
        {
            const Move3D::Trajectory& current_traj = m_paths[m_best_path_id];
            const double parameter = m_robot_steps_per_exection*m_robot_step;

            if( g == m_best_path_id )
            {
                stomp_trajs[g] = current_traj.extractSubTrajectory( parameter, current_traj.getRangeMax(), false );
            }
            else
            {
                Move3D::CostOptimization traj( m_paths[g] );
                confPtr_t q_cur = current_traj.configAtParam( parameter );

                if(!traj.connectConfigurationToBegin( q_cur, parameter/5, true ))
                {
                    if(!traj.connectConfigurationToClosestAtBegin( q_cur, parameter/5, false ))
                    {
                        cout << "Error in runMultipleStomp" << endl;
                        return;
                    }
                }
                stomp_trajs[g] = traj;
            }
        }
        else
        {
            cout << "start with straight line for goal : " << g << endl;
            stomp_trajs[g] = Move3D::Trajectory( robots[g] );
            stomp_trajs[g].push_back( m_q_start );
            stomp_trajs[g].push_back( m_goal_config[g] );
        }

        q_init.push_back( stomp_trajs[g].getBegin() );

        if( q_init.size() > 1)
        {
            cout << "Test first config, iteration : " << iter << endl;
            if(!q_init.back()->equal( *q_init[q_init.size()-2] ))
            {
                cout << "Error building the stomp traj" << endl;
            }
        }
    }

    if( !m_is_scenario_init )
    {
        traj_optim_initScenario();
        m_is_scenario_init = true;
    }

    stompRun* pool = new stompRun( traj_optim_get_collision_space(), traj_optim_get_planner_joints(), traj_optim_get_collision_points() );
    pool->setPool( robots );
    pool->start(); // locks the pool

    global_stompRun = pool;

    static_cast<p3d_rob*>( sce->getRobotByName("rob3")->getP3dRobotStruct() )->display_mode = P3D_ROB_NO_DISPLAY;
    static_cast<p3d_rob*>( sce->getRobotByName("rob4")->getP3dRobotStruct() )->display_mode = P3D_ROB_NO_DISPLAY;

    for( int g=0;g<int(stomp_trajs.size()); g++)
    {
        std::vector<double> color = (g == m_best_path_id) ? m_colors[1] : m_colors[0];
        pool->setPathColor( g, color );

        static_cast<p3d_rob*>( robots[g]->getP3dRobotStruct() )->display_mode = P3D_ROB_NO_DISPLAY;
        p3d_col_deactivate_rob_rob( static_cast<p3d_rob*>( robots[g]->getP3dRobotStruct() ), m_robot->getP3dRobotStruct() );

        boost::thread( &stompRun::run, pool, g, stomp_trajs[g] );
    }

    pool->isRunning(); // wait on lock until pool is finished

    for( int g=0;g<int(stomp_trajs.size()); g++)
    {
        m_paths[g] =  pool->getBestTrajectory( g );
        robots[g]->setAndUpdate( *init_conf[g] );
    }

    delete pool;
    global_stompRun = NULL;

//    if( PlanEnv->getBool(PlanParam::drawParallelTraj)) {
//        ENV.setBool(Env::drawTraj,true);
//    }
    cout << "Pool of stomps has ended" << endl;
}

void HumanPredictionSimulator::runParallelStomp( int iter, int id_goal )
{
    cout << "run parallel stomps" << endl;

    PlanEnv->setInt( PlanParam::nb_pointsOnTraj, 100 );
//    traj_optim_set_discretize( true );
//    traj_optim_set_discretization( 0.015 );

    m_robot->setAndUpdate( *m_q_start );

    Move3D::Trajectory stomp_traj;

    if( (iter>0))
    {
        const Move3D::Trajectory& current_traj = m_paths[m_best_path_id];
        const double parameter = m_robot_steps_per_exection*m_robot_step;

        if( id_goal == m_best_path_id )
        {
            stomp_traj = current_traj.extractSubTrajectory( parameter, current_traj.getRangeMax(), false );
        }
        else
        {
            Move3D::CostOptimization traj( m_paths[id_goal] );
            traj.connectConfigurationToBegin( current_traj.configAtParam( parameter ), parameter/5, true );
            stomp_traj = traj;
        }
    }
    else
    {
        stomp_traj.push_back( m_q_start );
        stomp_traj.push_back( m_goal_config[id_goal] );
    }

    if( !m_is_scenario_init )
    {
        traj_optim_initScenario();
        m_is_scenario_init = true;
    }

    std::vector<Robot*> robots; robots.push_back( m_robot );

    stompRun* pool = new stompRun( traj_optim_get_collision_space(),
                                   traj_optim_get_planner_joints(),
                                   traj_optim_get_collision_points() );
    pool->setPool( robots );

    robots.clear();
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob1") );
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob2") );
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob3") );
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob4") );
    pool->setRobotPool( 0, robots );

    for(int i=0;i<int(robots.size());i++)
    {
        static_cast<p3d_rob*>( robots[i]->getP3dRobotStruct() )->display_mode = P3D_ROB_NO_DISPLAY;
        robots[i]->setAndUpdate( *m_q_start );
        p3d_col_deactivate_rob_rob( static_cast<p3d_rob*>( robots[i]->getP3dRobotStruct() ), static_cast<p3d_rob*>( m_robot->getP3dRobotStruct() ) );
    }

    cout << "run pool" << endl;

    pool->run( 0, stomp_traj );
    m_paths[id_goal] = pool->getBestTrajectory( 0 );
    delete pool;
}

void HumanPredictionSimulator::runStandardStomp( int iter, int id_goal  )
{
    PlanEnv->setInt( PlanParam::nb_pointsOnTraj, 100 );
//    traj_optim_set_discretize( true );
//    traj_optim_set_discretization( 0.015 );

    m_robot->setAndUpdate( *m_q_start );

    if( iter>0 )
    {
        const Move3D::Trajectory& current_traj = m_paths[m_best_path_id];
        const double parameter = m_robot_steps_per_exection*m_robot_step;
        Move3D::Trajectory optimi_traj;

        if( id_goal == m_best_path_id )
        {
            optimi_traj = current_traj.extractSubTrajectory( parameter, current_traj.getRangeMax(), false );
        }
        else
        {
            Move3D::CostOptimization traj( m_paths[id_goal] );
            traj.connectConfigurationToBegin( current_traj.configAtParam( parameter ), parameter/5, true );
            optimi_traj = traj;
        }

        traj_optim_set_use_extern_trajectory( true );
        traj_optim_set_extern_trajectory( optimi_traj );
    }
    else
    {
        m_robot->setInitPos( *m_q_start );
        m_robot->setGoalPos( *m_goal_config[id_goal] );
        traj_optim_set_use_extern_trajectory( false );
    }

    traj_optim_set_use_iteration_limit(true);
    traj_optim_set_iteration_limit( PlanEnv->getInt(PlanParam::stompMaxIteration) );
    traj_optim_runStomp(0);

    m_paths[id_goal] = global_optimizer->getBestTraj();
}

int HumanPredictionSimulator::getBestPathId()
{
    std::vector< std::pair<double,int> > cost_sorter( m_paths.size() );

    for(int i=0;i<int(cost_sorter.size());i++)
    {
        Robot* rob = m_paths[i].getRobot();
        confPtr_t q = rob->getCurrentPos();
        m_cost[i].push_back( m_paths[i].cost() );
        cost_sorter[i] = std::make_pair( m_paths[i].cost(), i );
        rob->setAndUpdate(*q);
    }


    std::sort( cost_sorter.begin(), cost_sorter.end() );

    for(int i=0;i<int(cost_sorter.size());i++)
    {
        cout << "path " << cost_sorter[i].second << " cost : " << cost_sorter[i].first << endl;
    }
    return cost_sorter[0].second;
}

void HumanPredictionSimulator::execute(const Move3D::Trajectory& path, bool to_end)
{
    double s = 0.0;

    path.replaceP3dTraj();

    confPtr_t q;

    for( int i=0;
         (!to_end) ?
         ((i<m_robot_steps_per_exection) && (!PlanEnv->getBool(PlanParam::stopPlanner) )) :
         (s<path.getRangeMax() && (!PlanEnv->getBool(PlanParam::stopPlanner)));
         i++ )
    {
        q = path.configAtParam( s ); m_executed_path.push_back( q );
        m_robot->setAndUpdate( *q );
        g3d_draw_allwin_active();
        usleep(25000);
        s += m_robot_step;
    }

    m_q_start = q;
}

void HumanPredictionSimulator::runVoxelOccupancy()
{
    m_human_increment = 1;
//    for(int k=0;k<8;k++) // each class
//    {
     int k = 3;
        for(int i=2;i<3;i++) // 5 first motion
        {
            loadHumanTrajectory( m_recorder->getStoredMotions()[i+25*k] );

            cout << "----------------------------------------------------" << endl;
            cout << "Motion : " << i+25*k << ", size : " << m_motion.size() << endl;
            m_current_human_traj.resize( 13, 0 );

            for(int j=0;(!PlanEnv->getBool(PlanParam::stopPlanner)) && updateMotion();j++)
            {
                predictVoxelOccupancy();
                g3d_draw_allwin_active(); usleep(200000);
            }
        }
//    }
}

void HumanPredictionSimulator::printCosts() const
{
    for(int i =0;i<int(m_cost.size());i++)
    {
        for(int j =0;j<int(m_cost[i].size());j++)
        {
            cout << "cost_" << i << "_(" << j+1 << ") = " <<  m_cost[i][j] << ";" <<endl;
        }
    }
}

double HumanPredictionSimulator::run()
{
    m_current_human_traj.resize( 0, 0 );
    m_executed_path.clear();
    m_best_path_id = -1;

    loadGoalConfig();

    cout << "Load human traj id  : " << GestEnv->getInt(GestParam::human_traj_id) << endl;

    loadHumanTrajectory( m_recorder->getStoredMotions()[GestEnv->getInt(GestParam::human_traj_id)] );

    m_cost.resize( m_goal_config.size() );
    m_robot->setAndUpdate( *m_q_start );

    ENV.setBool( Env::isCostSpace, true );

    for(int i=0;(!PlanEnv->getBool(PlanParam::stopPlanner)) && updateMotion();i++)
    {
        predictVoxelOccupancy();

        g3d_draw_allwin_active();        

        if(!GestEnv->getBool(GestParam::parallelize_stomp) )
        {
            for( int j=0; (!PlanEnv->getBool(PlanParam::stopPlanner)) && j<int(m_goal_config.size()); j++ )
                runStandardStomp( i, j );
        }
        else
        {
            if( GestEnv->getBool(GestParam::with_multiple_stomps))
            {
                runMultipleStomp( i );
            }
            else
            {
                for( int j=0; (!PlanEnv->getBool(PlanParam::stopPlanner)) && j<int(m_goal_config.size()); j++ )
                    runParallelStomp( i, j );
            }
        }

        if( !PlanEnv->getBool(PlanParam::stopPlanner) )
        {
            m_best_path_id = getBestPathId();
            execute( m_paths[m_best_path_id] );
        }
    }

    if( !PlanEnv->getBool(PlanParam::stopPlanner) )
    {
        if( m_best_path_id != -1 )
        {
            const Move3D::Trajectory& traj = m_paths[m_best_path_id];
            const double parameter =  m_robot_steps_per_exection*m_robot_step;
            execute( traj.extractSubTrajectory( parameter, traj.getRangeMax(), false ), true );
        }
    }

    ENV.setBool( Env::isCostSpace, true );

    printCosts();

    cout << "m_executed_path.cost() : " << m_executed_path.cost() << endl;
    m_executed_path.replaceP3dTraj();
    return m_executed_path.cost();
}

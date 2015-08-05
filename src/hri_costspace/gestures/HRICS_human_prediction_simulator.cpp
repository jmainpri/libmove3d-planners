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
#include "HRICS_human_prediction_simulator.hpp"
#include "HRICS_human_prediction_cost_space.hpp"
#include "HRICS_gest_parameters.hpp"
#include "HRICS_openrave_human_map.hpp"

#include "API/project.hpp"
#include "API/Graphic/drawModule.hpp"

#include "planner/cost_space.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planner/TrajectoryOptim/Stomp/run_parallel_stomp.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"

#include "collision_space/collision_space_factory.hpp"

#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Collision-pkg.h>

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

void HRICS_initOccupancyPredictionFramework( std::string robot_name, std::string human_name )
{
    cout << "---------------------------------------------------------" << endl;
    cout << " HRICS_initOccupancyPredictionFramework " << endl;
    cout << "---------------------------------------------------------" << endl;

    std::vector<double> size = global_Project->getActiveScene()->getBounds();

//    std::vector<double> size(6);
//    size[0] = -2;
//    size[1] = 2;
//    size[2] = -2;
//    size[3] = 2;
//    size[4] = -2;
//    size[5] = 2;

    Scene* sce = global_Project->getActiveScene();

    // PR2_ROBOT ... ABBIE
    Robot* robot = sce->getRobotByName( robot_name );

    // HERAKLES_HUMAN1 ... HERAKLES_HUMAN
    Robot* human = sce->getRobotByName( human_name );

    if( robot == NULL || human == NULL )
    {
        cout << "human or robot not defined" << endl;
        return;
    }

    RecordMotion* recorder = new RecordMotion( human );

    if( global_motionRecorders.empty() )
        global_motionRecorders.push_back( recorder );

    std::string foldername;

//    std::string home = std::string( getenv("HOME_MOVE3D" ) ) +
//    std::string( "/../../workspace/gesture-recognition/bkp_recorded_motion/\
//    bkp_good_8_classes/" );

//    std::string home = std::string( getenv("ROS_CATKIN_WS" ) ) +
//            std::string( "/src/NRI-Human-Robot-Collaboration-dataset/\
//wpi_gesture_recognition/first_motions/good_8_classes/" );


    std::string home = std::string( getenv("HOME_MOVE3D" ) ) +
            std::string( "/../assets/Gesture/wpi_gesture_recognition/first_motions/good_8_classes/" );

    foldername = home + std::string("regressed/");
    if( recorder->loadRegressedFromCSV( foldername ) )
    {
        cout << "Motions loaded successfully!!!" << endl;
    }
    else {
         cout << "Error loading motions" << endl;
         delete recorder;
         return;
    }

    ClassifyMotion* classifier = new ClassifyMotion();

    foldername = home + std::string("gmm_data/");
    if( classifier->load_model( foldername ) )
    {
        cout << "GMMs loaded successfully!!!" << endl;
    }
    else {
         cout << "Error loading GMMs" << endl;
         delete recorder;
         delete classifier;
         return;
    }

    WorkspaceOccupancyGrid* occupancyGrid = new WorkspaceOccupancyGrid( human, 0.05, size );

    // Compute occupancy if loaded from library
    bool compute_from_library = false;

    // Set offset value when using openrave
//    if( !human->getUseLibmove3dStruct() ) recorder->setOffsetValue(0,0,0,0);

    // Translate Regressed motions
    recorder->translateStoredMotions(); // REGRESSED MOTIONS ARE NOT TRANSLATED

    // Get recorded motions
    const std::vector<motion_t>& r_motions = recorder->getStoredMotions();
    if( r_motions.empty() )
    {
        cout << "Could not load regressed motions" << endl;
        return;
    }
    occupancyGrid->setRegressedMotions( r_motions, compute_from_library );

    if( compute_from_library )
    {
        if( !occupancyGrid->computeOccpancy() )
        {
            cout << "Could not compute workspace occupancy" << endl;
            return;
        }
        // occupancyGrid->writeToXmlFile("saved_grid.xml");
    }
    else {
        occupancyGrid->loadFromXmlFile( home + "saved_grid.xml" );
        occupancyGrid->set_all_occupied_cells();
    }
    occupancyGrid->setClassToDraw(1);

    // Create the prediction costspace
    HumanPredictionCostSpace* predictionSpace = new HumanPredictionCostSpace( robot, occupancyGrid );

    // Create the simulator
    HumanPredictionSimulator* simulator = new HumanPredictionSimulator( robot, human, recorder, classifier, occupancyGrid );

//    foldername = home + std::string("motions/");
//    recorder->loadXMLFolder( foldername );
//    recorder->translateStoredMotions();
//    recorder->resampleAll( 100 );

    foldername = std::string( home + std::string("openrave_motions") );
    recorder->useOpenRAVEFormat( false );
    recorder->loadCSVFolder( foldername );
//    recorder->invertTranslationStoredMotions();

    // Set the orgin frames (translate the whole scene)

    human->setAndUpdate( *human->getInitPos() );

    // Origin of the motions in the library
    Eigen::Transform3d T_origin(Eigen::Transform3d::Identity());
    T_origin.translation()(0) = -0.115629;
    T_origin.translation()(1) = -1.83979;
    T_origin.translation()(2) =  0.758327;
    T_origin.linear() = Eigen::Matrix3d( Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                                       * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                                       * Eigen::AngleAxisd(1.40382, Eigen::Vector3d::UnitZ()) );

    // Origin of the grid
    Eigen::Transform3d T_draw( recorder->getOffsetTransform().inverse() );
    Eigen::Transform3d T_origin_inv( T_origin.inverse() );
    Eigen::Transform3d T_human( simulator->getHumanPose() );

    occupancyGrid->setDrawingTransform( T_human * T_origin_inv * T_draw );
    simulator->setHumanTransform( T_human * T_origin_inv  );

    // Save to openrave format
//    recorder->saveStoredToCSV( home + std::string("openrave_motions/motion_trajectory"), true );

    // IROS PAPER
    // 28 (25,26,27,28,29) Side by side (class 1) starting in 0
    // 77, Face to face (class 3)
    //cout << "loading trajectory : " << GestEnv->getInt(GestParam::human_traj_id) << endl;
    //simulator->loadHumanTrajectory( recorder->getStoredMotions()[GestEnv->getInt(GestParam::human_traj_id)] );

    // GUI global variables
    global_classifyMotion = classifier;
    global_workspaceOccupancy = occupancyGrid;
    global_humanPredictionCostSpace = predictionSpace;
    global_humanPredictionSimulator = simulator;

    if( global_costSpace != NULL )
    {
        // Define cost functions
        global_costSpace->addCost( "costHumanPredictionOccupancy" , boost::bind(HRICS_getPredictionOccupancyCost, _1) );
        global_costSpace->setCost( "costHumanPredictionOccupancy" );
    }
    else
        cout << "Warning: cost space not initialized" << endl;

    cout << "END INITIALIZING HUMAN PREDICITON SIMULATOR" << endl;
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

    // Set use openrave
    m_use_openrave = !human->getUseLibmove3dStruct();

    // Maps are static, always initilize
    HRICS::set_human_maps();

    // Set human dof map
    m_human_dof_map = m_use_openrave ? herakles_openrave_map : herakles_move3d_map;

    // Set human transform
    m_T_human = Eigen::Transform3d::Identity();
}

void HumanPredictionSimulator::loadHumanTrajectory( const motion_t& motion )
{
    m_motion = m_recorder->resample(motion,100);
}

Eigen::Transform3d HumanPredictionSimulator::getHumanPose()
{
    confPtr_t q_cur = m_human->getCurrentPos();

    Eigen::Transform3d T(Eigen::Transform3d::Identity());

    T.translation()(0) = (*q_cur)[m_human_dof_map["PelvisTransX"]];
    T.translation()(1) = (*q_cur)[m_human_dof_map["PelvisTransY"]];
    T.translation()(2) = (*q_cur)[m_human_dof_map["PelvisTransZ"]];

    // Assignment does not work with rotation()
    T.linear() = Eigen::Matrix3d( Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                                * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd((*q_cur)[m_human_dof_map["PelvisRotZ"]], Eigen::Vector3d::UnitZ()) );

    return T;
}

void HumanPredictionSimulator::setHumanConfig( confPtr_t q )
{
    confPtr_t q_cur = m_human->getCurrentPos();

    Eigen::Transform3d T(Eigen::Transform3d::Identity());

    double x = q->at(6);
    double y = q->at(7);
    double z = q->at(8);
    double r = q->at(11);

    T.translation()(0) = x;
    T.translation()(1) = y;
    T.translation()(2) = z;

    // Assignment does not work with rotation()
    T.linear()  = Eigen::Matrix3d( Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                                 * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                                 * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitZ()) );

    T =  m_T_human * T;

    Eigen::Vector3d angles = T.rotation().eulerAngles(0, 1, 2);

    cout << " trans " << endl;
    cout << T.translation().transpose() << endl;

    cout << " angles " << endl;
    cout << angles.transpose() << endl;

//    cout << " --------- human map -------- " << endl;
//    cout << m_human_dof_map["PelvisTransX"] << endl;
//    cout << m_human_dof_map["PelvisTransY"] << endl;
//    cout << m_human_dof_map["PelvisTransZ"] << endl;
//    cout << m_human_dof_map["PelvisRotZ"]  << endl;
//    cout << m_human_dof_map["TorsoX"]      << endl;
//    cout << m_human_dof_map["TorsoY"]      << endl;
//    cout << m_human_dof_map["TorsoZ"]      << endl;
//    cout << m_human_dof_map["rShoulderX"]   << endl;
//    cout << m_human_dof_map["rShoulderZ"]   << endl;
//    cout << m_human_dof_map["rShoulderY"]  << endl;
//    cout << m_human_dof_map["rArmTrans"]    << endl;
//    cout << m_human_dof_map["rElbowZ"] << endl;

    (*q_cur)[m_human_dof_map["PelvisTransX"]] = T.translation()(0);  // Pelvis
    (*q_cur)[m_human_dof_map["PelvisTransY"]] = T.translation()(1);  // Pelvis
    (*q_cur)[m_human_dof_map["PelvisTransZ"]] = T.translation()(2);  // Pelvis
    (*q_cur)[m_human_dof_map["PelvisRotZ"]]   = angles(2); // Pelvis
    (*q_cur)[m_human_dof_map["TorsoX"]]       = (*q)[12]; // TorsoX
    (*q_cur)[m_human_dof_map["TorsoY"]]       = (*q)[13]; // TorsoY
    (*q_cur)[m_human_dof_map["TorsoZ"]]       = (*q)[14]; // TorsoZ
    (*q_cur)[m_human_dof_map["rShoulderX"]]   = (*q)[18]; // rShoulderX
    (*q_cur)[m_human_dof_map["rShoulderZ"]]   = (*q)[19]; // rShoulderZ
    (*q_cur)[m_human_dof_map["rShoulderY"]]   = (*q)[20]; // rShoulderY
    (*q_cur)[m_human_dof_map["rArmTrans"]]    = (*q)[21]; // rArmTrans
    (*q_cur)[m_human_dof_map["rElbowZ"]]      = (*q)[22]; // rElbowZ

    m_human->setAndUpdate(*q_cur);
}

void HumanPredictionSimulator::setMatrixCol(Eigen::MatrixXd& matrix, int j, confPtr_t q)
{
    matrix(0,j) = j;        // Time
    matrix(1,j) = q->at(6);  // PelvisX
    matrix(2,j) = q->at(7);  // PelvisY
    matrix(3,j) = q->at(8);  // PelvisZ
    matrix(4,j) = q->at(11); // PelvisRotZ
    matrix(5,j) = q->at(12); // TorsoX
    matrix(6,j) = q->at(13); // TorsoY
    matrix(7,j) = q->at(14); // TorsoZ

    matrix(8,j) =  q->at(18);  // rShoulderX
    matrix(9,j) =  q->at(19);  // rShoulderZ
    matrix(10,j) = q->at(20); // rShoulderY
    matrix(11,j) = q->at(21); // rArmTrans
    matrix(12,j) = q->at(22); // rElbowZ
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

    // TODO becarful some motions are not translated
//    motion_t motion = m_recorder->invertTranslation( m_motion );

    int init_motion = std::max(0.0,double(m_current_human_traj.cols()-2*m_human_increment));
    int end_motion = m_current_human_traj.cols();

    for (int j=init_motion; j<end_motion; j++)
    {
        confPtr_t q = m_motion[j].second;
        cout << "q[6] : " << q->at(6) << endl;
        cout << "q[7] : " << q->at(7) << endl;
        cout << "q[8] : " << q->at(8) << endl;
        cout << "q[11] : " << q->at(11) << endl;
        setMatrixCol( m_current_human_traj, j, q );
    }
    //cout << cout << m_current_human_traj << endl;

    setHumanConfig( m_motion[m_current_human_traj.cols()-1].second->copy() ); // TODO copy because... why setandupdate modifies config

    return true;
}

void HumanPredictionSimulator::predictVoxelOccupancy()
{
    cout << "predict voxel occpancy" << endl;
    std::vector<double> likelihood = m_classifier->classify_motion( m_current_human_traj );
    m_occupacy_grid->setLikelihood( likelihood );

    if( m_human->getUseLibmove3dStruct() )
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
                stomp_trajs[g] = current_traj.extractSubTrajectory( parameter, current_traj.getParamMax(), false );
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

        boost::thread( &stompRun::run, pool, g, stomp_trajs[g], 0.0 );
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
            stomp_traj = current_traj.extractSubTrajectory( parameter, current_traj.getParamMax(), false );
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

    pool->run( 0, stomp_traj, 0.0 );
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
            optimi_traj = current_traj.extractSubTrajectory( parameter, current_traj.getParamMax(), false );
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
         (s<path.getParamMax() && (!PlanEnv->getBool(PlanParam::stopPlanner)));
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
    if(  m_recorder->getStoredMotions().empty() ){
        cout << "No recorded motions in : " << __PRETTY_FUNCTION__ << endl;
        return;
    }


    std::string filename = "traj_1.csv";

    m_human_increment = 1;
//    for(int k=0;k<8;k++) // each class
//    {
     int k = 3;
        for(int i=2;i<3;i++) // 5 first motion
        {
            int human_traj_id = i+25*k;

            // TODO, Fix resampling for OpenRAVE
            // Why make copies is necessary???
//            loadHumanTrajectory( m_recorder->getStoredMotions()[human_traj_id] );
            m_motion = m_recorder->getStoredMotions()[human_traj_id];


            m_recorder->saveToCSV( filename, m_motion );

            cout << "----------------------------------------------------" << endl;
            cout << "Motion : " << human_traj_id << ", size : " << m_motion.size() << endl;
            m_current_human_traj.resize( 13, 0 );

            for(int j=0;(!PlanEnv->getBool(PlanParam::stopPlanner)) && updateMotion();j++)
            {
                predictVoxelOccupancy();

                if( m_human->getUseLibmove3dStruct() )
                    g3d_draw_allwin_active();
                else{
                    move3d_draw_clear_handles();
                    m_occupacy_grid->draw();
                }

                usleep(200000);
            }
        }
//    }

        filename = "traj_2.csv";
        m_recorder->saveToCSV( filename, m_motion );
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
            execute( traj.extractSubTrajectory( parameter, traj.getParamMax(), false ), true );
        }
    }

    ENV.setBool( Env::isCostSpace, true );

    printCosts();

    cout << "m_executed_path.cost() : " << m_executed_path.cost() << endl;
    m_executed_path.replaceP3dTraj();
    return m_executed_path.cost();
}

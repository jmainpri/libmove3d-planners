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
#include "HRICS_ioc_sequences.hpp"

#include "HRICS_parameters.hpp"
#include "HRICS_ioc.hpp"
#include "HRICS_human_ioc.hpp"
#include "HRICS_human_cost_space.hpp"
#include "HRICS_human_simulator.hpp"
#include "HRICS_record_motion.hpp"
#include "HRICS_dynamic_time_warping.hpp"

#include "API/project.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "utils/misc_functions.hpp"
#include "utils/NumsAndStrings.hpp"

#include "feature_space/spheres.hpp"
#include "feature_space/squares.hpp"

#include "planner/planEnvironment.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Util-pkg.h>

#include <iomanip>

using namespace Move3D;
using namespace HRICS;
using std::cout;
using std::endl;

static std::string move3d_root("/home/jmainpri/Dropbox/move3d/");

// Folders for sphere (and plannar) type of features
static std::string move3d_demo_folder;
static std::string move3d_traj_folder;
static std::string move3d_tmp_data_folder;

// Folders for human trajs features
static std::string move3d_demo_folder_originals;
static std::string move3d_human_trajs_demo_folder_originals;
static std::string move3d_human_trajs_demo_folder_cut;

static bool original_demos = false;

void ioc_set_sphere_paths()
{
    // Folders for sphere (and plannar) type of features
    move3d_demo_folder = move3d_root + "assets/IOC/TRAJECTORIES/";
    move3d_traj_folder = move3d_root + "move3d-launch/matlab/stomp_trajs_home/per_feature_square/";
    move3d_tmp_data_folder = move3d_root + "move3d-launch/matlab/move3d_tmp_data_home/";
}

void ioc_set_human_paths()
{
    // Folders for human trajs features
    move3d_demo_folder_originals = move3d_root + "assets/Collaboration/TRAJECTORIES/";
    move3d_human_trajs_demo_folder_originals = move3d_root + "assets/Collaboration/TRAJECTORIES/originals/";
    move3d_human_trajs_demo_folder_cut = move3d_root + "assets/Collaboration/TRAJECTORIES/cut_demos/";
    move3d_demo_folder = move3d_human_trajs_demo_folder_cut;
    move3d_traj_folder = move3d_root + "move3d-launch/matlab/stomp_trajs/per_feature_human_traj/";
    move3d_tmp_data_folder =  move3d_root + "move3d-launch/matlab/move3d_tmp_data_human_trajs/";

    if( original_demos )
        move3d_demo_folder = move3d_human_trajs_demo_folder_originals; // Comment to save cut demos using Generate button
}

IocSequences::IocSequences()
{
    cout << __PRETTY_FUNCTION__ << endl;

    features_type_ = no_features;

    if( HriEnv->getBool(HricsParam::init_spheres_cost) )
    {
        ioc_set_sphere_paths();
        features_type_ = spheres;
    }
    else if( HriEnv->getBool(HricsParam::init_human_trajectory_cost) )
    {
        features_type_ = human_trajs;
        ioc_set_human_paths();
    }

    use_human_simulation_demo_ = HriEnv->getBool(HricsParam::ioc_use_simulation_demos);
}

bool IocSequences::run()
{
    cout << "************************************************************" << endl;
    cout << __PRETTY_FUNCTION__ << endl;
    cout << "************************************************************" << endl;

    if( features_type_ == no_features )
    {
        cout << "Error: No feature selected!!!" << endl;
        return false;
    }

    Scene* sce = global_Project->getActiveScene();
    Robot* rob = sce->getActiveRobot();
    if (!rob) {
        cout << "robot not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    cout << "Active robot is : " << rob->getName() << endl;

    confPtr_t q_init( rob->getInitPos() );
    confPtr_t q_goal( rob->getGoalPos() );
    if( *q_init == *q_goal )
    {
        cout << "init equal q_goal in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    if( features_type_ == spheres && global_PlanarCostFct == NULL )
    {
        cout << "global_PlanarCostFct not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    Robot* human1 = sce->getRobotByName( "HERAKLES_HUMAN1" );
    Robot* human2 = sce->getRobotByName( "HERAKLES_HUMAN2" );
    if( features_type_ == human_trajs )
    {
        if( human1 == NULL || human2 == NULL )
        {
            cout << "No humans HERAKLES in the the scene" << endl;
            return false;
        }

        if( global_ht_cost_space == NULL )
        {
            cout << "global_ht_cost_space not initialized in file "
                 << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
            return false;
        }
    }

    // LOAD PARAMETERS FROM SETTING FILE
    int nb_way_points       = HriEnv->getInt(HricsParam::ioc_nb_of_way_points);
    bool single_iteration   = HriEnv->getBool(HricsParam::ioc_single_iteration);
    bool sample_from_file   = HriEnv->getBool(HricsParam::ioc_load_samples_from_file);
    int file_offset         = HriEnv->getInt(HricsParam::ioc_from_file_offset);

    // Modify widget when adding a phase
    phase_                  = (phase_t)HriEnv->getInt(HricsParam::ioc_phase);
    cout << "ioc phase : " << phase_ << endl;

    bool StopRun = false;
    std::vector<Eigen::VectorXd> results;
    int iteration = 0;

    MultiplePlanners planners(rob);
    if( sample_from_file )
        // planners.loadTrajsFromFile( move3d_spheres_traj_folder + "FIRST_TRY" );
        // planners.loadTrajsFromFile( move3d_spheres_traj_folder + "THIRD_TRY" );
        // planners.loadTrajsFromFile( move3d_spheres_traj_folder + "STOMP_LARGE" );
        // planners.loadTrajsFromFile( move3d_spheres_traj_folder + "GENERAL_COSTMAP");
        // planners.loadTrajsFromFile( move3d_spheres_traj_folder + "STOMP_COMBINE" );
        planners.loadTrajsFromFile( move3d_traj_folder + "STOMP_VARIANCE_F1" );
    // planners.loadTrajsFromFile( move3d_spheres_traj_folder + "RANDOM_05" );

    // folder for tmp data
//    std::string move3d_tmp_data_folder;

    // Set feature function
    set_features();

    Eigen::MatrixXd samples( Eigen::MatrixXd::Zero(0,0) );

    if( !single_iteration )
    {
        samples = move3d_load_matrix_from_csv_file( move3d_tmp_data_folder + "samples_tmp.txt" );
        cout << "SAMPLING SEQUENCE : " << samples.row(0) << endl;
    }
    else {
        samples = Eigen::MatrixXd::Zero(1,1);
        samples(0,0) = HriEnv->getInt(HricsParam::ioc_sample_iteration);
    }

    // Main loop
    for(int i=0; i<samples.row(0).size() && !StopRun; i++)
    {
        int nb_samples = samples.row(0)(i);

        cout << "------------------------------" << endl;
        cout << " RUN, NB SAMPLES : " << nb_samples << endl;
        cout << "------------------------------" << endl;

//        TODO remove evalutation
//        if( eval_ != NULL){
//            delete eval_;
//        }
        eval_ = NULL;

        int nb_demos = 1;

        if( HriEnv->getBool(HricsParam::init_spheres_cost) )
        {
            eval_ = new IocEvaluation( rob, nb_demos, nb_samples, nb_way_points,
                                      planners, feature_fct_, active_joints_,
                                      move3d_demo_folder, move3d_traj_folder, move3d_tmp_data_folder );
            eval_->setPlannerType( astar );
        }
        else
        {
            // Human 2 is the planned human, Human 1 is the recorded motion
            eval_ = new HumanIoc( human2, human1, nb_demos, nb_samples, nb_way_points,
                                 planners, feature_fct_, active_joints_,
                                 move3d_demo_folder, move3d_traj_folder, move3d_tmp_data_folder );
            eval_->setPlannerType( stomp );

            if( use_human_simulation_demo_ )
                eval_->setUseContext( true );
        }

        if( eval_ == NULL){
            cout << "Error initilizing ioc evaluation module" << endl;
            return false;
        }

        switch( phase_ )
        {
        case save_feature_and_demo_size:

            cout << "SAVE FEATURES AND DEMO SIZE" << endl;
            eval_->loadDemonstrations();
            cout << "save problem" << endl;
            eval_->saveNbDemoAndNbFeatures();
            break;

        case generate:
        {
            cout << "GENERATE" << endl;

            // global_ht_cost_space->normalizing_by_sampling();

            setGenerationFeatures();

            if( use_human_simulation_demo_ )
            {
                std::vector<Move3D::Trajectory> trajs = global_ht_simulator->getDemoTrajectories();
                std::vector<Move3D::confPtr_t> context = global_ht_simulator->getContext();
                std::vector<int> ids = global_ht_simulator->getDemoIds();
                // trajs.push_back( HRICS::motion_to_traj( global_motionRecorders[0]->getStoredMotions()[0], human2, 60 ) );
                // human1->setAndUpdate( *global_motionRecorders[1]->getStoredMotions()[0][0].second );
                eval_->saveDemoToFile( trajs, ids, context );
            }
            else {
                eval_->generateDemonstrations( nb_demos );
            }

            g3d_draw_allwin_active();
        }
            break;

        case sample:
        {
            cout << "SAMPLE" << endl;

            cout << "global_ht_simulator : " << global_ht_simulator << endl;

            setSamplingFeatures();

//            eval->loadWeightVector();
//            eval->setLearnedWeights();
            if( !eval_->loadDemonstrations() ){
                cout << "ERROR LOADING DEMONSTRATIONS" << endl;
                return false;
            }
            // eval.runLearning();

            cout << "stack info" << endl;
            feature_fct_->printInfo();

            // Set what demo correspond to what sample
            std::vector<int> ids = global_ht_simulator->getDemoIds();
            eval_->setDemoIds( ids );

            if( sample_from_file )
                eval_->runFromFileSampling( file_offset );
            else // cout << "sampling" << endl;
                eval_->runSampling();

            g3d_draw_allwin_active();
        }
            break;

        case compare:
            cout << "COMPARE" << endl;

            setCompareFeatures();

            if( use_human_simulation_demo_ )
            {
                cout << "RUN SIMULATION" << endl;
                eval_->loadWeightVector();
                eval_->setLearnedWeights();
                eval_->setOriginalDemoFolder( move3d_demo_folder_originals );
                eval_->setUseSimulator( true );
            }
            else {
                eval_->setUseSimulator( false );
            }

            results.push_back( eval_->compareDemosAndPlanned() );

            eval_->setUseSimulator( false ); // Set use simulator back to false

            g3d_draw_allwin_active();
            break;

        case run_planner:
            cout << "RUN MULTI-PLANNER" << endl;
            eval_->loadDemonstrations();
            // eval.runPlannerMultipleFeature( 50 ); // 10
            eval_->runPlannerWeightedFeature( 50 ); // 50 * 16 = 800
            StopRun = true;
            break;

        case simulation:
        {
            cout << "RUN SIMULATION" << endl;
            eval_->setUseContext( true );
            eval_->loadWeightVector();
            eval_->setLearnedWeights();
            eval_->loadDemonstrations();

            const std::vector<motion_t>& demos = global_ht_simulator->getDemonstrations();

            // Get sample trajectories around demos
//            std::vector<std::vector<motion_t> > sample_trajs(samples.size());
//            std::vector<std::vector<Move3D::Trajectory> > samples;

//            bool perform_sampling = false;
//            if( perform_sampling )
//            {
//                samples = eval_->runSampling();

//                for( int d=0; d<samples.size(); d++ )
//                {
//                    double duration = motion_duration( demos[d] );

//                    for( int k=0; k<samples[d].size(); k++ )
//                        sample_trajs[d].push_back( traj_to_motion( samples[d][k] , duration) );
//                }
//            }

            global_ht_simulator->setDrawExecution( false );

            std::vector<std::string> active_features_names;
            active_features_names.push_back("SmoothnessAll");
//            active_features_names.push_back("Length");
            active_features_names.push_back("Distance");
//            active_features_names.push_back("Visibility");
//            active_features_names.push_back("Musculoskeletal");

            global_ht_simulator->getCostSpace()->setActiveFeatures( active_features_names );

            std::vector<motion_t> trajs;

            for( int j=0; j<global_ht_simulator->getNumberOfDemos(); j++ )
            {
                global_ht_simulator->setDemonstrationId( j );

                for( int k=0; k<10; k++ )
                {
                    global_ht_simulator->run();

                    std::stringstream ss;
                    ss.str("");
                    ss << "run_simulator_" << std::setw(3) << std::setfill( '0' ) << j;
                    ss <<              "_" << std::setw(3) << std::setfill( '0' ) << k << ".traj";

                    global_ht_simulator->getExecutedPath().saveToFile( ss.str() );

                    trajs.push_back( global_ht_simulator->getExecutedTrajectory() );
                }

                //                cout << "wait for key" << endl;
                //                std::cin.ignore();

//                break;
            }

//            for( int j=0; j<global_ht_simulator->getNumberOfDemos(); j++ )
//            {
//                global_ht_simulator->setDemonstrationId( j );
//                cout << " cost of demonstration : " << j << " " << global_ht_simulator->getCost( demos[j] ) << endl;
//                cout << " cost of executed trajectory : " << j << " " << global_ht_simulator->getCost( trajs[j] ) << endl;
//            }

//            if( perform_sampling )
//            {
//                for( int d=0; d<samples.size(); d++ )
//                {
//                    global_ht_simulator->setDemonstrationId( d );

//                    for( int k=0; k<samples[d].size(); k++ )
//                    {
//                        cout << " cost of sample  for demo : " << d << " , k " << k << " : " << global_ht_simulator->getCost( sample_trajs[d][k] ) << endl;
//                    }
//                }
//            }

//            dtw_compare_performance(100,30);

            global_ht_simulator->setDemonstrations( trajs );

            StopRun = true;
            break;
        }
        case monte_carlo:
            eval_->loadDemonstrations();
            eval_->monteCarloSampling( 10.0, 10 );
            break;

        case default_phase:

            eval_->loadDemonstrations();
            feature_fct_->printInfo();

        default:
            cout << "DEFAULT : LOAD TRAJECTORIES" << endl;
            eval_->loadPlannerTrajectories( 16, 16*i, 0 );
            // StopRun = true;
            break;
        }

        cout << "delete eval" << endl;
        delete eval_;

        if( single_iteration )
            break;

        if ( PlanEnv->getBool(PlanParam::stopPlanner) ) {
            StopRun = true;
        }
    }

    if( !results.empty() )
    {
        Eigen::MatrixXd mat( results.size(), results[0].size() );
        for( int i=0;i<mat.rows();i++)
        {
            mat.row(i) = results[i];
        }

        move3d_save_matrix_to_file( mat, move3d_tmp_data_folder + "result.txt" );
    }

    return true;
}

void IocSequences::set_features()
{
    if( features_type_ == spheres && global_PlanarCostFct != NULL )
    {
        feature_fct_ = new StackedFeatures;

        std::vector<int> aj(1); aj[0] = 1;
        active_joints_ = aj;

        //fct->addFeatureFunction( smoothness_fct_ );

        // TODO becareful with this
//        global_PlanarCostFct->setActiveDoFs( plangroup_->getActiveDofs() );

        if( !feature_fct_->addFeatureFunction( global_PlanarCostFct ) )
        {
            cout << "ERROR : could not add feature function!!!!" << endl;
        }
        else
        {
            feature_fct_->setWeights( global_PlanarCostFct->getWeights() );
            feature_fct_->printInfo();

            cout << "original_vect : " << endl;
            feature_fct_->printWeights();

            // Save costmap to matlab with original weights
            ChronoTimeOfDayOn();

            feature_fct_->setAllFeaturesActive();

//            std::vector<int> active_feature;
//            for( int i=0;i<feature_fct_->getNumberOfFeatures();i++)
//            {
//                // active_feature.clear();
//                active_feature.push_back(i);
//                // feature_fct_->setActiveFeatures( active_feature );
//                // global_PlanarCostFct->produceCostMap(i);
//                // global_PlanarCostFct->produceDerivativeFeatureCostMap(i);
//            }

//            feature_fct_->setActiveFeatures( active_feature );
            // global_PlanarCostFct->produceCostMap(0);
            // global_PlanarCostFct->produceDerivativeFeatureCostMap(0);

            double time;
            ChronoTimeOfDayTimes( &time );
            ChronoTimeOfDayOff();
            cout << "time to compute costmaps : " << time << endl;
        }
    }

    if( features_type_ == human_trajs && global_ht_cost_space != NULL )
    {
        feature_fct_ = global_ht_cost_space;

        // Set all feature active
        feature_fct_->setAllFeaturesActive();

        // Get active joints
        std::vector<Move3D::Joint*> joints = global_ht_simulator->getActiveJoints();
        active_joints_.clear();
        for( size_t i=0; i<joints.size(); i++)
            active_joints_.push_back( joints[i]->getId() );

        cout << "active_joints_.size() : " << active_joints_.size() << endl;
    }
}

void IocSequences::setGenerationFeatures()
{
    if( features_type_ == human_trajs && global_ht_cost_space != NULL )
    {
        std::vector<std::string> active_features;
//        active_features.push_back("Length");
        active_features.push_back("Distance");
        active_features.push_back("SmoothnessAll");
//        active_features.push_back("Collision");
        feature_fct_->setActiveFeatures( active_features );

//        feature_fct_->getFeatureFunction("Length")->setWeights( WeightVect::Ones(1) * 0.0001 );
        feature_fct_->getFeatureFunction("SmoothnessAll")->setWeights( PlanEnv->getDouble(PlanParam::trajOptimSmoothWeight) * WeightVect::Ones(4) );
        feature_fct_->getFeatureFunction("Distance")->setWeights( w_distance_16 );

        cout << "stack info" << endl;
        feature_fct_->printInfo();

        cout << "original_vect : " << endl;
        feature_fct_->printWeights();
    }
}

void IocSequences::setSamplingFeatures()
{
    if( features_type_ == human_trajs && global_ht_cost_space != NULL )
    {
        cout << "Set human features for sampling" << endl;

        std::vector<std::string> active_features;
//        active_features.push_back("Length");
        active_features.push_back("Distance");
        active_features.push_back("SmoothnessAll");
//        active_features.push_back("Visibility");
//        active_features.push_back("Collision");
//        active_features.push_back("Musculoskeletal");

        feature_fct_->setActiveFeatures( active_features );

//        feature_fct_->getFeatureFunction("Smoothness")->setWeights( PlanEnv->getDouble(PlanParam::trajOptimSmoothWeight) * WeightVect::Ones(1) );
        feature_fct_->getFeatureFunction("SmoothnessAll")->setWeights( PlanEnv->getDouble(PlanParam::trajOptimSmoothWeight) * WeightVect::Ones(4) );
        feature_fct_->getFeatureFunction("Distance")->setWeights( w_distance_16 );
//        feature_fct_->getFeatureFunction("Collision")->setWeights( 0.0001 * FeatureVect::Ones(1) );

//        feature_fct_->getFeatureFunction("Length")->setWeights( WeightVect::Ones(1) * 0.001 );
//        feature_fct_->getFeatureFunction("Distance")->setWeights( w_distance_16 );

//        feature_fct_->getFeatureFunction("Length")->setWeights( WeightVect::Ones(1) * 0.7 );
//        feature_fct_->getFeatureFunction("Distance")->setWeights( w_distance_16 );

//        double w_smoo = PlanEnv->getDouble(PlanParam::trajOptimSmoothWeight);
//        double w_obst = PlanEnv->getDouble(PlanParam::trajOptimObstacWeight);
//        double w_dist = PlanEnv->getDouble(PlanParam::trajOptimGlobalWeight);

//        PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,1.0);
//        PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,w_obst);
//        PlanEnv->setDouble(PlanParam::trajOptimGlobalWeight,w_dist);

//        feature_fct_->getFeatureFunction("Smoothness")->setWeights( w_smoo * FeatureVect::Ones(1) );
//        feature_fct_->getFeatureFunction("Collision")->setWeights( w_obst * FeatureVect::Ones(1) );
//        feature_fct_->getFeatureFunction("Distance")->setWeights( w_dist * w_distance_16 );


        cout << "stack info" << endl;
        feature_fct_->printInfo();

        cout << "original_vect : " << endl;
        feature_fct_->printWeights();
    }
}

void IocSequences::setCompareFeatures()
{
    if( features_type_ == human_trajs && global_ht_cost_space != NULL )
    {
        std::vector<std::string> active_features;
//        active_features.push_back("Length");
        active_features.push_back("Smoothness");
        active_features.push_back("Distance");
//        active_features.push_back("Collision");

        feature_fct_->setActiveFeatures( active_features );

//        feature_fct_->getFeatureFunction("Length")->setWeights( WeightVect::Ones(1) * 0.7 );
        feature_fct_->getFeatureFunction("Smoothness")->setWeights( PlanEnv->getDouble(PlanParam::trajOptimSmoothWeight) * WeightVect::Ones(1) );
        feature_fct_->getFeatureFunction("Distance")->setWeights( w_distance_16 );

        cout << "stack info" << endl;
        feature_fct_->printInfo();

        cout << "original_vect : " << endl;
        feature_fct_->printWeights();
    }
}

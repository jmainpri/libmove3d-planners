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

#include "API/project.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "utils/misc_functions.hpp"
#include "utils/NumsAndStrings.hpp"

#include "feature_space/spheres.hpp"
#include "feature_space/squares.hpp"

#include "planner/planEnvironment.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Util-pkg.h>

using namespace Move3D;
using namespace HRICS;
using std::cout;
using std::endl;

// Folders for sphere (and plannar) type of features
static std::string move3d_spheres_demo_folder("/home/jmainpri/Dropbox/move3d/assets/IOC/TRAJECTORIES/");
static std::string move3d_spheres_traj_folder("/home/jmainpri/Dropbox/move3d/move3d-launch/matlab/stomp_trajs_home/per_feature_square/");
static std::string move3d_spheres_tmp_data_folder("/home/jmainpri/Dropbox/move3d/move3d-launch/matlab/move3d_tmp_data_home/");

// Folders for human trajs features
static std::string move3d_human_trajs_demo_folder("/home/jmainpri/Dropbox/move3d/assets/Collaboration/TRAJECTORIES/");
static std::string move3d_human_trajs_traj_folder("/home/jmainpri/Dropbox/move3d/move3d-launch/matlab/stomp_trajs/per_feature_human_traj/");
static std::string move3d_human_trajs_tmp_data_folder("/home/jmainpri/Dropbox/move3d/move3d-launch/matlab/move3d_tmp_data_human_trajs/");

IocSequences::IocSequences()
{
    cout << __PRETTY_FUNCTION__ << endl;

    features_type_ = no_features;

    if(  HriEnv->getBool(HricsParam::init_spheres_cost) )
    {
        features_type_ = spheres;
    }
    else if(  HriEnv->getBool(HricsParam::init_human_trajectory_cost) )
    {
        features_type_ = human_trajs;
    }
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
    int nb_iterations       = HriEnv->getInt(HricsParam::ioc_sample_iteration);
    bool sample_from_file   = HriEnv->getBool(HricsParam::ioc_load_samples_from_file);
    int file_offset         = HriEnv->getInt(HricsParam::ioc_from_file_offset);
    phase_                  = (phase_t)HriEnv->getInt(HricsParam::ioc_phase);

//    phase_ = (phase_t)1;
    cout << "ioc phase : " << phase_ << endl;

    int nb_demos = 1;
    int nb_sampling_phase = 10;
    //    int min_samples = 10;
    //    int max_samples = 100;

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
        planners.loadTrajsFromFile( move3d_spheres_traj_folder + "STOMP_VARIANCE_F1" );
    // planners.loadTrajsFromFile( move3d_spheres_traj_folder + "RANDOM_05" );

    // folder for tmp data
    std::string move3d_tmp_data_folder;

    // Set feature function
    set_features();

    // Main loop
    for(int i=0; i<nb_sampling_phase && !StopRun; i++)
    {
        // iteration = i; // 2, 5, 30, 50

        if( single_iteration )
            iteration = nb_iterations;
        else
            iteration = i;

        cout << "------------------------------" << endl;
        cout << " RUN : " << iteration << endl;
        cout << "------------------------------" << endl;

        // interpolation for the number of sampling phase
        // int nb_samples = min_samples + double(iteration)*(max_samples-min_samples)/double(nb_sampling_phase-1);
        // int nb_samples = (iteration*100+1);
        int nb_samples = (16*(iteration+1));
        // int nb_samples = (160*(iteration+1));

        cout << "NB SAMPLES : " << nb_samples << endl;

        IocEvaluation* eval = NULL;

        if( HriEnv->getBool(HricsParam::init_spheres_cost) )
        {
            move3d_tmp_data_folder = move3d_spheres_tmp_data_folder;

            eval = new IocEvaluation( rob, nb_demos, nb_samples, nb_way_points,
                                      planners, feature_fct_, active_joints_,
                                      move3d_spheres_demo_folder, move3d_spheres_traj_folder, move3d_tmp_data_folder );
            eval->setPlannerType( astar );
        }
        else
        {
            move3d_tmp_data_folder = move3d_human_trajs_tmp_data_folder;

            // Human 2 is the planned human, Human 1 is the recorded motion
            eval = new HumanIoc( human2, human1, nb_demos, nb_samples, nb_way_points,
                                 planners, feature_fct_, active_joints_,
                                 move3d_human_trajs_demo_folder, move3d_human_trajs_traj_folder, move3d_tmp_data_folder );
            eval->setPlannerType( stomp );
        }

        if( eval == NULL){
            cout << "Error initilizing ioc evaluation module" << endl;
            return false;
        }

        switch( phase_ )
        {
        case generate:
            cout << "GENERATE" << endl;
            setGenerationFeatures();
            eval->generateDemonstrations();
            g3d_draw_allwin_active();
            break;

        case sample:
            cout << "SAMPLE" << endl;

            setSamplingFeatures();

            eval->loadWeightVector();
            eval->setLearnedWeights();
            eval->loadDemonstrations();
            // eval.runLearning();

            cout << "stack info" << endl;
            feature_fct_->printInfo();

            if( sample_from_file )
                eval->runFromFileSampling( file_offset );
            else //cout << "sampling" << endl;
                eval->runSampling();

            g3d_draw_allwin_active();
            break;

        case compare:
            cout << "COMPARE" << endl;
            results.push_back( eval->compareDemosAndPlanned() );
            g3d_draw_allwin_active();
            break;

        case run_planner:
            cout << "RUN MULTI-PLANNER" << endl;
            eval->loadDemonstrations();
            // eval.runPlannerMultipleFeature( 50 ); // 10
            eval->runPlannerWeightedFeature( 50 ); // 50 * 16 = 800
            StopRun = true;
            break;

        case monte_carlo:
            eval->loadDemonstrations();
            eval->monteCarloSampling( 10.0, 10 );
            break;

        default:
            cout << "DEFAULT : LOAD TRAJECTORIES" << endl;
            eval->loadPlannerTrajectories( 16, 16*i, 0 );
            // StopRun = true;
            break;
        }

        cout << "delete eval" << endl;
        delete eval;

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

        // Set active joints and joint bounds
        HumanTrajSimulator sim( global_ht_cost_space );
        sim.init();
        active_joints_ = sim.getActiveJoints();
        cout << "active_joints_.size() : " << active_joints_.size() << endl;
    }
}

void IocSequences::setGenerationFeatures()
{
    if( features_type_ == human_trajs && global_ht_cost_space != NULL )
    {
        std::vector<std::string> active_features;
        active_features.push_back("Distance");
//        active_features.push_back("Smoothness");
//        active_features.push_back("Collision");
        feature_fct_->setActiveFeatures( active_features );

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
        std::vector<std::string> active_features;
        active_features.push_back("Distance");
        active_features.push_back("Smoothness");
        active_features.push_back("Collision");

        feature_fct_->setActiveFeatures( active_features );

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

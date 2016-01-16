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

#include "API/Graphic/drawModule.hpp"
#include "API/Graphic/drawCost.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Util-pkg.h>

#include <iomanip>
#include <boost/bind.hpp>
#include <unistd.h>
#include <sys/stat.h>

using namespace Move3D;
using namespace HRICS;
using std::cout;
using std::endl;

extern test_set dataset;

static std::string move3d_root =
    std::string(getenv("HOME_MOVE3D")) + std::string("/../");
// static std::string move3d_root("/home/jmainpri/Dropbox/move3d/");

// Folders for sphere (and plannar) type of features
static std::string move3d_demo_folder;
static std::string move3d_traj_folder;
static std::string move3d_tmp_data_folder;

// Folders for human trajs features
static std::string move3d_demo_folder_originals;
static std::string move3d_human_trajs_demo_folder_originals;
static std::string move3d_human_trajs_demo_folder_cut;

// Folder for temporary files
static std::string move3d_process_folder = std::string(getenv("HOME")) +
                                           std::string("/move3d_tmp_files/") +
                                           num_to_string<int>(getpid()) + "/";
// static std::string move3d_process_folder = "";

static bool original_demos = false;

void ioc_set_sphere_paths() {
  // Folders for sphere (and plannar) type of features
  move3d_demo_folder = move3d_root + "assets/IOC/TRAJECTORIES/";
  move3d_traj_folder =
      move3d_root + "move3d-launch/matlab/stomp_trajs_home/per_feature_square/";
  move3d_tmp_data_folder =
      move3d_root + "move3d-launch/matlab/move3d_tmp_data_home/";
}

void ioc_set_human_paths() {
  // Folders for human trajs features
  move3d_demo_folder_originals =
      move3d_root + "assets/Collaboration/TRAJECTORIES/";
  move3d_human_trajs_demo_folder_originals =
      move3d_root + "assets/Collaboration/TRAJECTORIES/originals/";
  move3d_human_trajs_demo_folder_cut =
      move3d_root + "assets/Collaboration/TRAJECTORIES/cut_demos/";
  move3d_demo_folder = move3d_human_trajs_demo_folder_cut;
  move3d_traj_folder =
      move3d_root + "move3d-launch/matlab/stomp_trajs/per_feature_human_traj/";
  move3d_tmp_data_folder =
      move3d_root + "move3d-launch/matlab/move3d_tmp_data_human_trajs/";

  if (original_demos)  // Comment to save cut demos using Generate button
    move3d_demo_folder = move3d_human_trajs_demo_folder_originals;
}

Move3D::Trajectory load_one_traj(Move3D::Robot* active_human,
                                 std::string folder_name,
                                 int id_demo,
                                 int id_run);

std::vector<std::vector<Move3D::Trajectory> > load_trajs_names(
    std::string folder,
    const std::vector<std::string>& demo_names,
    std::string split,
    Eigen::Vector3d color,
    bool draw);

std::vector<std::vector<Move3D::Trajectory> > load_trajs_names(
    std::string folder,
    const std::vector<std::string>& demo_names,
    int demo_id,
    Eigen::Vector3d color,
    bool draw);

std::vector<std::vector<Move3D::Trajectory> > load_trajs(std::string folder,
                                                         int nb_demos,
                                                         int demo_id,
                                                         Eigen::Vector3d color,
                                                         bool draw);

IocSequences::IocSequences() {
  cout << __PRETTY_FUNCTION__ << endl;

  features_type_ = no_features;

  if (HriEnv->getBool(HricsParam::init_spheres_cost)) {
    ioc_set_sphere_paths();
    features_type_ = spheres;
  } else if (HriEnv->getBool(HricsParam::init_human_trajectory_cost)) {
    features_type_ = human_trajs;
    ioc_set_human_paths();
  }

  use_human_simulation_demo_ =
      HriEnv->getBool(HricsParam::ioc_use_simulation_demos);
}

bool IocSequences::run() {
  cout << "****************************************************" << endl;
  cout << __PRETTY_FUNCTION__ << endl;
  cout << "****************************************************" << endl;

  if (!HriEnv->getBool(HricsParam::ioc_parallel_job)) {
    move3d_process_folder = "";
  }

  if (move3d_process_folder != "") {
    mkdir(move3d_process_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  if (features_type_ == no_features) {
    cout << "Error: No feature selected!!!" << endl;
    return false;
  }

  Scene* sce = global_Project->getActiveScene();
  Robot* rob = sce->getActiveRobot();
  if (!rob) {
    cout << "robot not initialized in file " << __FILE__ << " ,  "
         << __PRETTY_FUNCTION__ << endl;
    return false;
  }

  cout << "Active robot is : " << rob->getName() << endl;

  confPtr_t q_init(rob->getInitPos());
  confPtr_t q_goal(rob->getGoalPos());
  if (*q_init == *q_goal) {
    cout << "init equal q_goal in file " << __FILE__ << " ,  "
         << __PRETTY_FUNCTION__ << endl;
    return false;
  }

  if (features_type_ == spheres && global_PlanarCostFct == NULL) {
    cout << "global_PlanarCostFct not initialized in file " << __FILE__
         << " ,  " << __PRETTY_FUNCTION__ << endl;
    return false;
  }

  Robot* human1 = sce->getRobotByName("HERAKLES_HUMAN1");
  Robot* human2 = sce->getRobotByName("HERAKLES_HUMAN2");
  if (features_type_ == human_trajs) {
    if (human1 == NULL) {
      cout << "No humans HERAKLES_HUMAN1 in the the scene" << endl;

      // Prediction with robot
      human1 = sce->getRobotByName("PR2_ROBOT");
      if (human1 == NULL) {
        cout << "No robot PR2_ROBOT in the the scene" << endl;
        return false;
      } else
        cout << "Set PR2_ROBOT as active agent" << endl;
    }

    if (human2 == NULL) {
      cout << "No active humans HERAKLES_HUMAN2 in the the scene" << endl;

      // Retargeting
      human2 = sce->getRobotByName("PR2_ROBOT");
      if (human2 == NULL) {
        cout << "No robot PR2_ROBOT in the the scene" << endl;
        return false;
      } else
        cout << "Set PR2_ROBOT as active agent" << endl;
    }

    if (global_human_traj_features == NULL) {
      cout << "global_human_traj_features not initialized in file " << __FILE__
           << " ,  " << __PRETTY_FUNCTION__ << endl;
      return false;
    }
  }

  // LOAD PARAMETERS FROM SETTING FILE
  int nb_way_points = HriEnv->getInt(HricsParam::ioc_nb_of_way_points);
  bool single_iteration = HriEnv->getBool(HricsParam::ioc_single_iteration);
  bool sample_from_file =
      HriEnv->getBool(HricsParam::ioc_load_samples_from_file);
  int file_offset = HriEnv->getInt(HricsParam::ioc_from_file_offset);

  // Modify widget when adding a phase
  phase_ = (phase_t)HriEnv->getInt(HricsParam::ioc_phase);
  cout << "ioc phase : " << phase_ << endl;

  // Modify sample ik
  sample_ik_ = (sample_ik_t)HriEnv->getInt(HricsParam::ioc_ik);
  cout << "ioc sample ik : " << sample_ik_ << endl;

  bool StopRun = false;
  std::vector<Eigen::VectorXd> results;
  // int iteration = 0;

  MultiplePlanners planners(rob);
  if (sample_from_file)
    // planners.loadTrajsFromFile( move3d_spheres_traj_folder + "FIRST_TRY" );
    // planners.loadTrajsFromFile( move3d_spheres_traj_folder + "THIRD_TRY" );
    // planners.loadTrajsFromFile( move3d_spheres_traj_folder + "STOMP_LARGE" );
    // planners.loadTrajsFromFile( move3d_spheres_traj_folder +
    // "GENERAL_COSTMAP");
    // planners.loadTrajsFromFile( move3d_spheres_traj_folder + "STOMP_COMBINE"
    // );
    planners.loadTrajsFromFile(move3d_traj_folder + "STOMP_VARIANCE_F1");
  // planners.loadTrajsFromFile( move3d_spheres_traj_folder + "RANDOM_05" );

  // folder for tmp data
  //    std::string move3d_tmp_data_folder;

  // Set feature function
  set_features();

  Eigen::MatrixXd samples(Eigen::MatrixXd::Zero(0, 0));

  if (!single_iteration) {
    samples = move3d_load_matrix_from_csv_file(move3d_tmp_data_folder +
                                               "samples_tmp.txt");
    cout << "SAMPLING SEQUENCE : " << samples.row(0) << endl;
  } else {
    samples = Eigen::MatrixXd::Zero(1, 1);
    samples(0, 0) = HriEnv->getInt(HricsParam::ioc_sample_iteration);
  }
  if (samples.rows() == 0) {
    cout << "error defining the number of samples" << endl;
    return false;
  }

  // Main loop
  for (int i = 0; i < samples.row(0).size() && !StopRun; i++) {
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

    if (HriEnv->getBool(HricsParam::init_spheres_cost)) {
      eval_ = new IocEvaluation(rob,
                                nb_demos,
                                nb_samples,
                                nb_way_points,
                                planners,
                                feature_fct_,
                                active_joints_,
                                move3d_demo_folder,
                                move3d_traj_folder,
                                move3d_tmp_data_folder);
      eval_->setPlannerType(astar);
    } else {
      // Human 2 is the planned human, Human 1 is the recorded motion
      eval_ = new HumanIoc(human2,
                           human1,
                           nb_demos,
                           nb_samples,
                           nb_way_points,
                           planners,
                           feature_fct_,
                           active_joints_,
                           move3d_demo_folder,
                           move3d_traj_folder,
                           move3d_tmp_data_folder);
      eval_->setPlannerType(stomp);

      if (use_human_simulation_demo_) eval_->setUseContext(true);
    }

    if (eval_ == NULL) {
      cout << "Error initilizing ioc evaluation module" << endl;
      return false;
    }

    switch (phase_) {
      case save_feature_and_demo_size:

      {
        std::vector<int> ids;  // demo ids

        if (use_human_simulation_demo_) {
          std::vector<Move3D::Trajectory> trajs =
              global_human_traj_simulator->getDemoTrajectories();
          std::vector<Move3D::confPtr_t> context =
              global_human_traj_simulator->getContext();
          ids = global_human_traj_simulator->getDemoIds();
          std::vector<std::string> demo_names =
              global_human_traj_simulator->getMotionsNames();
          // trajs.push_back( HRICS::motion_to_traj(
          // global_motionRecorders[0]->getStoredMotions()[0], human2, 60 ) );
          // human1->setAndUpdate(
          // *global_motionRecorders[1]->getStoredMotions()[0][0].second );
          eval_->setDemoIds(ids);
          eval_->setDemoNames(demo_names);
          eval_->saveDemoToFile(trajs, context);
        } else {
          eval_->generateDemonstrations(nb_demos);
        }

        cout << "SAVE FEATURES AND DEMO SIZE" << endl;
        eval_->loadDemonstrations(ids);
        cout << "save problem" << endl;
        eval_->saveNbDemoAndNbFeatures();
      } break;

      case generate: {
        cout << "GENERATE" << endl;

        // global_human_traj_features->normalizing_by_sampling();

        setGenerationFeatures();

        if (use_human_simulation_demo_) {
          std::vector<Move3D::Trajectory> trajs =
              global_human_traj_simulator->getDemoTrajectories();
          std::vector<Move3D::confPtr_t> context =
              global_human_traj_simulator->getContext();
          std::vector<int> ids = global_human_traj_simulator->getDemoIds();
          std::vector<std::string> demo_names =
              global_human_traj_simulator->getMotionsNames();
          // trajs.push_back( HRICS::motion_to_traj(
          // global_motionRecorders[0]->getStoredMotions()[0], human2, 60 ) );
          // human1->setAndUpdate(
          // *global_motionRecorders[1]->getStoredMotions()[0][0].second );
          eval_->setDemoIds(ids);
          eval_->setDemoNames(demo_names);
          eval_->saveDemoToFile(trajs, context);
        } else {
          eval_->generateDemonstrations(nb_demos);
        }

        g3d_draw_allwin_active();
      } break;

      case sample: {
        std::vector<int> ids;

        {
          cout << "GENERATE" << endl;

          if (use_human_simulation_demo_) {
            std::vector<Move3D::Trajectory> trajs =
                global_human_traj_simulator->getDemoTrajectories();
            std::vector<Move3D::confPtr_t> context =
                global_human_traj_simulator->getContext();
            ids = global_human_traj_simulator->getDemoIds();
            std::vector<std::string> demo_names =
                global_human_traj_simulator->getMotionsNames();
            // trajs.push_back( HRICS::motion_to_traj(
            // global_motionRecorders[0]->getStoredMotions()[0],
            // human2, 60 ) );
            // human1->setAndUpdate(
            // *global_motionRecorders[1]->getStoredMotions()[0][0].second );

            eval_->setDemoIds(ids);
            eval_->setDemoNames(demo_names);
            eval_->saveDemoToFile(trajs, context);
          } else {
            eval_->generateDemonstrations(nb_demos);
          }

          g3d_draw_allwin_active();
        }

        cout << "SAMPLE" << endl;

        // eval->loadWeightVector();
        // eval->setLearnedWeights();
        if (!eval_->loadDemonstrations(ids)) {
          cout << "ERROR LOADING DEMONSTRATIONS" << endl;
          return false;
        }
        // eval.runLearning();

        cout << "stack info" << endl;
        feature_fct_->printInfo();

        if (sample_from_file)

          eval_->runFromFileSampling(file_offset);

        else  // cout << "sampling" << endl;
        {
          if (sample_ik_ == no_ik) {
            eval_->setSampleGoalSet(false);
            eval_->runSamplingSequence();
          } else if (sample_ik_ == ik_and_traj) {
            eval_->setSampleGoalSet(true);
            eval_->runSamplingSequence();
          } else if (sample_ik_ == only_ik) {
            eval_->setSampleGoalSet(false);
            eval_->runIKSampling();
          }
        }

        g3d_draw_allwin_active();
      } break;

      case compare:
        cout << "COMPARE" << endl;

        setCompareFeatures();

        if (use_human_simulation_demo_) {
          cout << "RUN SIMULATION" << endl;
          eval_->loadWeightVector();
          eval_->setLearnedWeights();
          eval_->setOriginalDemoFolder(move3d_demo_folder_originals);
          eval_->setUseSimulator(true);
        } else {
          eval_->setUseSimulator(false);
        }

        results.push_back(eval_->compareDemosAndPlanned());

        eval_->setUseSimulator(false);  // Set use simulator back to false

        g3d_draw_allwin_active();
        break;

      case run_planner:
        cout << "RUN MULTI-PLANNER" << endl;
        eval_->loadDemonstrations();
        // eval.runPlannerMultipleFeature( 50 ); // 10
        eval_->runPlannerWeightedFeature(50);  // 50 * 16 = 800
        StopRun = true;
        break;

      case simulation: {
        cout << "RUN SIMULATION" << endl;
        eval_->setUseContext(true);

        HRICS::HumanTrajSimulator* simulator = global_human_traj_simulator;

        // eval_->loadWeightVector();
        // eval_->setLearnedWeights();

        if (use_human_simulation_demo_) {
          std::vector<Move3D::Trajectory> trajs =
              simulator->getDemoTrajectories();
          std::vector<Move3D::confPtr_t> context = simulator->getContext();
          std::vector<int> ids = simulator->getDemoIds();
          std::vector<std::string> demo_names = simulator->getMotionsNames();
          // trajs.push_back( HRICS::motion_to_traj(
          // global_motionRecorders[0]->getStoredMotions()[0], human2, 60 ) );
          // human1->setAndUpdate(
          // *global_motionRecorders[1]->getStoredMotions()[0][0].second );
          eval_->setProcessDirectory(move3d_process_folder);
          eval_->setDemoIds(ids);
          eval_->setDemoNames(demo_names);
          eval_->saveDemoToFile(trajs, context);
        }

        cout << "load demonstrations" << endl;
        eval_->loadDemonstrations();

        simulator->setDrawExecution(false);

        std::vector<std::string> active_features_names;

        // BECARFUL (Comment for baseline)
        if (!HriEnv->getBool(HricsParam::ioc_use_baseline))
          active_features_names.push_back("SmoothnessAll");

        active_features_names.push_back("Distance");
        // active_features_names.push_back("Length");
        // active_features_names.push_back("Visibility");
        // active_features_names.push_back("Musculoskeletal");

        cout << "set active features" << endl;
        simulator->getCostSpace()->setActiveFeatures(active_features_names);

        if (HriEnv->getBool(HricsParam::ioc_use_baseline)) {
          WeightVect w(WeightVect::Zero(16));
          PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight, 0.001);

          if (HriEnv->getBool(HricsParam::ioc_conservative_baseline)) {
            w = (5 * WeightVect::Ones(16));
          }
          simulator->getCostSpace()->setWeights(w);
        }

        std::vector<motion_t> trajs;

        std::string split = HriEnv->getString(HricsParam::ioc_traj_split_name);

        if (!simulator->setDemonstrationId(split)) {
          cout << "ERROR COULD NOT FIND SPLIT : " << split << endl;
        } else {
          cout << "DEMO NAME IS : " << split << endl;

          // CASE WHERE WE USE THE IOC WEIGTHT
          if (!HriEnv->getBool(HricsParam::ioc_use_baseline))
          // SET BASELINE HERE
          {
            // Set active features
            simulator->getCostSpace()->clearFeatureStack();
            simulator->getCostSpace()->addFeaturesSmoothness();
            simulator->getCostSpace()->addFeaturesDistance();
            simulator->getCostSpace()->setAllFeaturesActive();

            std::stringstream ss;

            switch (dataset) {
              case icra_paper_sept: {
                /// 1 - CASE FOR LEAVE ONE OUT
                ss << split << "_spheres_weights_700.txt";
                break;
              }
              case icra_paper_feb: {
                /// 2- CASE FOR FEBRUARY
                std::string demo_split = "[2711-2823]";
                ss << demo_split << "_spheres_weights_700.txt";
                break;
              }
              case human_robot_experiment:
              case user_study: {
                /// 3 - CASE FOR 80 / 20
                if (HriEnv->getBool(HricsParam::ioc_no_replanning)) {
                  ss << "user_study_noreplan_spheres_weights_300.txt";
                } else {
                  ss << "user_study_replan_spheres_weights_300.txt";
                }
                break;
              }
            }

            // eval_->loadWeightVector( "single_weight/dynamic/" + ss.str() );
            eval_->loadWeightVector("tmp_weights/" + ss.str());
            eval_->setLearnedWeights();
          }

          simulator->clearLastSimulationMotions();

          for (int k = 0; k < 10; k++) {
            simulator->run();

            std::stringstream ss;
            ss.str("");
            ss << "run_simulator";
            ss << "_" << std::setw(3) << std::setfill('0') << 0;
            ss << "_" << std::setw(3) << std::setfill('0') << k << ".traj";

            // GET EXECUTED PATH
            Move3D::Trajectory traj(
                HriEnv->getBool(HricsParam::ioc_no_replanning)
                    ? simulator->getCurrentPath()
                    : simulator->getExecutedPath());

            std::string folder_param =
                HriEnv->getString(HricsParam::ioc_tmp_traj_folder);

            std::string traj_folder = folder_param == ""
                                          ? move3d_tmp_data_folder + "tmp_trajs"
                                          : folder_param;

            std::string path = traj_folder + "/" + ss.str();
            cout << "SAVE TRAJ TO : " << path << endl;
            cout << " nb of way points : " << traj.getNbOfViaPoints() << endl;
            traj.saveToFile(path);
            trajs.push_back(simulator->getExecutedTrajectory());
          }

          // cout << "wait for key" << endl;
          // std::cin.ignore();
        }

        StopRun = true;
        break;
      }
      case generate_results:
        GenerateResults();
        break;

      case monte_carlo:
        eval_->loadDemonstrations();
        eval_->monteCarloSampling(10.0, 10);
        break;

      case default_phase:

        eval_->loadDemonstrations();
        feature_fct_->printInfo();

      default:
        cout << "DEFAULT : LOAD TRAJECTORIES" << endl;
        eval_->loadPlannerTrajectories(16, 16 * i, 0);
        // StopRun = true;
        break;
    }

    cout << "delete eval : " << eval_ << endl;
    delete eval_;

    if (single_iteration) break;

    if (PlanEnv->getBool(PlanParam::stopPlanner)) {
      StopRun = true;
    }
  }

  if (!results.empty()) {
    Eigen::MatrixXd mat(results.size(), results[0].size());
    for (int i = 0; i < mat.rows(); i++) {
      mat.row(i) = results[i];
    }

    move3d_save_matrix_to_file(mat, move3d_tmp_data_folder + "result.txt");
  }

  return true;
}

void IocSequences::set_features() {
  if (features_type_ == spheres && global_PlanarCostFct != NULL) {
    feature_fct_ = new StackedFeatures;

    std::vector<int> aj(1);
    aj[0] = 1;
    active_joints_ = aj;

    // fct->addFeatureFunction( smoothness_fct_ );

    // TODO becareful with this
    //        global_PlanarCostFct->setActiveDoFs( plangroup_->getActiveDofs()
    //        );

    if (!feature_fct_->addFeatureFunction(global_PlanarCostFct)) {
      cout << "ERROR : could not add feature function!!!!" << endl;
    } else {
      feature_fct_->setWeights(global_PlanarCostFct->getWeights());
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
      //                //
      //                global_PlanarCostFct->produceDerivativeFeatureCostMap(i);
      //            }

      //            feature_fct_->setActiveFeatures( active_feature );
      // global_PlanarCostFct->produceCostMap(0);
      // global_PlanarCostFct->produceDerivativeFeatureCostMap(0);

      double time;
      ChronoTimeOfDayTimes(&time);
      ChronoTimeOfDayOff();
      cout << "time to compute costmaps : " << time << endl;
    }
  }

  if (features_type_ == human_trajs && global_human_traj_features != NULL) {
    feature_fct_ = global_human_traj_features;

    // Set all feature active
    feature_fct_->setAllFeaturesActive();

    // Get active joints
    std::vector<Move3D::Joint*> joints =
        global_human_traj_simulator->getActiveJoints();
    active_joints_.clear();
    for (size_t i = 0; i < joints.size(); i++)
      active_joints_.push_back(joints[i]->getId());

    cout << "active_joints_.size() : " << active_joints_.size() << endl;
  }
}

void IocSequences::setGenerationFeatures() {
  if (features_type_ == human_trajs && global_human_traj_features != NULL) {
    std::vector<std::string> active_features;
    // active_features.push_back("Length");
    active_features.push_back("Distance");
    active_features.push_back("SmoothnessAll");
    // active_features.push_back("Collision");
    feature_fct_->setActiveFeatures(active_features);

    // feature_fct_->getFeatureFunction("Length")->setWeights(
    // WeightVect::Ones(1) * 0.0001 );
    feature_fct_->getFeatureFunction("SmoothnessAll")
        ->setWeights(PlanEnv->getDouble(PlanParam::trajOptimSmoothWeight) *
                     WeightVect::Ones(4));
    feature_fct_->getFeatureFunction("Distance")->setWeights(w_distance_16);

    cout << "stack info" << endl;
    feature_fct_->printInfo();

    cout << "original_vect : " << endl;
    feature_fct_->printWeights();
  }
}

void IocSequences::setSamplingFeatures() {
  if (features_type_ == human_trajs && global_human_traj_features != NULL) {
    cout << "Set human features for sampling" << endl;

    std::vector<std::string> active_features;
    //        active_features.push_back("Length");
    active_features.push_back("Distance");
    active_features.push_back("SmoothnessAll");
    //        active_features.push_back("Visibility");
    //        active_features.push_back("Collision");
    //        active_features.push_back("Musculoskeletal");

    feature_fct_->setActiveFeatures(active_features);

    //        feature_fct_->getFeatureFunction("Smoothness")->setWeights(
    //        PlanEnv->getDouble(PlanParam::trajOptimSmoothWeight) *
    //        WeightVect::Ones(1) );
    feature_fct_->getFeatureFunction("SmoothnessAll")
        ->setWeights(PlanEnv->getDouble(PlanParam::trajOptimSmoothWeight) *
                     WeightVect::Ones(4));
    feature_fct_->getFeatureFunction("Distance")->setWeights(w_distance_16);
    //        feature_fct_->getFeatureFunction("Collision")->setWeights( 0.0001
    //        * FeatureVect::Ones(1) );

    //        feature_fct_->getFeatureFunction("Length")->setWeights(
    //        WeightVect::Ones(1) * 0.001 );
    //        feature_fct_->getFeatureFunction("Distance")->setWeights(
    //        w_distance_16 );

    //        feature_fct_->getFeatureFunction("Length")->setWeights(
    //        WeightVect::Ones(1) * 0.7 );
    //        feature_fct_->getFeatureFunction("Distance")->setWeights(
    //        w_distance_16 );

    //        double w_smoo =
    //        PlanEnv->getDouble(PlanParam::trajOptimSmoothWeight);
    //        double w_obst =
    //        PlanEnv->getDouble(PlanParam::trajOptimObstacWeight);
    //        double w_dist =
    //        PlanEnv->getDouble(PlanParam::trajOptimGlobalWeight);

    //        PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,1.0);
    //        PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,w_obst);
    //        PlanEnv->setDouble(PlanParam::trajOptimGlobalWeight,w_dist);

    //        feature_fct_->getFeatureFunction("Smoothness")->setWeights( w_smoo
    //        * FeatureVect::Ones(1) );
    //        feature_fct_->getFeatureFunction("Collision")->setWeights( w_obst
    //        * FeatureVect::Ones(1) );
    //        feature_fct_->getFeatureFunction("Distance")->setWeights( w_dist *
    //        w_distance_16 );

    cout << "stack info" << endl;
    feature_fct_->printInfo();

    cout << "original_vect : " << endl;
    feature_fct_->printWeights();
  }
}

void IocSequences::setSamplingFeaturesIk() {
  if (features_type_ == human_trajs && global_human_traj_features != NULL) {
    cout << "Set human features for sampling" << endl;

    std::vector<std::string> active_features;
    active_features.push_back("Distance");
    active_features.push_back("Musculoskeletal");

    feature_fct_->setActiveFeatures(active_features);

    feature_fct_->getFeatureFunction("Distance")->setWeights(w_distance_16);
    feature_fct_->getFeatureFunction("Musculoskeletal")
        ->setWeights(w_musculo_03);

    cout << "stack info" << endl;
    feature_fct_->printInfo();

    cout << "original_vect : " << endl;
    feature_fct_->printWeights();
  }
}

void IocSequences::setCompareFeatures() {
  if (features_type_ == human_trajs && global_human_traj_features != NULL) {
    std::vector<std::string> active_features;
    //        active_features.push_back("Length");
    active_features.push_back("Smoothness");
    active_features.push_back("Distance");
    //        active_features.push_back("Collision");

    feature_fct_->setActiveFeatures(active_features);

    //        feature_fct_->getFeatureFunction("Length")->setWeights(
    //        WeightVect::Ones(1) * 0.7 );
    feature_fct_->getFeatureFunction("Smoothness")
        ->setWeights(PlanEnv->getDouble(PlanParam::trajOptimSmoothWeight) *
                     WeightVect::Ones(1));
    feature_fct_->getFeatureFunction("Distance")->setWeights(w_distance_16);

    cout << "stack info" << endl;
    feature_fct_->printInfo();

    cout << "original_vect : " << endl;
    feature_fct_->printWeights();
  }
}

std::vector<Eigen::VectorXd> dtw(ChompPlanningGroup* plangroup,
                                 const Move3D::Trajectory& demo,
                                 const std::vector<Move3D::Trajectory>& trajs,
                                 std::vector<Move3D::Joint*> active_joints,
                                 std::vector<Move3D::Joint*> end_effector) {
  std::vector<Eigen::VectorXd> costs(2);

  std::vector<double> costs_tmp;

  costs_tmp = dtw_compare_performance(plangroup, demo, trajs, active_joints);
  costs[0] = Eigen::VectorXd::Zero(costs_tmp.size());
  for (int k = 0; k < int(costs_tmp.size()); k++) costs[0][k] = costs_tmp[k];

  costs_tmp = dtw_compare_performance(plangroup, demo, trajs, end_effector);
  costs[1] = Eigen::VectorXd::Zero(costs_tmp.size());
  for (int k = 0; k < int(costs_tmp.size()); k++) costs[1][k] = costs_tmp[k];

  return costs;
}

struct ioc_statistics_t {
  ioc_statistics_t(int nb_samples) {
    sum = Eigen::VectorXd::Zero(nb_samples);
    sum_of_sqares = Eigen::VectorXd::Zero(nb_samples);
    max = std::numeric_limits<double>::min();
    min = std::numeric_limits<double>::max();
    stddev = 0.;
    avg = 0.;
  }

  void setMin(double val) {
    if (val < min) min = val;
  }

  void setMax(double val) {
    if (val > max) max = val;
  }

  Eigen::VectorXd sum;
  Eigen::VectorXd sum_of_sqares;
  double min;
  double max;
  double stddev;
  double avg;
};

void dtw_update_stat(const Eigen::VectorXd& values,
                     ioc_statistics_t& dtw_stats) {
  dtw_stats.sum += values;
  dtw_stats.sum_of_sqares += Eigen::VectorXd(values.array() * values.array());
  dtw_stats.setMin(values.minCoeff());
  dtw_stats.setMax(values.maxCoeff());
}

void dtw_compute_average_and_stddev(ioc_statistics_t& dtw_stats,
                                    int nb_data_points) {
  dtw_stats.avg = dtw_stats.sum.sum() / double(nb_data_points);
  double sq_mean = dtw_stats.avg * dtw_stats.avg;
  dtw_stats.stddev = std::sqrt(
      dtw_stats.sum_of_sqares.sum() / double(nb_data_points) - sq_mean);
}

void IocSequences::GenerateResults() {
  cout << " ------------------------------------- " << endl;
  cout << "       GENERATE DTW                    " << endl;
  cout << " ------------------------------------- " << endl;

  std::string home(getenv("HOME"));
  std::string folder("/move3d_tmp/trajectories_process/");

  bool load_replan = false;
  bool compute_dtw = true;

  std::string folder_demos;

  switch (dataset) {
    case icra_paper_sept:
      folder_demos = "loo_trajectories/demos/";
      break;
    case icra_paper_feb:
      folder_demos = "loo_trajectories/first_run_feb_demos/";
      break;
    case user_study:
      folder_demos = "loo_trajectories/testing_userstudy/";
      break;
    case human_robot_experiment:
      folder_demos = "loo_trajectories/human_robot_experiment/";
      break;
  }

  std::string folder_baseline_agressive =
      home + folder + "stomp_baseline_agressive_replan/";
  std::string folder_baseline_conservative =
      home + folder + "stomp_baseline_conservative_replan/";
  std::string folder_recovered = home + folder + "stomp_replan/";

  std::string folder_noreplan_baseline_agressive =
      home + folder + "stomp_baseline_agressive_noreplan/";
  std::string folder_noreplan_baseline_conservative =
      home + folder + "stomp_baseline_conservative_noreplan/";
  std::string folder_noreplan_recovered = home + folder + "stomp_noreplan/";

  // Get demo files and split names

  std::vector<std::string> demos_filenames =
      move3d_get_files_in_folder(folder_demos, "traj");
  for (size_t i = 0; i < demos_filenames.size(); i++)
    cout << demos_filenames[i] << endl;

  //

  HRICS::HumanTrajSimulator* sim = global_human_traj_simulator;

  std::vector<std::string> demo_split_names = sim->getMotionsNames();
  // move3d_get_folders_in_folder( folder_recovered );
  for (size_t i = 0; i < demo_split_names.size(); i++) {
    demo_split_names[i] = demo_split_names[i].substr(0, 11);
    cout << demo_split_names[i] << endl;
  }

  Move3D::Robot* active_human = sim->getActiveHuman();
  Move3D::Robot* passive_human = sim->getPassiveHuman();

  active_human->getP3dRobotStruct()->tcur = NULL;

  std::vector<int> active_joint_id;
  std::vector<Move3D::Joint*> active_joints = sim->getActiveJoints();

  for (int i = 0; i < int(active_joints.size()); i++) {
    active_joint_id.push_back(active_joints[i]->getId());

    cout << "active_joints[" << i << "] : " << active_joints[i]->getName()
         << endl;
  }

  ChompPlanningGroup* plangroup =
      new ChompPlanningGroup(active_human, active_joint_id);

  int nb_demos = demo_split_names.size();
  cout << "nb of demos : " << nb_demos << endl;

  std::string split = HriEnv->getString(HricsParam::ioc_traj_split_name);

  int demo_id =
      std::find(demo_split_names.begin(), demo_split_names.end(), split) -
      demo_split_names.begin();
  // = HriEnv->getInt(HricsParam::ioc_sample_iteration); // TODO change that
  if (demo_id >= nb_demos) {
    demo_id = nb_demos - 1;
  }

  global_linesToDraw.clear();

  if (global_DrawModule) {
    global_DrawModule->addDrawFunction("Draw3DTrajs",
                                       boost::bind(&g3d_draw_3d_lines));
    global_DrawModule->enableDrawFunction("Draw3DTrajs");
  }

  //    std::vector<int> active_dofs;

  //    active joints dof (Pelvis) [0] : 6
  //    active joints dof (TorsoX) [1] : 12
  //    active joints dof (TorsoZ) [2] : 13
  //    active joints dof (TorsoY) [3] : 14
  //    active joints dof (rShoulderTransX) [4] : 18
  //    active joints dof (rShoulderTransY) [5] : 19
  //    active joints dof (rShoulderTransZ) [6] : 20
  //    active joints dof (rShoulderY1) [7] : 21
  //    active joints dof (rShoulderX) [8] : 22
  //    active joints dof (rShoulderY2) [9] : 23
  //    active joints dof (rArmTrans) [10] : 24
  //    active joints dof (rElbowZ) [11] : 25
  //    active joints dof (rElbowX) [12] : 26
  //    active joints dof (rElbowY) [13] : 27
  //    active joints dof (lPoint) [14] : 28
  //    active joints dof (rWristZ) [15] : 29
  //    active joints dof (rWristX) [16] : 30
  //    active joints dof (rWristY) [17] : 31

  //    active_dofs.push_back( 6 );
  //    active_dofs.push_back( 7 );
  //    active_dofs.push_back( 8 );
  //    active_dofs.push_back( 9 );

  //    active_dofs.push_back( 12 );
  //    active_dofs.push_back( 13 );
  //    active_dofs.push_back( 14 );

  //    active_dofs.push_back( 21 );
  //    active_dofs.push_back( 22 );
  //    active_dofs.push_back( 23 );

  //    active_dofs.push_back( 25 );
  //    active_dofs.push_back( 26 );
  //    active_dofs.push_back( 27 );

  //    active_dofs.push_back( 29 );
  //    active_dofs.push_back( 30 );
  //    active_dofs.push_back( 31 );

  //    sim->getActiveDofs();

  if (true || demos_.empty()) {
    for (int d = 0; d < nb_demos; d++)
    //    if( true )
    {
      std::stringstream ss;
      ss.str("");
      ss << "trajectory_human_trajs_" << std::setw(3) << std::setfill('0') << d;
      ss << "_" << std::setw(3) << std::setfill('0') << int(0) << ".traj";

      Move3D::Trajectory traj(active_human);
      traj.loadFromFile(folder_demos + ss.str());

      cout << "loading trajectory : " << folder_demos + ss.str()
           << " nb of waypoints : " << traj.getNbOfViaPoints() << endl;

      traj.setColor(d);

      //            double alpha = double(d)/double(nb_demos);

      if (d == demo_id)
        global_linesToDraw.push_back(std::make_pair(
            Eigen::Vector3d(1, 0, 0),
            traj.getJointPoseTrajectory(active_human->getJoint(45))));

      global_trajToDraw.push_back(traj);
      demos_.push_back(traj);
    }
  }

  //    int nb_runs = 10;

  if (noreplan_recovered_.empty()) {
    bool draw_noreplanning = !HriEnv->getBool(HricsParam::ioc_show_replanning);

    noreplan_baseline_agressive_ =
        load_trajs_names(folder_noreplan_baseline_agressive,
                         demo_split_names,
                         split,
                         Eigen::Vector3d(0, 0, 1),
                         draw_noreplanning);  // Blue
    noreplan_baseline_conservative_ =
        load_trajs_names(folder_noreplan_baseline_conservative,
                         demo_split_names,
                         split,
                         Eigen::Vector3d(1, 1, 0),
                         draw_noreplanning);  // Yellow
    noreplan_recovered_ = load_trajs_names(folder_noreplan_recovered,
                                           demo_split_names,
                                           split,
                                           Eigen::Vector3d(0, 1, 0),
                                           draw_noreplanning);  // Green

    if (load_replan) {
      bool draw_replanning = HriEnv->getBool(HricsParam::ioc_show_replanning);

      baseline_agressive_ = load_trajs_names(folder_baseline_agressive,
                                             demo_split_names,
                                             split,
                                             Eigen::Vector3d(0, 0, 1),
                                             draw_replanning);  // Blue
      baseline_conservative_ = load_trajs_names(folder_baseline_conservative,
                                                demo_split_names,
                                                split,
                                                Eigen::Vector3d(1, 1, 0),
                                                draw_replanning);  // Yellow
      recovered_ = load_trajs_names(folder_recovered,
                                    demo_split_names,
                                    split,
                                    Eigen::Vector3d(0, 1, 0),
                                    draw_replanning);  // Green
    }
  }

  // SET HUMAN CONFIGURAION

  cout << "nb of demos loaded : " << sim->getDemonstrationsPassive().size()
       << endl;
  cout << "demo_id : " << demo_id << endl;
  cout << "size : " << sim->getDemonstrationsPassive()[demo_id].size() << endl;
  cout << "passive name : " << passive_human->getName() << endl;

  active_human->setAndUpdate(*demos_[demo_id].getEnd());
  passive_human->setAndUpdate(
      *sim->getDemonstrationsPassive()[demo_id][0].second);

  cout << "get passive trajectory" << endl;
  // Show trajectory
  // Comment to compute DTW
  // Move3D::Trajectory passive_traj(
  //             HRICS::motion_to_traj(
  //             sim->getDemonstrationsPassive()[demo_id],
  //                                   passive_human ) );
  // Move3D::Trajectory& active_traj = recovered_[demo_id][9];
  // global_linesToDraw.push_back( std::make_pair( Eigen::Vector3d(1, 0, 0),
  // active_traj.getJointPoseTrajectory( active_human->getJoint(45) ) ) );

  // cout << "passive human traj size : " << passive_traj.size() << endl;
  //    qt_showMotion2( active_traj, passive_traj, true );

  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------
  // END VIZUALIZATION
  if (!compute_dtw) return;

  std::vector<Move3D::Joint*> joints;
  joints.push_back(active_human->getJoint(45));

  active_joints.clear();

  active_joints.push_back(
      active_human->getJoint("Pelvis"));  // joint name : Pelvis
  active_joints.push_back(active_human->getJoint("TorsoX"));
  active_joints.push_back(
      active_human->getJoint("rShoulderX"));  // joint name : rShoulderX
  active_joints.push_back(
      active_human->getJoint("rElbowZ"));  // joint name : rElbowZ
  active_joints.push_back(
      active_human->getJoint("rWristX"));  // joint name : rWristX

  int nb_samples = noreplan_baseline_agressive_[0].size();
  ioc_statistics_t stat_baseline_agressive_joint(nb_samples);
  ioc_statistics_t stat_baseline_agressive_task(nb_samples);
  ioc_statistics_t stat_baseline_conservative_joint(nb_samples);
  ioc_statistics_t stat_baseline_conservative_task(nb_samples);
  ioc_statistics_t stat_recovered_joint(nb_samples);
  ioc_statistics_t stat_recovered_task(nb_samples);

  ioc_statistics_t stat_noreplan_baseline_agressive_joint(nb_samples);
  ioc_statistics_t stat_noreplan_baseline_agressive_task(nb_samples);
  ioc_statistics_t stat_noreplan_baseline_conservative_joint(nb_samples);
  ioc_statistics_t stat_noreplan_baseline_conservative_task(nb_samples);
  ioc_statistics_t stat_noreplan_recovered_joint(nb_samples);
  ioc_statistics_t stat_noreplan_recovered_task(nb_samples);

  for (size_t d = 0; d < demos_.size(); d++) {
    std::vector<Eigen::VectorXd> dtw_noreplan_baseline_agressive =
        dtw(plangroup,
            demos_[d],
            noreplan_baseline_agressive_[d],
            active_joints,
            joints);

    dtw_update_stat(dtw_noreplan_baseline_agressive[0],
                    stat_noreplan_baseline_agressive_joint);
    dtw_update_stat(dtw_noreplan_baseline_agressive[1],
                    stat_noreplan_baseline_agressive_task);

    std::vector<Eigen::VectorXd> dtw_noreplan_baseline_conservative =
        dtw(plangroup,
            demos_[d],
            noreplan_baseline_conservative_[d],
            active_joints,
            joints);

    dtw_update_stat(dtw_noreplan_baseline_conservative[0],
                    stat_noreplan_baseline_conservative_joint);
    dtw_update_stat(dtw_noreplan_baseline_conservative[1],
                    stat_noreplan_baseline_conservative_task);

    std::vector<Eigen::VectorXd> dtw_noreplan_recovered = dtw(
        plangroup, demos_[d], noreplan_recovered_[d], active_joints, joints);

    dtw_update_stat(dtw_noreplan_recovered[0], stat_noreplan_recovered_joint);
    dtw_update_stat(dtw_noreplan_recovered[1], stat_noreplan_recovered_task);

    if (load_replan) {
      std::vector<Eigen::VectorXd> dtw_baseline_agressive = dtw(
          plangroup, demos_[d], baseline_agressive_[d], active_joints, joints);

      dtw_update_stat(dtw_baseline_agressive[0], stat_baseline_agressive_joint);
      dtw_update_stat(dtw_baseline_agressive[1], stat_baseline_agressive_task);

      std::vector<Eigen::VectorXd> dtw_baseline_conservative =
          dtw(plangroup,
              demos_[d],
              baseline_conservative_[d],
              active_joints,
              joints);

      dtw_update_stat(dtw_baseline_conservative[0],
                      stat_baseline_conservative_joint);
      dtw_update_stat(dtw_baseline_conservative[1],
                      stat_baseline_conservative_task);

      std::vector<Eigen::VectorXd> dtw_recovered =
          dtw(plangroup, demos_[d], recovered_[d], active_joints, joints);

      dtw_update_stat(dtw_recovered[0], stat_recovered_joint);
      dtw_update_stat(dtw_recovered[1], stat_recovered_task);
    }
  }

  int nb_data_points = nb_demos * nb_samples;

  dtw_compute_average_and_stddev(stat_noreplan_baseline_agressive_joint,
                                 nb_data_points);
  dtw_compute_average_and_stddev(stat_noreplan_baseline_agressive_task,
                                 nb_data_points);
  dtw_compute_average_and_stddev(stat_noreplan_baseline_conservative_joint,
                                 nb_data_points);
  dtw_compute_average_and_stddev(stat_noreplan_baseline_conservative_task,
                                 nb_data_points);
  dtw_compute_average_and_stddev(stat_noreplan_recovered_joint, nb_data_points);
  dtw_compute_average_and_stddev(stat_noreplan_recovered_task, nb_data_points);

  if (load_replan) {
    dtw_compute_average_and_stddev(stat_baseline_agressive_joint,
                                   nb_data_points);
    dtw_compute_average_and_stddev(stat_baseline_agressive_task,
                                   nb_data_points);
    dtw_compute_average_and_stddev(stat_baseline_conservative_joint,
                                   nb_data_points);
    dtw_compute_average_and_stddev(stat_baseline_conservative_task,
                                   nb_data_points);
    dtw_compute_average_and_stddev(stat_recovered_joint, nb_data_points);
    dtw_compute_average_and_stddev(stat_recovered_task, nb_data_points);
  }

  cout << "set human configuration" << endl;

  // SET HUMAN CONFIGURAION
  active_human->setAndUpdate(*demos_[demo_id].getEnd());
  Move3D::confPtr_t q = sim->getDemonstrationsPassive()[demo_id].back().second;
  passive_human->setAndUpdate(*q);

  double b1_00(0);
  double b1_01(0);
  double b1_02(0);
  double b1_03(0);
  double b1_04(0);
  double b1_05(0);
  double b1_06(0);
  double b1_07(0);
  double b1_08(0);
  double b1_09(0);
  double b1_10(0);
  double b1_11(0);
  double b1_12(0);
  double b1_13(0);
  double b1_14(0);
  double b1_15(0);

  double b0_00(0);
  double b0_01(0);
  double b0_02(0);
  double b0_03(0);
  double b0_04(0);
  double b0_05(0);
  double b0_06(0);
  double b0_07(0);
  double b0_08(0);
  double b0_09(0);
  double b0_10(0);
  double b0_11(0);
  double b0_12(0);
  double b0_13(0);
  double b0_14(0);
  double b0_15(0);

  double ioc_00(0);
  double ioc_01(0);
  double ioc_02(0);
  double ioc_03(0);
  double ioc_04(0);
  double ioc_05(0);
  double ioc_06(0);
  double ioc_07(0);
  double ioc_08(0);
  double ioc_09(0);
  double ioc_10(0);
  double ioc_11(0);
  double ioc_12(0);
  double ioc_13(0);
  double ioc_14(0);
  double ioc_15(0);

  b0_08 = stat_noreplan_baseline_agressive_joint.avg;
  b0_09 = stat_noreplan_baseline_agressive_joint.stddev;
  b0_10 = stat_noreplan_baseline_agressive_joint.min;
  b0_11 = stat_noreplan_baseline_agressive_joint.max;

  b0_12 = stat_noreplan_baseline_agressive_task.avg;
  b0_13 = stat_noreplan_baseline_agressive_task.stddev;
  b0_14 = stat_noreplan_baseline_agressive_task.min;
  b0_15 = stat_noreplan_baseline_agressive_task.max;

  if (load_replan) {
    b0_00 = stat_baseline_agressive_joint.avg;
    b0_01 = stat_baseline_agressive_joint.stddev;
    b0_02 = stat_baseline_agressive_joint.min;
    b0_03 = stat_baseline_agressive_joint.max;

    b0_04 = stat_baseline_agressive_task.avg;
    b0_05 = stat_baseline_agressive_task.stddev;
    b0_06 = stat_baseline_agressive_task.min;
    b0_07 = stat_baseline_agressive_task.max;
  }

  b1_08 = stat_noreplan_baseline_conservative_joint.avg;
  b1_09 = stat_noreplan_baseline_conservative_joint.stddev;
  b1_10 = stat_noreplan_baseline_conservative_joint.min;
  b1_11 = stat_noreplan_baseline_conservative_joint.max;

  b1_12 = stat_noreplan_baseline_conservative_task.avg;
  b1_13 = stat_noreplan_baseline_conservative_task.stddev;
  b1_14 = stat_noreplan_baseline_conservative_task.min;
  b1_15 = stat_noreplan_baseline_conservative_task.max;

  if (load_replan) {
    b1_00 = stat_baseline_conservative_joint.avg;
    b1_01 = stat_baseline_conservative_joint.stddev;
    b1_02 = stat_baseline_conservative_joint.min;
    b1_03 = stat_baseline_conservative_joint.max;

    b1_04 = stat_baseline_conservative_task.avg;
    b1_05 = stat_baseline_conservative_task.stddev;
    b1_06 = stat_baseline_conservative_task.min;
    b1_07 = stat_baseline_conservative_task.max;
  }

  ioc_08 = stat_noreplan_recovered_joint.avg;
  ioc_09 = stat_noreplan_recovered_joint.stddev;
  ioc_10 = stat_noreplan_recovered_joint.min;
  ioc_11 = stat_noreplan_recovered_joint.max;

  ioc_12 = stat_noreplan_recovered_task.avg;
  ioc_13 = stat_noreplan_recovered_task.stddev;
  ioc_14 = stat_noreplan_recovered_task.min;
  ioc_15 = stat_noreplan_recovered_task.max;

  if (load_replan) {
    ioc_00 = stat_recovered_joint.avg;
    ioc_01 = stat_recovered_joint.stddev;
    ioc_02 = stat_recovered_joint.min;
    ioc_03 = stat_recovered_joint.max;

    ioc_04 = stat_recovered_task.avg;
    ioc_05 = stat_recovered_task.stddev;
    ioc_06 = stat_recovered_task.min;
    ioc_07 = stat_recovered_task.max;
  }

  // TODO SWITCH B0 and B1 to mach paper

  cout << "LATEX TABLE : " << endl;

  cout << "Method & $\\mu$ & $\\sigma$ & min & max & $\\mu$ & $\\sigma$ & min "
          "& max & $\\mu$ & $\\sigma$ & min & max & $\\mu$ & $\\sigma$ & min & "
          "max \\\\" << endl;

  cout.precision(1);
  cout << "\\hline" << endl;
  cout << "\\textit{baseline 1}  & " << b1_00 << " & " << b1_01 << " & "
       << b1_02 << " & " << b1_03 << " & " << b1_04 << " & " << b1_05 << " & "
       << b1_06 << " & " << b1_07 << " & " << b1_08 << " & " << b1_09 << " & "
       << b1_10 << " & " << b1_11 << " & " << b1_12 << " & " << b1_13 << " & "
       << b1_14 << " & " << b1_15 << " \\\\" << endl;

  cout.precision(1);
  cout << "\\hline" << endl;
  cout << "\\textit{baseline 0} & " << b0_00 << " & " << b0_01 << " & " << b0_02
       << " & " << b0_03 << " & " << b0_04 << " & " << b0_05 << " & " << b0_06
       << " & " << b0_07 << " & " << b0_08 << " & " << b0_09 << " & " << b0_10
       << " & " << b0_11 << " & " << b0_12 << " & " << b0_13 << " & " << b0_14
       << " & " << b0_15 << " \\\\" << endl;

  cout.precision(1);
  cout << "\\hline" << endl;
  cout << "With IOC & " << ioc_00 << " & " << ioc_01 << " & " << ioc_02 << " & "
       << ioc_03 << " & " << ioc_04 << " & " << ioc_05 << " & " << ioc_06
       << " & " << ioc_07 << " & " << ioc_08 << " & " << ioc_09 << " & "
       << ioc_10 << " & " << ioc_11 << " & " << ioc_12 << " & " << ioc_13
       << " & " << ioc_14 << " & " << ioc_15 << " \\\\" << endl;

  //    cout << endl;
  //    cout << " MEAN mean  : "  << stats1[0].mean() << endl;
  //    cout << " MEAN stdev : " << stats1[1].mean() << endl;
  //    cout << " MEAN min   : "   << stats1[2].mean() << endl;
  //    cout << " MEAN max   : "   << stats1[3].mean() << endl;

  //**********************************************************

  //  cout << "Plan param 1 : " << PlanEnv->getBool(PlanParam::starRRT) << endl;
  //  cout << "Plan param 2 : " << PlanEnv->getBool(PlanParam::starRewire) <<
  //  endl;

  // HRICS::printHumanConfig();
  // HRICS::setTenAccessiblePositions();

  //  p3d_set_goal_solution_function( manipulation_get_free_holding_config );
  //  HRICS::setSimulationRobotsTransparent();
  // HRICS_humanCostMaps->loadAgentGrids();

  //  if (HRICS::initShelfScenario())
  //  {
  //    HRICS::execShelfScenario();
  //  }

  //    cout << "Clear traj" << endl;
  //    Move3D::Robot* robot =
  //    global_Project->getActiveScene()->getActiveRobot();
  //    p3d_destroy_traj( robot->getP3dRobotStruct(),
  //    robot->getP3dRobotStruct()->tcur );

  //    if( global_rePlanningEnv != NULL )
  //        global_rePlanningEnv->resetTrajectoriesToDraw();
}

Move3D::Trajectory load_one_traj(Move3D::Robot* active_human,
                                 std::string folder_name,
                                 int id_demo,
                                 int id_run) {
  std::stringstream ss;
  ss.str("");
  ss << "run_simulator_" << std::setw(3) << std::setfill('0') << id_demo << "_"
     << std::setw(3) << std::setfill('0') << id_run << ".traj";

  Move3D::Trajectory traj(active_human);
  traj.loadFromFile(folder_name + ss.str());

  cout << "loading trajectory : " << folder_name + ss.str()
       << " nb of waypoints : " << traj.getNbOfViaPoints() << endl;

  // traj.setColor( 0 );
  // double alpha = double(d)/double(nb_demos);

  return traj;
}

std::vector<std::vector<Move3D::Trajectory> > load_trajs_names(
    std::string folder,
    const std::vector<std::string>& demo_names,
    std::string split,
    Eigen::Vector3d color,
    bool draw) {
  std::vector<std::vector<Move3D::Trajectory> > trajs(demo_names.size());

  int nb_runs = 10;

  Move3D::Robot* active_human = global_human_traj_simulator->getActiveHuman();

  for (size_t d = 0; d < demo_names.size(); d++)  // For all demo names
    for (int k = 0; k < nb_runs; k++)             // For all runs
    {
      trajs[d].push_back(
          load_one_traj(active_human, folder + demo_names[d] + "/", 0, k));

      if (demo_names[d] == split && draw)
        global_linesToDraw.push_back(std::make_pair(
            color,
            trajs[d][k].getJointPoseTrajectory(active_human->getJoint(45))));
    }

  return trajs;
}

std::vector<std::vector<Move3D::Trajectory> > load_trajs_names(
    std::string folder,
    const std::vector<std::string>& demo_names,
    int demo_id,
    Eigen::Vector3d color,
    bool draw) {
  return load_trajs_names(folder, demo_names, demo_names[demo_id], color, draw);
}

std::vector<std::vector<Move3D::Trajectory> > load_trajs(std::string folder,
                                                         int nb_demos,
                                                         int demo_id,
                                                         Eigen::Vector3d color,
                                                         bool draw) {
  std::vector<std::vector<Move3D::Trajectory> > trajs(nb_demos);

  int nb_runs = 10;

  Move3D::Robot* active_human = global_human_traj_simulator->getActiveHuman();

  for (int d = 0; d < nb_demos; d++)
    for (int k = 0; k < nb_runs; k++) {
      trajs[d].push_back(load_one_traj(active_human, folder, d, k));

      if ((d == demo_id) && draw)
        global_linesToDraw.push_back(std::make_pair(
            color,
            trajs[d][k].getJointPoseTrajectory(active_human->getJoint(45))));
    }

  return trajs;
}

std::vector<Move3D::Trajectory> demos;
std::vector<std::vector<Move3D::Trajectory> > baseline;
std::vector<std::vector<Move3D::Trajectory> > recovered;
std::vector<std::vector<Move3D::Trajectory> > noreplan_baseline;
std::vector<std::vector<Move3D::Trajectory> > noreplan_recovered;

void hrics_ioc_compute_results() {
  bool icra_paper_sept = true;

  std::string folder_demos;
  std::string folder_base_line;
  std::string folder_recovered;
  std::string folder_no_replan_base_line;
  std::string folder_no_replan_recovered;

  std::vector<std::string> demo_split_names;

  if (icra_paper_sept) {
    /** PAPER ICRA OCTOBER **/

    folder_demos = "loo_trajectories/demos/";

    //    std::string folder_base_line =
    //    "loo_trajectories/paper_icra_0/baseline/";
    //    std::string folder_recovered =
    //    "loo_trajectories/paper_icra_0/recovered/";
    //    std::string folder_no_replan_base_line =
    //    "loo_trajectories/paper_icra_0/no_replan_baseline/";
    //    std::string folder_no_replan_recovered =
    //    "loo_trajectories/paper_icra_0/no_replan_recovered/";

    demo_split_names.resize(8);

    folder_base_line =
        "loo_trajectories/with_collisions_final/zero_baseline_8/";
    folder_recovered = "loo_trajectories/with_collisions_final/recovered_8/";
    folder_no_replan_base_line =
        "loo_trajectories/with_collisions_final/no_replan_zero_baseline_8/";
    folder_no_replan_recovered =
        "loo_trajectories/with_collisions_final/no_replan_recovered_8/";

    demo_split_names.push_back("[0444-0585]");
    demo_split_names.push_back("[0446-0578]");
    demo_split_names.push_back("[0489-0589]");
    demo_split_names.push_back("[0525-0657]");
    demo_split_names.push_back("[0780-0871]");
    demo_split_names.push_back("[1537-1608]");
    demo_split_names.push_back("[2711-2823]");
  } else {
    folder_demos = "loo_trajectories/first_run_feb_demos//";

    folder_base_line = "loo_trajectories/first_run_feb/replan/baseline0/";
    folder_recovered = "loo_trajectories/first_run_feb/replan/recovered/";
    folder_no_replan_base_line =
        "loo_trajectories/first_run_feb/noreplan/baseline0/";
    folder_no_replan_recovered =
        "loo_trajectories/first_run_feb/noreplan/recovered/";

    demo_split_names.push_back("[0649-0740]");
    demo_split_names.push_back("[1282-1370]");
    demo_split_names.push_back("[1593-1696]");
    demo_split_names.push_back("[1619-1702]");
    demo_split_names.push_back("[1696-1796]");
  }

  Move3D::Robot* active_human = global_human_traj_simulator->getActiveHuman();
  Move3D::Robot* passive_human = global_human_traj_simulator->getPassiveHuman();

  active_human->getP3dRobotStruct()->tcur = NULL;

  std::vector<int> active_joint_id;
  std::vector<Move3D::Joint*> active_joints =
      global_human_traj_simulator->getActiveJoints();
  for (int i = 0; i < int(active_joints.size()); i++) {
    active_joint_id.push_back(active_joints[i]->getId());
    cout << "active_joints[" << i << "] : " << active_joints[i]->getName()
         << endl;
  }

  ChompPlanningGroup* plangroup =
      new ChompPlanningGroup(active_human, active_joint_id);

  int nb_demos = demo_split_names.size();
  int demo_id =
      HriEnv->getInt(HricsParam::ioc_sample_iteration);  // TODO change that
  if (demo_id >= nb_demos) {
    demo_id = nb_demos - 1;
  }

  global_linesToDraw.clear();

  if (global_DrawModule) {
    global_DrawModule->addDrawFunction("Draw3DTrajs",
                                       boost::bind(&g3d_draw_3d_lines));
    global_DrawModule->enableDrawFunction("Draw3DTrajs");
  }

  //    std::vector<int> active_dofs;

  //    active joints dof (Pelvis) [0] : 6
  //    active joints dof (TorsoX) [1] : 12
  //    active joints dof (TorsoZ) [2] : 13
  //    active joints dof (TorsoY) [3] : 14
  //    active joints dof (rShoulderTransX) [4] : 18
  //    active joints dof (rShoulderTransY) [5] : 19
  //    active joints dof (rShoulderTransZ) [6] : 20
  //    active joints dof (rShoulderY1) [7] : 21
  //    active joints dof (rShoulderX) [8] : 22
  //    active joints dof (rShoulderY2) [9] : 23
  //    active joints dof (rArmTrans) [10] : 24
  //    active joints dof (rElbowZ) [11] : 25
  //    active joints dof (rElbowX) [12] : 26
  //    active joints dof (rElbowY) [13] : 27
  //    active joints dof (lPoint) [14] : 28
  //    active joints dof (rWristZ) [15] : 29
  //    active joints dof (rWristX) [16] : 30
  //    active joints dof (rWristY) [17] : 31

  //    active_dofs.push_back( 6 );
  //    active_dofs.push_back( 7 );
  //    active_dofs.push_back( 8 );
  //    active_dofs.push_back( 9 );

  //    active_dofs.push_back( 12 );
  //    active_dofs.push_back( 13 );
  //    active_dofs.push_back( 14 );

  //    active_dofs.push_back( 21 );
  //    active_dofs.push_back( 22 );
  //    active_dofs.push_back( 23 );

  //    active_dofs.push_back( 25 );
  //    active_dofs.push_back( 26 );
  //    active_dofs.push_back( 27 );

  //    active_dofs.push_back( 29 );
  //    active_dofs.push_back( 30 );
  //    active_dofs.push_back( 31 );

  //    global_human_traj_simulator->getActiveDofs();

  if (true || demos.empty()) {
    for (int d = 0; d < nb_demos; d++)
    //    if( true )
    {
      std::stringstream ss;
      ss.str("");
      ss << "trajectory_human_trajs_" << std::setw(3) << std::setfill('0') << d;
      ss << "_" << std::setw(3) << std::setfill('0') << int(0) << ".traj";

      Move3D::Trajectory traj(active_human);
      traj.loadFromFile(folder_demos + ss.str());

      cout << "loading trajectory : " << folder_demos + ss.str()
           << " nb of waypoints : " << traj.getNbOfViaPoints() << endl;

      traj.setColor(d);

      //            double alpha = double(d)/double(nb_demos);

      if (d == demo_id)
        global_linesToDraw.push_back(std::make_pair(
            Eigen::Vector3d(1, 0, 0),
            traj.getJointPoseTrajectory(active_human->getJoint(45))));

      global_trajToDraw.push_back(traj);
      demos.push_back(traj);
    }
  }

  //    int nb_runs = 10;

  if (icra_paper_sept) {
    if (true || baseline.empty()) {
      baseline = load_trajs(
          folder_base_line, nb_demos, demo_id, Eigen::Vector3d(0, 1, 0), false);
      recovered = load_trajs(
          folder_recovered, nb_demos, demo_id, Eigen::Vector3d(0, 0, 1), true);
      noreplan_baseline = load_trajs(folder_no_replan_base_line,
                                     nb_demos,
                                     demo_id,
                                     Eigen::Vector3d(0, 1, 0),
                                     false);
      noreplan_recovered = load_trajs(folder_no_replan_recovered,
                                      nb_demos,
                                      demo_id,
                                      Eigen::Vector3d(0, 1, 0),
                                      true);
    }
  } else {
    if (true || baseline.empty()) {
      baseline = load_trajs_names(folder_base_line,
                                  demo_split_names,
                                  demo_id,
                                  Eigen::Vector3d(0, 1, 0),
                                  false);
      recovered = load_trajs_names(folder_recovered,
                                   demo_split_names,
                                   demo_id,
                                   Eigen::Vector3d(0, 0, 1),
                                   true);
      noreplan_baseline = load_trajs_names(folder_no_replan_base_line,
                                           demo_split_names,
                                           demo_id,
                                           Eigen::Vector3d(0, 1, 0),
                                           false);
      noreplan_recovered = load_trajs_names(folder_no_replan_recovered,
                                            demo_split_names,
                                            demo_id,
                                            Eigen::Vector3d(0, 1, 0),
                                            true);
    }
  }

  // SET HUMAN CONFIGURAION

  cout << "nb of demos loaded : "
       << global_human_traj_simulator->getDemonstrationsPassive().size()
       << endl;
  cout << "demo_id : " << demo_id << endl;
  cout
      << "size : "
      << global_human_traj_simulator->getDemonstrationsPassive()[demo_id].size()
      << endl;

  active_human->setAndUpdate(*demos[demo_id].getEnd());
  passive_human->setAndUpdate(
      *global_human_traj_simulator->getDemonstrationsPassive()[demo_id]
           .back()
           .second);

  // Show trajectory
  // Comment to compute DTW
  Move3D::Trajectory passive_traj(HRICS::motion_to_traj(
      global_human_traj_simulator->getDemonstrationsPassive()[demo_id],
      passive_human));
  // Move3D::Trajectory& active_traj = recovered[demo_id][9];
  // global_linesToDraw.push_back( std::make_pair( Eigen::Vector3d(1, 0, 0),
  // active_traj.getJointPoseTrajectory( active_human->getJoint(45) ) ) );

  cout << "passive human traj size : " << passive_traj.size() << endl;
  //    qt_showMotion2( active_traj, passive_traj, true );
  return;

  std::vector<std::vector<Eigen::VectorXd> > costs(nb_demos);
  for (int d = 0; d < nb_demos; d++) costs[d].resize(8);

  //    std::vector<Eigen::VectorXd> stats1( 4 );
  //    for( int i=0; i<int(stats1.size()); i++ )
  //        stats1[i] = Eigen::VectorXd::Zero( 2 );

  std::vector<Move3D::Joint*> joints;
  joints.push_back(active_human->getJoint(45));

  active_joints.clear();
  active_joints.push_back(
      active_human->getJoint("Pelvis"));  // joint name : Pelvis
  active_joints.push_back(active_human->getJoint("TorsoX"));
  active_joints.push_back(
      active_human->getJoint("rShoulderX"));  // joint name : rShoulderX
  active_joints.push_back(
      active_human->getJoint("rElbowZ"));  // joint name : rElbowZ
  active_joints.push_back(
      active_human->getJoint("rWristX"));  // joint name : rWristX

  for (int d = 0; d < nb_demos; d++) {
    std::vector<double> costs_tmp = dtw_compare_performance(
        plangroup, demos[d], baseline[d], active_joints);
    costs[d][0] = Eigen::VectorXd::Zero(costs_tmp.size());
    for (int k = 0; k < int(costs_tmp.size()); k++)
      costs[d][0][k] = costs_tmp[k];
  }

  for (int d = 0; d < nb_demos; d++) {
    std::vector<double> costs_tmp =
        dtw_compare_performance(plangroup, demos[d], baseline[d], joints);
    costs[d][1] = Eigen::VectorXd::Zero(costs_tmp.size());
    for (int k = 0; k < int(costs_tmp.size()); k++)
      costs[d][1][k] = costs_tmp[k];
  }

  for (int d = 0; d < nb_demos; d++) {
    std::vector<double> costs_tmp = dtw_compare_performance(
        plangroup, demos[d], recovered[d], active_joints);
    costs[d][2] = Eigen::VectorXd::Zero(costs_tmp.size());
    for (int k = 0; k < int(costs_tmp.size()); k++)
      costs[d][2][k] = costs_tmp[k];
  }

  for (int d = 0; d < nb_demos; d++) {
    std::vector<double> costs_tmp =
        dtw_compare_performance(plangroup, demos[d], recovered[d], joints);
    costs[d][3] = Eigen::VectorXd::Zero(costs_tmp.size());
    for (int k = 0; k < int(costs_tmp.size()); k++)
      costs[d][3][k] = costs_tmp[k];
  }

  for (int d = 0; d < nb_demos; d++) {
    std::vector<double> costs_tmp = dtw_compare_performance(
        plangroup, demos[d], noreplan_baseline[d], active_joints);
    costs[d][4] = Eigen::VectorXd::Zero(costs_tmp.size());
    for (int k = 0; k < int(costs_tmp.size()); k++)
      costs[d][4][k] = costs_tmp[k];
  }

  for (int d = 0; d < nb_demos; d++) {
    std::vector<double> costs_tmp = dtw_compare_performance(
        plangroup, demos[d], noreplan_baseline[d], joints);
    costs[d][5] = Eigen::VectorXd::Zero(costs_tmp.size());
    for (int k = 0; k < int(costs_tmp.size()); k++)
      costs[d][5][k] = costs_tmp[k];
  }

  for (int d = 0; d < nb_demos; d++) {
    std::vector<double> costs_tmp = dtw_compare_performance(
        plangroup, demos[d], noreplan_recovered[d], active_joints);
    costs[d][6] = Eigen::VectorXd::Zero(costs_tmp.size());
    for (int k = 0; k < int(costs_tmp.size()); k++)
      costs[d][6][k] = costs_tmp[k];
  }

  for (int d = 0; d < nb_demos; d++) {
    std::vector<double> costs_tmp = dtw_compare_performance(
        plangroup, demos[d], noreplan_recovered[d], joints);
    costs[d][7] = Eigen::VectorXd::Zero(costs_tmp.size());
    for (int k = 0; k < int(costs_tmp.size()); k++)
      costs[d][7][k] = costs_tmp[k];
  }

  // SET HUMAN CONFIGURAION
  active_human->setAndUpdate(*demos[demo_id].getEnd());
  passive_human->setAndUpdate(
      *global_human_traj_simulator->getDemonstrationsPassive()[demo_id]
           .back()
           .second);

  std::vector<std::string> names;
  names.push_back("1 (1.31) & ");
  names.push_back("2 (1.31) & ");
  names.push_back("3 (1.40) & ");
  names.push_back("4 (0.99) & ");
  names.push_back("5 (0.90) & ");
  names.push_back("6 (0.70) & ");
  names.push_back("7 (1.11) & ");
  names.push_back("8 & ");

  std::vector<std::vector<Eigen::VectorXd> > stats(nb_demos);

  for (int d = 0; d < nb_demos; d++) {
    stats[d].resize(8);

    for (int i = 0; i < 8; i++) {
      double mean = costs[d][i].mean();
      double sq_sum = costs[d][i].transpose() * costs[d][i];
      double stdev = std::sqrt(
          sq_sum / double(costs[d][i].size()) -
          (mean * mean));  // Moyenne de carrés moins le carré de la moyenne
      double min = costs[d][i].minCoeff();
      double max = costs[d][i].maxCoeff();

      stats[d][i].resize(4);
      stats[d][i][0] = mean;
      stats[d][i][1] = stdev;
      stats[d][i][2] = min;
      stats[d][i][3] = max;
    }

    // BASELINE
    //        cout << stats[d][0][0] << "  " << stats[d][0][1] << "  " <<
    //        stats[d][0][2] << "  " << stats[d][0][3] << "  " ;
    //        cout << stats[d][1][0] << "  " << stats[d][1][1] << "  " <<
    //        stats[d][1][2] << "  " << stats[d][1][3] << "  " ;

    //        cout << stats[d][4][0] << "  " << stats[d][4][1] << "  " <<
    //        stats[d][4][2] << "  " << stats[d][4][3] << "  " ;
    //        cout << stats[d][5][0] << "  " << stats[d][5][1] << "  " <<
    //        stats[d][5][2] << "  " << stats[d][5][3] << "  " << endl;

    // RECOVERED
    cout << stats[d][2][0] << "  " << stats[d][2][1] << "  " << stats[d][2][2]
         << "  " << stats[d][2][3] << "  ";
    cout << stats[d][3][0] << "  " << stats[d][3][1] << "  " << stats[d][3][2]
         << "  " << stats[d][3][3] << "  ";

    cout << stats[d][6][0] << "  " << stats[d][6][1] << "  " << stats[d][6][2]
         << "  " << stats[d][6][3] << "  ";
    cout << stats[d][7][0] << "  " << stats[d][7][1] << "  " << stats[d][7][2]
         << "  " << stats[d][7][3] << "  " << endl;
  }

  cout << endl;

  for (int d = 0; d < nb_demos; d++) {
    stats[d].resize(8);

    for (int i = 0; i < 8; i++) {
      double mean = costs[d][i].mean();
      double sq_sum = costs[d][i].transpose() * costs[d][i];
      double stdev = std::sqrt(
          sq_sum / double(costs[d][i].size()) -
          (mean * mean));  // Moyenne de carrés moins le carré de la moyenne
      double min = costs[d][i].minCoeff();
      double max = costs[d][i].maxCoeff();

      stats[d][i].resize(4);
      stats[d][i][0] = mean;
      stats[d][i][1] = stdev;
      stats[d][i][2] = min;
      stats[d][i][3] = max;
    }

    cout << names[d];

    // BASELINE
    //        cout << stats[d][0][0] << " & " << stats[d][0][1] << " & " <<
    //        stats[d][0][2] << " & " << stats[d][0][3] << " & " ;
    //        cout << stats[d][1][0] << " & " << stats[d][1][1] << " & " <<
    //        stats[d][1][2] << " & " << stats[d][1][3] << " & " ;

    //        cout << stats[d][4][0] << " & " << stats[d][4][1] << " & " <<
    //        stats[d][4][2] << " & " << stats[d][4][3] << " & " ;
    //        cout << stats[d][5][0] << " & " << stats[d][5][1] << " & " <<
    //        stats[d][5][2] << " & " << stats[d][5][3] << " \\" << endl;

    // RECOVERED
    cout << stats[d][2][0] << " & " << stats[d][2][1] << " & " << stats[d][2][2]
         << " & " << stats[d][2][3] << " & ";
    cout << stats[d][3][0] << " & " << stats[d][3][1] << " & " << stats[d][3][2]
         << " & " << stats[d][3][3] << " & ";

    cout << stats[d][6][0] << " & " << stats[d][6][1] << " & " << stats[d][6][2]
         << " & " << stats[d][6][3] << " & ";
    cout << stats[d][7][0] << " & " << stats[d][7][1] << " & " << stats[d][7][2]
         << " & " << stats[d][7][3] << "  " << endl;
  }

  double mean = costs[demo_id][0].mean();
  double sq_sum = costs[demo_id][0].transpose() * costs[demo_id][0];
  double stdev = std::sqrt(
      sq_sum / double(costs[demo_id][0].size()) -
      (mean * mean));  // Moyenne de carrés moins le carré de la moyenne
  double min = costs[demo_id][0].minCoeff();
  double max = costs[demo_id][0].maxCoeff();

  double mean1 = costs[demo_id][1].mean();
  double sq_sum1 = costs[demo_id][1].transpose() * costs[demo_id][1];
  double stdev1 = std::sqrt(
      sq_sum1 / double(costs[demo_id][1].size()) -
      (mean1 * mean1));  // Moyenne de carrés moins le carré de la moyenne
  double min1 = costs[demo_id][1].minCoeff();
  double max1 = costs[demo_id][1].maxCoeff();

  cout << "replanning baseline joint / task " << endl;
  cout << mean << " & " << stdev << " & " << min << " & " << max << " &  "
       << mean1 << " & " << stdev1 << " & " << min1 << " & " << max1 << endl;

  mean = costs[demo_id][2].mean();
  sq_sum = costs[demo_id][2].transpose() * costs[demo_id][2];
  stdev = std::sqrt(
      sq_sum / double(costs[demo_id][2].size()) -
      (mean * mean));  // Moyenne de carrés moins le carré de la moyenne
  min = costs[demo_id][2].minCoeff();
  max = costs[demo_id][2].maxCoeff();

  mean1 = costs[demo_id][3].mean();
  sq_sum1 = costs[demo_id][3].transpose() * costs[demo_id][3];
  stdev1 = std::sqrt(
      sq_sum1 / double(costs[demo_id][3].size()) -
      (mean1 * mean1));  // Moyenne de carrés moins le carré de la moyenne
  min1 = costs[demo_id][3].minCoeff();
  max1 = costs[demo_id][3].maxCoeff();
  cout << "replanning recovered joint / task " << endl;
  cout << mean << " & " << stdev << " & " << min << " & " << max << " &  "
       << mean1 << " & " << stdev1 << " & " << min1 << " & " << max1 << endl;

  mean = costs[demo_id][4].mean();
  sq_sum = costs[demo_id][4].transpose() * costs[demo_id][4];
  stdev = std::sqrt(
      sq_sum / double(costs[demo_id][4].size()) -
      (mean * mean));  // Moyenne de carrés moins le carré de la moyenne
  min = costs[demo_id][4].minCoeff();
  max = costs[demo_id][4].maxCoeff();

  mean1 = costs[demo_id][5].mean();
  sq_sum1 = costs[demo_id][5].transpose() * costs[demo_id][5];
  stdev1 = std::sqrt(
      sq_sum1 / double(costs[demo_id][5].size()) -
      (mean1 * mean1));  // Moyenne de carrés moins le carré de la moyenne
  min1 = costs[demo_id][5].minCoeff();
  max1 = costs[demo_id][5].maxCoeff();

  cout << "one iteration baseline joint / task " << endl;
  cout << mean << " & " << stdev << " & " << min << " & " << max << " &  "
       << mean1 << " & " << stdev1 << " & " << min1 << " & " << max1 << endl;

  mean = costs[demo_id][6].mean();
  sq_sum = costs[demo_id][6].transpose() * costs[demo_id][6];
  stdev = std::sqrt(
      sq_sum / double(costs[demo_id][6].size()) -
      (mean * mean));  // Moyenne de carrés moins le carré de la moyenne
  min = costs[demo_id][6].minCoeff();
  max = costs[demo_id][6].maxCoeff();

  mean1 = costs[demo_id][7].mean();
  sq_sum1 = costs[demo_id][7].transpose() * costs[demo_id][7];
  stdev1 = std::sqrt(
      sq_sum1 / double(costs[demo_id][7].size()) -
      (mean1 * mean1));  // Moyenne de carrés moins le carré de la moyenne
  min1 = costs[demo_id][7].minCoeff();
  max1 = costs[demo_id][7].maxCoeff();
  cout << "one iteration recovered joint / task " << endl;
  cout << mean << " & " << stdev << " & " << min << " & " << max << " &  "
       << mean1 << " & " << stdev1 << " & " << min1 << " & " << max1 << endl;

  //    cout << endl;
  //    cout << " MEAN mean  : "  << stats1[0].mean() << endl;
  //    cout << " MEAN stdev : " << stats1[1].mean() << endl;
  //    cout << " MEAN min   : "   << stats1[2].mean() << endl;
  //    cout << " MEAN max   : "   << stats1[3].mean() << endl;

  //**********************************************************

  //  cout << "Plan param 1 : " << PlanEnv->getBool(PlanParam::starRRT) << endl;
  //  cout << "Plan param 2 : " << PlanEnv->getBool(PlanParam::starRewire) <<
  //  endl;

  // HRICS::printHumanConfig();
  // HRICS::setTenAccessiblePositions();

  //  p3d_set_goal_solution_function( manipulation_get_free_holding_config );
  //  HRICS::setSimulationRobotsTransparent();
  // HRICS_humanCostMaps->loadAgentGrids();

  //  if (HRICS::initShelfScenario())
  //  {
  //    HRICS::execShelfScenario();
  //  }

  //    cout << "Clear traj" << endl;
  //    Move3D::Robot* robot =
  //    global_Project->getActiveScene()->getActiveRobot();
  //    p3d_destroy_traj( robot->getP3dRobotStruct(),
  //    robot->getP3dRobotStruct()->tcur );

  //    if( global_rePlanningEnv != NULL )
  //        global_rePlanningEnv->resetTrajectoriesToDraw();
}

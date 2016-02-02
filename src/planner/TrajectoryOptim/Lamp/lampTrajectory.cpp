#include "lampTrajectory.hpp"

#include "API/project.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "API/Graphic/drawCost.hpp"
#include "API/Graphic/drawModule.hpp"

#include "utils/misc_functions.hpp"
#include "utils/NumsAndStrings.hpp"

#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planner/AStar/AStarPlanner.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"
#include "planner/cost_space.hpp"

#include "collision_space/collision_space_factory.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Util-pkg.h>

#include <Eigen/Eigenvalues>

#include <iomanip>
#include <sstream>
#include <fstream>

using namespace Move3D;
using std::cout;
using std::endl;
using std::cin;

void lamp_sample_trajectories() {
  Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();

  // Warning leaking
  //    Move3D::AStarPlanner* planner = new Move3D::AStarPlanner(robot);
  //    planner->set_pace( PlanEnv->getDouble(PlanParam::grid_pace) );
  //    planner->init();

  //    Move3D::confPtr_t q_init = robot->getInitPos();
  //    Move3D::confPtr_t q_goal = robot->getGoalPos();
  //    Move3D::Trajectory* traj = planner->computeRobotTrajectory( q_init,
  //    q_goal );

  /* TODO planar grid

  std::vector<double> env_size = global_Project->getActiveScene()->getBounds();
  env_size.resize(4);
  cout << "pace : " << pace_ << " meters" << endl;
  grid = new PlanGrid( robot,  pace, env_size );

  if( API_activeGrid != NULL )
      delete API_activeGrid;
  API_activeGrid = grid;

  */

  //    Move3D::Trajectory traj( robot );
  //    traj.loadFromFile(
  //    "/jim_local/Dropbox/move3d/move3d-launch/launch_files/3d_traj.txt" );
  //    robot->setCurrentMove3DTraj( traj );

  Move3D::Trajectory traj(robot);
  traj.push_back(robot->getInitPos());
  traj.push_back(robot->getGoalPos());
  robot->setCurrentMove3DTraj(traj);

  //    LampSampler sampler( robot );
  //    sampler.initialize();
  //    sampler.sampleTrajectories( 15, traj );
}

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------

LampSampler::LampSampler(int num_var_free, int num_dofs)
    : num_vars_free_(num_var_free), num_dofs_(num_dofs) {}

void LampSampler::initialize(const std::vector<double>& derivative_costs,
                             int nb_points) {
  cout << "initialize sampler" << endl;

  std::vector<int> joints;

  // Initializae scenario
  if (robot_model_->getName().find("PR2_ROBOT") != std::string::npos) {
    traj_optim_init_collision_spaces(traj_optim::Shelf, robot_model_);
    joints = traj_optim_get_planner_joints();
  } else {
    joints = robot_model_->getActiveJointsIds();
    // joints.push_back(1);
  }

  // Initialize planning group
  planning_group_ = new ChompPlanningGroup(robot_model_, joints);
  planning_group_->collision_points_ = traj_optim_get_collision_points();

  num_vars_free_ = nb_points;
  num_dofs_ = planning_group_->chomp_dofs_.size();

  cout << "num_vars_free_ : " << num_vars_free_ << endl;
  cout << "num_dofs_ : " << num_dofs_ << endl;

  policy_.setPrintDebug(false);

  // initializes the policy
  policy_.initialize(num_vars_free_,
                     num_dofs_,
                     1.0,
                     0.0,
                     derivative_costs,
                     false,
                     planning_group_);
  policy_.getControlCosts(control_costs_);

  tmp_noise_ = Eigen::VectorXd::Zero(num_vars_free_ * num_dofs_);

  min_eigen_value_ = std::numeric_limits<double>::max();
  max_eigen_value_ =
      std::numeric_limits<double>::min();  // WARNING THIS is 0 ...

  preAllocateMultivariateGaussianSampler();
}

void LampSampler::setOneDofBlockOfPrecisionMatrix(int dof,
                                                  Eigen::MatrixXd matrix) {
  for (int i = 0; i < matrix.rows(); i++)
    for (int j = 0; j < matrix.cols(); j++) {
      if ((i * num_dofs_ + dof) < precision_.rows() &&
          (j * num_dofs_ + dof) < precision_.cols()) {
        precision_(i * num_dofs_ + dof, j * num_dofs_ + dof) += matrix(i, j);
      }
    }
}

void LampSampler::setTimeStepPrecisionMatrix(int time_step,
                                             Eigen::MatrixXd matrix) {
  if ((num_dofs_ == matrix.rows() && num_dofs_ == matrix.cols())) {
    precision_.block(
        time_step * num_dofs_, time_step * num_dofs_, num_dofs_, num_dofs_) +=
        matrix;
    // cout << "Add for time step : " << time_step << endl;
  }
  //    else
  //    {
  //        cout << "hessian (" << matrix.rows() << " , " << matrix.cols() <<
  //        ")" << endl;
  //    }
}

Eigen::MatrixXd LampSampler::getOneDofBlockOfPrecisionMatrix(int dof) {
  Eigen::MatrixXd matrix(num_vars_free_, num_vars_free_);

  for (int i = 0; i < matrix.rows(); i++)
    for (int j = 0; j < matrix.cols(); j++) {
      matrix(i, j) = precision_(i + dof, j + num_dofs_ * j);
    }

  return matrix;
}

Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>
LampSampler::getConfigurationBlockOfPrecisionMatrix(int var) {
  return precision_.block(
      var * num_dofs_, var * num_dofs_, num_dofs_, num_dofs_);
}

bool LampSampler::preAllocateMultivariateGaussianSampler() {
  // invert the control costs, initialize noise generators:
  // inv_control_costs_.clear();
  //    noise_generators_.clear();

  // inv_control_costs_.push_back( control_costs_[j].inverse() );

  // move3d_save_matrix_to_file( inv_control_costs_[0],
  // "../matlab/invcost_matrix.txt" );

  // Uncomment to print the precision Matrix
  // cout << endl << control_costs_[j] << endl;

  precision_ = Eigen::MatrixXd::Zero(num_vars_free_ * num_dofs_,
                                     num_vars_free_ * num_dofs_);

  for (int j = 0; j < num_dofs_; j++) {
    setOneDofBlockOfPrecisionMatrix(j, control_costs_[j]);
  }

  control_ = precision_;

  // Get covariance matrix
  double sigma(std::pow(PlanEnv->getDouble(PlanParam::trajOptimStdDev), 2));
  covariance_ = sigma * precision_.inverse();

  // TODO see of the noise generator needs to be var free or var all
  MultivariateGaussian mvg(Eigen::VectorXd::Zero(num_vars_free_ * num_dofs_),
                           covariance_);
  noise_generators_.clear();
  noise_generators_.push_back(mvg);

  //    covariance_ = precision_.inverse();

  //    cout << "control cost : " << endl << control_costs_[0] << endl;
  //    cout << "precision_ : " << endl << precision_<< endl;

  //    move3d_save_matrix_to_file( precision_, "../matlab/cost_free.txt" );
  //    move3d_save_matrix_to_file( covariance_, "../matlab/invcost_matrix.txt"
  //    );

  return true;
}

bool LampSampler::addHessianToPrecisionMatrix(
    const Move3D::VectorTrajectory& traj) {
  // invert the control costs, initialize noise generators:
  // inv_control_costs_.clear();
  //    noise_generators_.clear();

  for (int i = 0; i < num_vars_free_; ++i) {
    Move3D::confPtr_t q = traj.getMove3DConfiguration(i);

    // Eigen::MatrixXd H = PlanEnv->getDouble(PlanParam::hessian_factor) *
    // std::pow( q->cost(), 4 )  * Eigen::MatrixXd::Identity( num_dofs_ ,
    // num_dofs_ );
    //         PlanEnv->getDouble(PlanParam::hessian_factor) *
    //         global_costSpace->getHessian( *q,
    //         planning_group_->getActiveDofs() );

    Eigen::MatrixXd H =
        PlanEnv->getDouble(PlanParam::lamp_hessian_factor) *
        global_costSpace->getHessian(*q, planning_group_->getActiveDofs());

    Eigen::EigenSolver<Eigen::MatrixXd> es(H);
    //        Eigen::VectorXcd eigen_values = es.eigenvalues();

    Eigen::VectorXcd eig_val = es.eigenvalues();
    //        Eigen::MatrixXcd D = eig_val.asDiagonal();
    Eigen::MatrixXcd V = es.eigenvectors();
    ;
    std::vector<bool> is_negative(eig_val.size(), false);

    for (size_t k = 0; k < is_negative.size(); k++) {
      if (0 > eig_val[k].real()) is_negative[k] = true;
      //                eig_val[k].real() = 0;
    }

    for (size_t k = 0; k < is_negative.size(); k++) {
      if (is_negative[k]) {
        Eigen::VectorXd Vk = V.col(k).real();
        H = H + Vk * Vk.transpose() * (-eig_val(k).real());
      }
    }

    //        H = ( V * D * V.inverse() ).real();

    //        Eigen::VectorXd J =  global_costSpace->getJacobian( *q,
    //        planning_group_->getActiveDofs() );
    //        cout << "J size : " << J.size() << endl;
    //         Eigen::MatrixXd H2 =
    //         PlanEnv->getDouble(PlanParam::hessian_factor) * J * J.transpose()
    //         ;

    //         cout << "H2 : "  << endl << H2 << endl;
    //         H += H2;

    // Eigen::MatrixXd H = PlanEnv->getDouble(PlanParam::hessian_factor) *
    // global_costSpace->getHessian( *q, planning_group_->getActiveDofs() );

    setTimeStepPrecisionMatrix(i, H);

    //        Eigen::VectorXcd ev = H.eigenvalues();

    //        for( int k=0; k<ev.size(); k++)
    //        {
    //            double real = ev[k].real();
    //            if( min_eigen_value_ > real )
    //                min_eigen_value_ = real;
    //        }

    //        if( min_eigen_value_ >  )

    //        cout << "H is : " << endl << H << endl;
    //        cout << "eig is : " << endl << H.eigenvalues().transpose().real()
    //        << endl;

    //        cout << "A : " << A << endl;
    //        cout << "min_eigen_value_ : " <<  min_eigen_value_ << endl;
  }

  //    cout << "control cost : " << endl << control_costs_[0] << endl;
  //    cout << "precision_ : " << endl << precision_<< endl;

  //    precision_ -= min_eigen_value_ * Eigen::MatrixXd::Identity(
  //    num_vars_free_*num_dofs_ , num_vars_free_*num_dofs_ );

  //    covariance_ = precision_.inverse();

  //    move3d_save_matrix_to_file( precision_, "../matlab/cost_free.txt" );
  //    move3d_save_matrix_to_file( covariance_, "../matlab/invcost_matrix.txt"
  //    );

  return true;
}

Eigen::VectorXd LampSampler::sample(double std_dev) {
  Eigen::VectorXd traj(num_dofs_ * num_vars_free_);
  noise_generators_[0].sample(tmp_noise_);
  traj = std_dev * tmp_noise_;
  return traj;
}

std::vector<VectorTrajectory> LampSampler::sampleTrajectories(
    int nb_trajectories, const Move3D::VectorTrajectory& current_trajectory) {
  global_trajToDraw.clear();

  //    preAllocateMultivariateGaussianSampler();
  //    addHessianToPrecisionMatrix( current_trajectory );

  std::vector<VectorTrajectory> trajectories(nb_trajectories);

  for (size_t i = 0; i < trajectories.size(); i++) {
    Eigen::VectorXd noise = sample();

    trajectories[i] = current_trajectory;
    trajectories[i].trajectory_ += noise;

    cout << "norm of noise : " << noise.norm() << endl;

    //        if( ENV.getBool(Env::drawTrajVector) && ENV.getBool(Env::drawTraj)
    //        )
    //        {
    //            global_trajToDraw.push_back(
    //            trajectories[i].getMove3DTrajectory() );
    //            global_trajToDraw.back().setColor( i );
    //        }
  }

  return trajectories;
}

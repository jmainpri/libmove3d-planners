#include "vector_trajectory.hpp"

using namespace Move3D;
using std::cout;
using std::endl;
using std::cin;

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------

VectorTrajectory::VectorTrajectory(int nb_dofs, int nb_var, double duration) {
  state_costs_ = Eigen::VectorXd::Zero(nb_var);
  out_of_bounds_ = false;
  num_vars_free_ = nb_var;
  num_dofs_ = nb_dofs;

  // Set duration to external parameter if not equal 0.0
  use_time_ = duration != 0.0 ? true : false;
  duration_ = use_time_ ? duration : 0.0;
  discretization_ = duration_ / double(num_vars_free_);

  trajectory_.resize(num_dofs_ * num_vars_free_);
}

void VectorTrajectory::setFromMove3DTrajectory(const Move3D::Trajectory& T) {
  const std::vector<Move3D::ChompDof>& joints = planning_group_->chomp_dofs_;

  trajectory_ = Eigen::VectorXd::Zero(num_vars_free_ * num_dofs_);

  double delta = T.getParamMax() / double(num_vars_free_ - 1);
  double s = 0.0;

  for (int i = 0; i < num_vars_free_; ++i) {
    Move3D::confPtr_t q = T.configAtParam(s);
    s += delta;

    for (size_t j = 0; j < joints.size(); ++j)
      (*this)(i, j) = (*q)[joints[j].move3d_dof_index_];
  }

  if (!use_time_) {
    discretization_ = T.getParamMax() / double(num_vars_free_);
  }
}

Move3D::confPtr_t VectorTrajectory::getMove3DConfiguration(int i) const {
  const std::vector<Move3D::ChompDof>& joints = planning_group_->chomp_dofs_;

  Move3D::confPtr_t q = planning_group_->robot_->getInitPos();

  for (size_t j = 0; j < joints.size(); ++j)
    (*q)[joints[j].move3d_dof_index_] = (*this)(i, j);

  return q;
}

Eigen::VectorXd VectorTrajectory::getTrajectoryPoint(int i) const {
  Eigen::VectorXd q(num_dofs_);

  for (int j = 0; j < num_dofs_; ++j) q(j) = (*this)(i, j);

  //    TODO implement as segment...
  //    q = trajectory_.segment(i,n);

  return q;
}

Move3D::Trajectory VectorTrajectory::getMove3DTrajectory() const {
  Move3D::Robot* rob = planning_group_->robot_;

  if (trajectory_.size() == 0) {
    cout << "empty parameters" << endl;
    return Move3D::Trajectory(rob);
  }

  const std::vector<Move3D::ChompDof>& joints = planning_group_->chomp_dofs_;

  // Create move3d trajectory
  Move3D::Trajectory T(rob);

  // cout << "discretization : " << discretization_ << endl;

  if (discretization_ != 0.0) {
    T.setUseTimeParameter(true);
    T.setUseConstantTime(true);
    T.setDeltaTime(discretization_);
  }

  for (int i = 0; i < num_vars_free_; ++i) {
    Move3D::confPtr_t q = rob->getInitPos();

    for (size_t j = 0; j < joints.size(); ++j)
      (*q)[joints[j].move3d_dof_index_] = (*this)(i, j);

    T.push_back(q->copy());
  }

  return T;
}

//! Interpolates linearly two configurations
//! u = 0 -> a
//! u = 1 -> b
Eigen::VectorXd VectorTrajectory::interpolate(const Eigen::VectorXd& a,
                                              const Eigen::VectorXd& b,
                                              double u) const {
  Eigen::VectorXd out;
  if (a.size() != b.size()) {
    cout << "Error in interpolate" << endl;
    return out;
  }

  out = a;
  for (int i = 0; i < int(out.size()); i++) {
    out[i] += u * (b[i] - a[i]);
  }
  return out;
}

Eigen::VectorXd VectorTrajectory::getSraightLineTrajectory() {
  // Set size
  Eigen::VectorXd straight_line = trajectory_;

  // Get number of points
  //    int nb_points = num_vars_free_;
  //    int dimension = num_dofs_;

  if (num_dofs_ < 1) {
    cout << "Error : the trajectory only has one dimension" << endl;
    return Eigen::VectorXd::Zero(1);
  }

  if (num_vars_free_ <= 2) {
    cout << "Warning : the trajectory is less or equal than 2 waypoints"
         << endl;
    return Eigen::VectorXd::Zero(1);
    ;
  }

  Eigen::VectorXd a(num_dofs_);
  Eigen::VectorXd b(num_dofs_);
  Eigen::VectorXd c(num_dofs_);

  //    for ( int j=0; j<num_dofs_; j++ ) {
  //        a[j] = straight_line[0*int(num_dofs_) + int(j)];
  //        b[j] = straight_line[(num_dofs_-1)*int(num_dofs_) + int(j)];
  //    }

  std::vector<int> dofs = planning_group_->getActiveDofs();

  //    for( int i=0; i<dofs.size() ; i++)
  //    {
  //        cout << "active dof " << i << " : " << dofs[i] << endl;
  //    }

  //    cout << "planning_group_ : " << planning_group_ << endl;
  //    cout << " -- robot : " << planning_group_->robot_ << endl;

  a = planning_group_->robot_->getInitPos()->getEigenVector(dofs);
  b = planning_group_->robot_->getGoalPos()->getEigenVector(dofs);

  //    cout << "a : " << a.transpose() << endl;
  //    cout << "b : " << b.transpose() << endl;

  double delta = 1 / double(num_vars_free_ - 1);
  double s = 0.0;

  // Only fill the inside points of the trajectory
  for (int i = 0; i < num_vars_free_; i++) {
    c = interpolate(a, b, s);
    s += delta;

    for (int j = 0; j < num_dofs_; j++)
      straight_line[i * int(num_dofs_) + int(j)] = c[j];
  }

  // cout << "straight line : " << endl << straight_line.transpose() << endl;

  return straight_line;
}

void VectorTrajectory::setDofTrajectoryBlock(int dof,
                                             const Eigen::VectorXd traj) {
  for (int i = 0; i < num_vars_free_; i++) {
    (*this)(i, dof) = traj[i];
  }
}

void VectorTrajectory::addToDofTrajectoryBlock(int dof,
                                               const Eigen::VectorXd traj) {
  for (int i = 0; i < num_vars_free_; i++) {
    (*this)(i, dof) += traj[i];
  }
}

Eigen::VectorXd VectorTrajectory::getDofTrajectoryBlock(int dof) const {
  Eigen::VectorXd traj(num_vars_free_);

  for (int i = 0; i < num_vars_free_; i++) {
    traj[i] = (*this)(i, dof);
  }

  return traj;
}

bool VectorTrajectory::getParameters(
    std::vector<Eigen::VectorXd>& parameters) const {
  if (int(parameters.size()) != num_dofs_) return false;

  for (int dof = 0; dof < num_dofs_; dof++)
    parameters[dof] = getDofTrajectoryBlock(dof);

  return true;
}

bool VectorTrajectory::getFreeParameters(
    std::vector<Eigen::VectorXd>& parameters) const {
  if (int(parameters.size()) != num_dofs_) return false;

  for (int dof = 0; dof < num_dofs_; dof++)
    parameters[dof] =
        getDofTrajectoryBlock(dof);  //.segment(start_index_, getNumFreePoints()
                                     ///*-id_fixed_*/ ); // TODO see why

  return true;
}

void VectorTrajectory::getTrajectoryPointP3d(int traj_point,
                                             Eigen::VectorXd& jnt_array) const {
  jnt_array.resize(num_dofs_);

  for (int i = 0; i < num_dofs_; i++) {
    jnt_array(i) = (*this)(traj_point, i);

    if (std::isnan(jnt_array(i))) {
      jnt_array(i) = 0;
    }
  }
}

int VectorTrajectory::getVectorIndex(int traj_point, int dof) {
  return traj_point * num_dofs_ + dof;
}

double& VectorTrajectory::dof_cost(int traj_point, int dof) {
  return dof_costs_[traj_point * num_dofs_ + dof];
}

double VectorTrajectory::dof_cost(int traj_point, int dof) const {
  return dof_costs_[traj_point * num_dofs_ + dof];
}

double& VectorTrajectory::operator()(int traj_point, int dof) {
  return trajectory_[traj_point * num_dofs_ + dof];
}

double VectorTrajectory::operator()(int traj_point, int dof) const {
  return trajectory_[traj_point * num_dofs_ + dof];
}

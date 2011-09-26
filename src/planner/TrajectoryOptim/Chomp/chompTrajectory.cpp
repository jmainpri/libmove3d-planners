/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#include "chompTrajectory.hpp"
#include <iostream>

using namespace std;
using namespace tr1;

//namespace chomp
//{

ChompTrajectory::ChompTrajectory(const API::Trajectory& T, int diff_rule_length, const ChompPlanningGroup& active_joints_):
robot_model_(T.getRobot())
//planning_group_(planning_group),
//discretization_(source_traj.discretization_)
{
  //num_joints_ = robot_model_->getNumberOfJoints();
  num_joints_ = active_joints_.num_joints_;
  
  duration_ = T.getRangeMax();
  
  int number_inital_points = T.getNbOfPaths()+1;
  
  // figure out the num_points_:
  // we need diff_rule_length-1 extra points on either side:
  int start_extra = (diff_rule_length - 1);
  int end_extra = (diff_rule_length - 1);
  
  num_points_ = (T.getNbOfPaths()+1) + start_extra + end_extra;
  
  start_index_ = diff_rule_length - 1;
  end_index_ = (num_points_ - 1) - (diff_rule_length - 1);
  //duration_ = (num_points_ - 1)*discretization_;
  discretization_ = duration_/(num_points_-1);
  
  // allocate the memory:
  init();
  
  full_trajectory_index_.resize(num_points_);
  
  // now copy the trajectories over:
  for (int i=0; i<num_points_; i++)
  {
    int source_traj_point = i - start_extra;
    
    if (source_traj_point < 0)
      source_traj_point = 0;
    
    if (source_traj_point >= number_inital_points )
      source_traj_point = number_inital_points-1;
    
    full_trajectory_index_[i] = source_traj_point;
    
    for(int j=0; j<num_joints_; j++)
    {
      int source_joint = active_joints_.chomp_joints_[j].move3d_dof_index_;
      
      shared_ptr<Configuration> q;
      
      if( source_traj_point != (number_inital_points-1) )
      {
        q = T.getLocalPathPtrAt( source_traj_point )->getBegin();
      }
      else
      {
        q = T.getLocalPathPtrAt( source_traj_point-1 )->getEnd();
      }
      
      (*this)(i,j) = (*q)[source_joint];
    }
  }
}

ChompTrajectory::ChompTrajectory(const ChompTrajectory& source_traj, 
                                 int diff_rule_length):
robot_model_(source_traj.robot_model_),
discretization_(source_traj.discretization_)
{
  num_joints_ = source_traj.num_joints_;
  
  // figure out the num_points_:
  // we need diff_rule_length-1 extra points on either side:
  int start_extra = (diff_rule_length - 1) - source_traj.start_index_;
  int end_extra = (diff_rule_length - 1) - ((source_traj.num_points_-1) - source_traj.end_index_);
  
  num_points_ = source_traj.num_points_ + start_extra + end_extra;
  
  cout << "source_traj.num_points_ = " << source_traj.num_points_ << endl;
  cout << "start_extra = " << start_extra << endl;
  cout << "end_extra = " << end_extra << endl;
  
  start_index_ = diff_rule_length - 1;
  end_index_ = (num_points_ - 1) - (diff_rule_length - 1);
  duration_ = (num_points_ - 1)*discretization_;
  
  // allocate the memory:
  init();
  
  full_trajectory_index_.resize(num_points_);
  
  // now copy the trajectories over:
  for (int i=0; i<num_points_; i++)
  {
    int source_traj_point = i - start_extra;
    
    if (source_traj_point < 0)
      source_traj_point = 0;
    
    if (source_traj_point >= source_traj.num_points_)
      source_traj_point = source_traj.num_points_-1;
    
    full_trajectory_index_[i] = source_traj_point;
    
    for (int j=0; j<num_joints_; j++)
    {
      int source_joint =j;
      (*this)(i,j) = source_traj(source_traj_point, source_joint);
    }
  }
}

ChompTrajectory::~ChompTrajectory()
{
  
}

void ChompTrajectory::init()
{
  //trajectory_.resize(num_points_, Eigen::VectorXd(num_joints_));
  trajectory_ = Eigen::MatrixXd(num_points_, num_joints_);
}

void ChompTrajectory::updateFromGroupTrajectory(const ChompTrajectory& group_trajectory)
{
  //cout << "Todo : updateFromGroupTrajectory" << endl;
  int num_vars_free = end_index_ - start_index_ + 1;
  
  //  for (int i=0; i<group_trajectory.planning_group_->num_joints_; i++)
  //  {
  //    int target_joint = group_trajectory.planning_group_->chomp_joints_[i].kdl_joint_index_;
  //    trajectory_.block(start_index_, target_joint, num_vars_free, 1)
  //    = group_trajectory.trajectory_.block(group_trajectory.start_index_, i, num_vars_free, 1);
  //  }
  
  for (int i=0; i<group_trajectory.num_joints_; i++)
  {
    int target_joint = i;
    trajectory_.block(start_index_, target_joint, num_vars_free, 1)
    = group_trajectory.trajectory_.block(group_trajectory.start_index_, i, num_vars_free, 1);
  }
}

void ChompTrajectory::fillInMinJerk()
{
  double start_index = start_index_-1;
  double end_index = end_index_+1;
  double T[6]; // powers of the time duration
  T[0] = 1.0;
  T[1] = (end_index - start_index)*discretization_;
  
  for (int i=2; i<=5; i++)
    T[i] = T[i-1]*T[1];
  
  // calculate the spline coefficients for each joint:
  // (these are for the special case of zero start and end vel and acc)
  double coeff[num_joints_][6];
  for (int i=0; i<num_joints_; i++)
  {
    double x0 = (*this)(start_index,i);
    double x1 = (*this)(end_index,i);
    coeff[i][0] = x0;
    coeff[i][1] = 0;
    coeff[i][2] = 0;
    coeff[i][3] = (-20*x0 + 20*x1) / (2*T[3]);
    coeff[i][4] = (30*x0 - 30*x1) / (2*T[4]);
    coeff[i][5] = (-12*x0 + 12*x1) / (2*T[5]);
  }
  
  // now fill in the joint positions at each time step
  for (int i=start_index+1; i<end_index; i++)
  {
    double t[6]; // powers of the time index point
    t[0] = 1.0;
    t[1] = (i - start_index)*discretization_;
    for (int k=2; k<=5; k++)
      t[k] = t[k-1]*t[1];
    
    for (int j=0; j<num_joints_; j++)
    {
      (*this)(i,j) = 0.0;
      for (int k=0; k<=5; k++)
      {
        (*this)(i,j) += t[k]*coeff[j][k];
      }
    }
  }
}

void ChompTrajectory::getTrajectoryPointP3d(int traj_point, Eigen::VectorXd& jnt_array) const
{
  jnt_array.resize( num_joints_ );
  
  for (int i=0; i<num_joints_; i++)
  {
    jnt_array(i) = trajectory_(traj_point,i);
    if ( isnan(jnt_array(i)) ) 
    {
      jnt_array(i) = 0;
    }
  }
}

void ChompTrajectory::print()
{
  cout << trajectory_ << endl;
}

// } // namespace chomp

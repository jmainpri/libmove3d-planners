//
//  chompPlanningGroup.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 04/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "chompPlanningGroup.hpp"
#include "chompUtils.hpp"

// Included for random number
#include <libmove3d/include/P3d-pkg.h>

using namespace std;
using namespace Move3D;

ChompPlanningGroup::ChompPlanningGroup(Robot* rob,
                                       const std::vector<int>& active_joints) {
  robot_ = rob;
  chomp_dofs_.clear();

  bool print_group = true;

  if (print_group)
    cout << "Creating planning group for : " << robot_->getName() << endl;

  for (size_t i = 0; i < active_joints.size(); i++) {
    for (size_t j = 0; j < robot_->getJoint(active_joints[i])->getNumberOfDof();
         j++) {
      Joint* move3d_joint = robot_->getJoint(active_joints[i]);

      if (!move3d_joint->isJointDofUser(j)) continue;

      double min, max;
      // robot_->getJoint( active_joints[i] )->getDofBounds(j,min,max);
      move3d_joint->getDofRandBounds(j, min, max);

      if (min == max) continue;

      if (print_group) {
        cout << "pgID(" << chomp_dofs_.size() << ") : Joint("
             << active_joints[i]
             << "), Dof : " << move3d_joint->getIndexOfFirstDof() + j << ", "
             << move3d_joint->getName() << " , ";
        cout << "Is dof user : ";
        cout << "(min = " << min << ", max = " << max << ") , ";
        cout << "Is dof angular :  " << move3d_joint->isJointDofAngular(j)
             << endl;
      }

      ChompDof jnt;

      jnt.move3d_joint_ = move3d_joint;
      jnt.move3d_joint_index_ = active_joints[i];
      jnt.move3d_dof_index_ = move3d_joint->getIndexOfFirstDof() + j;
      jnt.chomp_joint_index_ = i;
      jnt.joint_name_ = move3d_joint->getName();
      jnt.is_circular_ = move3d_joint->isJointDofCircular(j);
      jnt.has_joint_limits_ = true;
      jnt.joint_limit_min_ = min;
      jnt.joint_limit_max_ = max;
      jnt.joint_update_limit_ = 0.10;
      jnt.is_translational_ = !move3d_joint->isJointDofAngular(j);
      jnt.range_ = max - min;

      chomp_dofs_.push_back(jnt);
    }
  }

  num_dofs_ = chomp_dofs_.size();
}

// Change robot
ChompPlanningGroup::ChompPlanningGroup(const ChompPlanningGroup& pg,
                                       Robot* rob) {
  robot_ = rob;
  chomp_dofs_.clear();

  link_names_ = pg.link_names_;
  collision_link_names_ = pg.collision_link_names_;
  collision_points_ = pg.collision_points_;

  for (size_t i = 0; i < pg.chomp_dofs_.size(); i++) {
    ChompDof jnt = pg.chomp_dofs_[i];
    jnt.move3d_joint_ = robot_->getJoint(jnt.joint_name_);
    chomp_dofs_.push_back(jnt);
  }

  num_dofs_ = chomp_dofs_.size();
}

std::vector<int> ChompPlanningGroup::getActiveDofs() const {
  std::vector<int> active_joints;
  for (int i = 0; i < int(chomp_dofs_.size()); i++) {
    active_joints.push_back(chomp_dofs_[i].move3d_dof_index_);
    //        cout << "name : " << chomp_dofs_[i].joint_name_ << ", dof_index_ :
    //        " << chomp_dofs_[i].move3d_dof_index_ << endl;
  }

  return active_joints;
}

std::vector<Move3D::Joint*> ChompPlanningGroup::getActiveJoints() const {
  std::vector<Move3D::Joint*> active_joints;
  for (int i = 0; i < int(chomp_dofs_.size()); i++) {
    Move3D::Joint* joint = chomp_dofs_[i].move3d_joint_;

    bool add_to_vector = true;  // only add if not already in vector
    for (int j = 0; j < int(active_joints.size()); j++) {
      if (joint == active_joints[j]) {
        add_to_vector = false;
        break;
      }
    }
    if (add_to_vector) active_joints.push_back(joint);
  }

  return active_joints;
}

bool ChompPlanningGroup::addCollisionPoint(CollisionPoint& collision_point) {
  // create the new parent joints indexing vector:
  // std::vector<int> parent_joints( num_joints_, 0 );

  // ROS_INFO_STREAM("Num joints is " << num_joints_ << " parent size is " <<
  // collision_point.getParentJoints().size());

  // check if this collision point is controlled by any joints which belong to
  // the group
  bool add_this_point = false;
  for (int i = 0; i < num_dofs_; i++) {
    if (collision_point.isParentJoint(chomp_dofs_[i].move3d_joint_index_)) {
      add_this_point = true;
      break;
    }
  }

  if (!add_this_point) return false;

  collision_points_.push_back(
      CollisionPoint(collision_point, collision_point.getParentJoints()));
  return true;
}

void ChompPlanningGroup::draw() const {
  //  cout << "Number of collision points : " << collision_points_.size() <<
  //  endl;
  for (unsigned int i = 0; i < collision_points_.size(); i++) {
    //    if (i>=38)
    //      continue;

    Eigen::Transform3d T =
        robot_->getJoint(collision_points_[i].getParentJoints().back())
            ->getMatrixPos();

    collision_points_[i].draw(T);
  }
}

void ChompPlanningGroup::draw(std::vector<Eigen::Transform3d>& segment) const {
  for (unsigned int i = 0; i < collision_points_.size(); i++) {
    Eigen::Transform3d T = segment[collision_points_[i].getSegmentNumber()];
    collision_points_[i].draw(T);
  }
}

void ChompPlanningGroup::draw(
    std::vector<std::vector<double> >& segment) const {
  for (unsigned int i = 0; i < collision_points_.size(); i++) {
    Eigen::Transform3d T;
    stdVectorToEigenTransform(segment[collision_points_[i].getSegmentNumber()],
                              T);
    collision_points_[i].draw(T);
  }
}

double Move3D::chomp_random_value(double max, double min) {
  return ((((double)p3d_random(0, RAND_MAX)) / RAND_MAX) * (max - min)) + min;
}

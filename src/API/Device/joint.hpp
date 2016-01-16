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
#ifndef JOINT_HPP
#define JOINT_HPP

#include "API/ConfigSpace/configuration.hpp"
#include "utils/eigen_transition.hpp"

#include <libmove3d/include/p3d_matrix.h>

#ifndef _DEVICE_H
struct jnt;
#endif

namespace Move3D {

class Robot;

/*!
 @ingroup ROBOT
 @brief This class holds a Joint and is associated with a Body (Link)
 It's the basic element of a kinematic chain
 */
class Joint {
 public:
  /**
 * Constructor
 * @param The p3d_jnt that is used
 */
  Joint(Move3D::Robot* R, void* jntPt, int id = -1, bool copy = false);

  /**
 * Destructor of the class
 */
  ~Joint();

  /**
 * Returns the name of the joint
 */
  std::string getName() const { return m_Name; }

  /**
   * Get the Joint structure
   */
  jnt* getP3dJointStruct() const { return static_cast<jnt*>(m_Joint); }

  /**
   * Get the Joint structure
   */
  void* getJointStruct() const { return m_Joint; }

  /**
   * Get the Matrix abs_pos of the Joint
   */
  Eigen::Transform3d getMatrixPos() const;

  /**
   * Get the Vector abs_pos of the Joint
   */
  Eigen::Vector3d getVectorPos() const;

  /**
   * Get the Vector abs_pos of the Joint
   */
  Eigen::VectorXd getXYZPose() const;

  /**
   * Random shoot the joint
   */
  void shoot(Configuration& q, bool sample_passive = false);

  /**
   * Returns true if the joint Dof is angular
   */
  bool isJointDofAngular(int ithDoF) const;

  /**
   * Returns true if the joint Dof is circular
   */
  bool isJointDofCircular(int ithDoF) const;

  /**
   * Returns the Joint Dof
   */
  double getJointDof(int ithDoF) const;

  /**
   * Set the Joint Dof
   */
  void setJointDof(int ithDoF, double value);

  /**
   * True if Joint Dof is user
   */
  bool isJointDofUser(int ithDoF) const;

  /**
   * Get Min Max dof
   */
  void getDofBounds(int ithDoF, double& vmin, double& vmax) const;

  /**
    * Returns the random bounds
    */
  void getDofRandBounds(int ithDoF, double& vmin, double& vmax) const;

  /**
   * Get Number of DoF
   */
  unsigned int getNumberOfDof() const;

  /**
   * Get Dof Pos in Configuration
   */
  unsigned int getIndexOfFirstDof() const;

  /**
   * Get Dof Pos in Configuration
   */
  std::vector<unsigned int> getDofIndices() const;

  /**
 * Get the id in the joint structure of the robot
 */
  int getId() const { return m_id; }

  /**
   * Set the config from the DoF values
   */
  void setConfigFromDofValues(Configuration& q);

  /**
 * Returns the previous joint
 */
  Joint* getPreviousJoint();

  /**
 * Returns the array of previous joints
 */
  std::vector<Joint*> getAllPrevJoints();

  /**
   * Returns the value size of the attached body
   */
  double getDist();

  /**
    * Returns the robot
    */
  Robot* getRobot() const { return m_Robot; }

 private:
  Robot* m_Robot;
  void* m_Joint;      /*!< The p3d structure for the Joint*/
  std::string m_Name; /*!< The Joint's Name */
  bool m_copy; /*!< Is true if the p3d_jnt copies and not only points to the
                  structure */
  int m_id;    /*!< id with which it was initilized */
};
}

void move3d_set_fct_joint_constructor(
    boost::function<void*(Move3D::Joint*, void*, std::string& name)> fct);
void move3d_set_fct_joint_get_vector_pos(
    boost::function<Eigen::Vector3d(const Move3D::Joint*)> fct);
void move3d_set_fct_joint_get_matrix_pos(
    boost::function<Eigen::Transform3d(const Move3D::Joint*)> fct);
void move3d_set_fct_joint_joint_shoot(
    boost::function<void(Move3D::Joint*, Move3D::Configuration&, bool)> fct);
void move3d_set_fct_joint_get_joint_dof(
    boost::function<double(const Move3D::Joint*, int)> fct);
void move3d_set_fct_joint_is_joint_dof_angular(
    boost::function<double(const Move3D::Joint*, int)> fc);
void move3d_set_fct_joint_is_joint_dof_circular(
    boost::function<double(const Move3D::Joint*, int)> fc);
void move3d_set_fct_joint_set_joint_dof(
    boost::function<void(const Move3D::Joint*, int, double)> fct);
void move3d_set_fct_joint_is_joint_user(
    boost::function<bool(const Move3D::Joint*, int)> fct);
void move3d_set_fct_joint_get_bound(
    boost::function<void(const Move3D::Joint*, int, double&, double&)> fct);
void move3d_set_fct_joint_get_bound_rand(
    boost::function<void(const Move3D::Joint*, int, double&, double&)> fct);
void move3d_set_fct_joint_get_nb_of_dofs(
    boost::function<int(const Move3D::Joint*)> fct);
void move3d_set_fct_joint_get_index_of_first_dof(
    boost::function<int(const Move3D::Joint*)> fct);
void move3d_set_fct_joint_get_previous_joint(
    boost::function<Move3D::Joint*(const Move3D::Joint*, Move3D::Robot*)> fct);
void move3d_set_fct_joint_joint_dist(
    boost::function<double(const Move3D::Joint*)> fct);

#endif

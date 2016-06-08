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
#ifndef ROBOT_HPP
#define ROBOT_HPP

////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API

#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include "API/Device/joint.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Trajectory/trajectory.hpp"

#ifndef _DEVICE_H
struct rob;
// struct jnt;
#endif

namespace Move3D
{

class Scene;

/**
 * @ingroup CPP_API
 * @defgroup ROBOT Device
 * @brief The robot class
 */

/*!
 @ingroup ROBOT
 @brief This class holds a the robot
 represented by a kinematic chain
 */
class Robot
{
 public:
  // constructor and destructor
  /**
   * Constructeur de la classe
   * @param R le p3d_rob pour lequel l'objet Robot est créé
   */
  Robot(void* R, bool copy = false);

  /**
   * Destructeur de la classe
   */
  virtual ~Robot();

  /**
   * @brief copyRobotStruct
   * @param robotPt
   * @return
   */
  void* copyRobotStruct(void* robotPt);

  // Accessor
  /**
   * obtient la structure p3d_rob de la classe
   * @return la structure p3d_rob
   */
  rob* getP3dRobotStruct();

  /**
   * obtient la structure p3d_rob de la classe
   * @return la structure p3d_rob
   */
  void* getRobotStruct() { return robot_kin_struct_; }

  /**
   * obtient le nom du Robot
   * @return le nom du Robot
   */
  const std::string& getName();

  /**
   * Gets wether the robot uses libmove3d structures
   * @return bool
   */
  bool getUseLibmove3dStruct() { return contains_libmove3d_struct_; }

  /**
   * Sets wether the robot uses libmove3d structures
   */
  void setUseLibmove3dStruct(bool use_libmove3d_struct)
  {
    contains_libmove3d_struct_ = use_libmove3d_struct;
  }

  /**
   * Associate an active scene to the robot
   */
  void setActiveScene(Move3D::Scene* sc) { active_scene_ = sc; }

  /**
   * Returns the active scene involving the robot
   */
  Move3D::Scene* getActiveScene() { return active_scene_; }

  /**
   * Gets traj associated with Robot
   * @return pointer to structure p3d_traj
   */
  void* getTrajStruct();

  /**
   * Gets the current trajectory
   */
  Move3D::Trajectory getCurrentTraj();

  /**
   * Gets the current trajectory
   */
  void removeCurrentTraj();

  /**
   * Gets the current API Trajectory
   */
  const Move3D::Trajectory& getCurrentMove3DTraj();

  /**
   * Gets the current trajectory
   */
  void setCurrentMove3DTraj(const Move3D::Trajectory& traj);

  /**
   * Gets the current trajectory
   */
  void removeCurrentMove3DTraj();

  /**
   * Get the number of dofs
   * @return the Number of dofs
   */
  unsigned int getNumberOfDofs() const;

  /**
   * Get the number of Joints
   * @return the Number of Joints
   */
  unsigned int getNumberOfJoints() const;

  /**
   * Gets the ith joint structure
   * @return ith joint structure
   */
  Joint* getJoint(unsigned int i);

  /**
   * Gets joint by name
   * @return pointer to the joint by name or NULL if not found
   */
  Joint* getJoint(std::string name) const;

  /**
   * Gets joint by name
   * @return pointer to the joint by name or NULL if not found
   */
  const std::vector<Joint*>& getJoints() const { return joints_; }

  /**
 * Returns an vector of all robot joints
 */
  const std::vector<Joint*>& getAllJoints();

  /*
    * Is point in object box
    */
  bool isInObjectBox(const Eigen::Vector3d& p0);

  /**
   * Returns the Object Box
   */
  std::vector<Eigen::Vector3d> getObjectBox();

  /**
   * Initializes the box in which the FF Will be sampled
   * @param dimensions 3 first and offset from robot base 3 others
   */
  void initObjectBox(
      const std::vector<double>& dimensions = std::vector<double>());

  /**
   * tire une Configuration aléatoire pour le Robot
   * @param samplePassive (default = TRUE) indique si l'on tire les joints
   * passif ou non (ie. FALSE dans le cas de ML-RRT)
   * @return la Configuration tirée
   */
  confPtr_t shoot(bool samplePassive = false);

  /**
   * obtient une Configuration-Direction aléatoire pour le Robot
   * @param samplePassive (default = true) indique si l'on tire les joints
   * passif ou non (ie. FALSE dans le cas de ML-RRT)
   * @return la Configuration tirée
   */
  confPtr_t shootDir(bool samplePassive = false);

  /**
   * shoots the active free flyer inside a box
   */
  confPtr_t shootFreeFlyer(double* box);

  /**
   * set and update the active free flyer
   */
  int setAndUpdateFreeFlyer(const Eigen::Vector3d& pos);

  /**
   * place le Robot dans une Configuration
   * @param q la Configuration dans laquelle le Robot sera placé
   * @return la Configuration est atteignable cinématiquement
   */
  int setAndUpdate(const Configuration& q, bool withoutFreeFlyers = false);

  /**
   * place le Robot dans une Configuration
   * @param q la Configuration dans laquelle le Robot sera placé
   * @return la Configuration est atteignable cinématiquement
   */
  bool setAndUpdateMultiSol(const Configuration& q);

  /**
   * place le Robot dans une Configuration, without checking the cinematic
   * constraints.
   * @param q la Configuration dans laquelle le Robot sera placé
   */
  void setAndUpdateWithoutConstraints(const Configuration& q);

  /**
   * obtient la Configuration current du Robot
   * @return la Configuration current du Robot
   */
  confPtr_t getInitPos();

  /**
   * Sets the Initial Position of the Robot
   */
  void setInitPos(Configuration& conf);

  /**
   * Returns true if the robot is
   * in colision with obstacles, other robots and self
   */
  bool isInCollision();

  /**
   * Returns true if the robot is
   * in colision with obstacles or other robots
   */
  bool isInCollisionWithOthersAndEnv();

  /**
   * Returns true if the robot is
   * in colision with robots
   */
  bool isInCollisionWithOthers(std::vector<Move3D::Robot*>& others);

  /**
    * Robot to environment distance
    */
  double distanceToEnviroment();

  /**
    * Robot to robot distance
    */
  double distanceToRobot(Robot* robot);

  /**
   * obtient la Configuration GoTo du Robot
   * @return la Configuration GoTo du Robot
   */
  confPtr_t getGoalPos();

  /**
   * Sets the Goto Position of the Robot
   */
  void setGoalPos(Configuration& conf);

  /**
   * Returns the Robot current Configuration
   */
  confPtr_t getCurrentPos();

  /**
   * Returns a new configuration
   */
  confPtr_t getNewConfig();

  /**
    * Get stored vector config
    */
  std::vector<confPtr_t> getStoredConfigs();

  /**
   * Returns an array of dof ids
   */
  std::vector<int> getActiveJointsIds();

  /**
   * Returns an array of dof ids
   */
  std::vector<int> getActiveDoFsFromJoints(const std::vector<int>& joint_ids);

  /**
   * Get Number of active DoFs
   */
  unsigned int getNumberOfActiveDoF();

  /**
   * Get Number of active DoFs
   */
  Joint* getIthActiveDoFJoint(unsigned int ithActiveDoF,
                              unsigned int& ithDofOnJoint);

  /**
   * Get all dof ids
   */
  std::vector<int> getAllDofIds() const;

  /**
   * Get Jacobian
   */
  Eigen::MatrixXd getJacobian(const std::vector<Joint*>& active_joints,
                              Joint* eef,
                              bool with_rotations,
                              bool with_height = false) const;

#ifdef LIGHT_PLANNER
  /**
   * Returns the Virtual object dof
   */
  int getObjectDof();

  /**
   * Returns Wether the closed chain constraint
   * is active
   */
  bool isActiveCcConstraint();

  /**
   * Activate Constraint
   */
  void activateCcConstraint();

  /**
   * Deactivate Constraint
   */
  void deactivateCcConstraint();

  /**
   * Returns the base Joint
   */
  void* getBaseJnt();

  /**
   * Shoots a random direction
   */
  confPtr_t shootRandomDirection();

  /**
   * Shoots the base Joint of the robot
   */
  confPtr_t shootBase();

  /**
   *
   */
  confPtr_t shootBaseWithoutCC();

  /**
   * Shoots the base Joint of the robot
   */
  confPtr_t shootAllExceptBase();

  /**
   * Update but not base
   */
  bool setAndUpdateAllExceptBase(Configuration& Conf);

  /**
   * shoots the robot by shooting the
   * FreeFlyer inside the accecibility box
   */
  void shootObjectJoint(Configuration& Conf);
#endif

 private:
  void* robot_kin_struct_; /*!< une structure de p3d_rob contenant les données
                              sur le Robot*/
  std::string name_;       /*!< le nom du Robot*/
  Move3D::Scene* active_scene_;
  bool copy_; /*!< Is true if the p3d_jnt copies and not only points to the
                 structure */
  bool contains_libmove3d_struct_;
  unsigned int nb_dofs_;
  std::vector<Joint*> joints_;
  Eigen::Vector3d object_box_center_;
  Eigen::Vector3d object_box_dimentions_;
  Move3D::Trajectory current_trajectory_;
};

extern Robot* API_activeRobot;
}

void move3d_set_fct_robot_constructor(
    boost::function<void(Move3D::Robot*,
                         void*,
                         unsigned int&,
                         bool,
                         std::string&,
                         std::vector<Move3D::Joint*>&)> fct);
void move3d_set_fct_robot_get_current_trajectory(
    boost::function<Move3D::Trajectory(Move3D::Robot*)> fct);
void move3d_set_fct_robot_shoot(
    boost::function<Move3D::confPtr_t(Move3D::confPtr_t, bool)> fct);
void move3d_set_fct_robot_shoot_dir(
    boost::function<Move3D::confPtr_t(Move3D::confPtr_t, bool)> fct);
void move3d_set_fct_robot_set_and_update(boost::function<
    bool(Move3D::Robot*, const Move3D::Configuration& q, bool)> fct);
void move3d_set_fct_robot_set_and_update_multi_sol(
    boost::function<bool(Move3D::Robot*, const Move3D::Configuration& q)> fct);
void move3d_set_fct_robot_without_constraints(
    boost::function<void(Move3D::Robot*, const Move3D::Configuration& q)> fct);
void move3d_set_fct_robot_is_in_collision(
    boost::function<bool(Move3D::Robot*)> fct);
void move3d_set_fct_robot_is_in_collision_with_others_and_env(
    boost::function<bool(Move3D::Robot*)> fct);
void move3d_set_fct_robot_is_in_collision_with_others(
    boost::function<bool(Move3D::Robot*, std::vector<Move3D::Robot*>&)> fct);
void move3d_set_fct_robot_distance_to_env(
    boost::function<double(Move3D::Robot*)> fct);
void move3d_set_fct_robot_distance_to_robot(
    boost::function<double(Move3D::Robot*, Move3D::Robot*)> fct);
void move3d_set_fct_robot_get_init_pos(
    boost::function<Move3D::confPtr_t(Move3D::Robot*)> fct);
void move3d_set_fct_robot_set_init_pos(
    boost::function<void(Move3D::Robot*, const Move3D::Configuration&)> fct);
void move3d_set_fct_robot_get_goal_pos(
    boost::function<Move3D::confPtr_t(Move3D::Robot*)> fct);
void move3d_set_fct_robot_set_goal_pos(
    boost::function<void(Move3D::Robot*, const Move3D::Configuration&)> fct);
void move3d_set_fct_robot_get_current_pos(
    boost::function<Move3D::confPtr_t(Move3D::Robot*)> fct);
void move3d_set_fct_robot_get_new_pos(
    boost::function<Move3D::confPtr_t(Move3D::Robot*)> fct);
void move3d_set_fct_robot_get_number_of_active_dofs(
    boost::function<unsigned int(Move3D::Robot*)> fct);
void move3d_set_fct_robot_get_ith_active_dof_joint(boost::function<
    Move3D::Joint*(Move3D::Robot*, unsigned int, unsigned int&)> fct);

#endif

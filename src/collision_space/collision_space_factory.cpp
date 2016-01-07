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
#include "collision_space_factory.hpp"

#include "body_surface_sampler.hpp"
#include "collision_space.hpp"

#include "API/project.hpp"
#include "API/Device/robot.hpp"
#include "API/libmove3d_api.hpp"
#include "API/libmove3d_simple_api.hpp"

#include "planner/cost_space.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planner/planEnvironment.hpp"

#include "hri_costspace/HRICS_costspace.hpp"

#include "utils/multilocalpath_utils.hpp"

#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/Planner-pkg.h>

using namespace Move3D;
using std::cout;
using std::endl;

CollisionSpace* m_collspace = NULL;

static std::vector<int> m_active_joints;
static std::vector<int> m_planner_joints;
static std::vector<CollisionPoint> m_collision_points;

static bool m_add_human = true;
static bool m_init = false;

static Robot* m_robot = NULL;

static int m_id_of_first_active_joint = 2;

void traj_optim_reset_collision_space() {
  if (global_collisionSpace) delete global_collisionSpace;
  global_collisionSpace = NULL;

  //    if( m_collspace )
  //        delete m_collspace;
  m_collspace = NULL;

  m_active_joints.clear();
  m_planner_joints.clear();
  m_collision_points.clear();
  m_init = false;
}

void traj_optim_add_human_to_collision_space(bool add) { m_add_human = add; }

const CollisionSpace* traj_optim_get_collision_space() { return m_collspace; }

std::vector<CollisionPoint> traj_optim_get_collision_points() {
  return m_collision_points;
}

std::vector<int> traj_optim_get_active_joints() { return m_active_joints; }

std::vector<int> traj_optim_get_planner_joints() { return m_planner_joints; }

//! initializes the collision points
// --------------------------------------------------------
bool traj_optim_init_collision_points() {
  // Generate Bounding volumes for active joints
  BodySurfaceSampler* sampler = m_collspace->getBodySampler();

  // Get all joints active in the motion planning
  // and compute bounding cylinders
  std::vector<Joint*> joints;
  joints.clear();

  for (unsigned int i = 0; i < m_active_joints.size(); i++) {
    joints.push_back(m_robot->getJoint(m_active_joints[i]));
  }
  /*double maxRadius =*/sampler->generateRobotBoudingCylinder(m_robot, joints);

  // Get all planner joint and compute collision points
  std::vector<int> planner_joints_id;
  for (unsigned int i = 0; i < m_planner_joints.size(); i++) {
    planner_joints_id.push_back(m_planner_joints[i]);
  }
  m_collision_points = sampler->generateRobotCollisionPoints(
      m_robot, m_active_joints, planner_joints_id, m_id_of_first_active_joint);

  cout << "nb of collision point are " << m_collision_points.size() << endl;

  // Set the collision space as global (drawing)
  global_collisionSpace = m_collspace;
  return true;
}

//! initializes the collision space
// --------------------------------------------------------
void traj_optim_init_collision_space() {
  //    std::vector<double> env_size =
  //    global_Project->getActiveScene()->getBounds();
  //    double pace = env_size[1] - env_size[0];
  //    pace = max( env_size[3] - env_size[3], pace);
  //    pace = max( env_size[4] - env_size[5], pace);
  //    pace /= ENV.getInt(Env::nbCells);

  //    ChronoTimeOfDayOn();

  m_collspace =
      new CollisionSpace(m_robot,
                         double(ENV.getInt(Env::nbCells)) / 100,
                         global_Project->getActiveScene()->getBounds());
  m_collspace->resetPoints();

  // Warning
  // If not human add bodies that are
  // not part of the active joints set
  if (m_robot->getName().find("HUMAN") == std::string::npos) {
    //  cout << "robot name : " << m_robot->getName() << endl;
    //  cout << "Add robot bodies exit " << endl; // exit(0);

    for (unsigned int joint_id = 0; joint_id < m_robot->getNumberOfJoints();
         joint_id++) {
      if (find(m_active_joints.begin(), m_active_joints.end(), joint_id) ==
          m_active_joints.end()) {
        m_collspace->addRobotBody(m_robot->getJoint(joint_id));
      }
    }
  }
  // Special case for BIOMECH model
  // Add the pelvis and the legs
  else if (m_robot->getName().find("HERAKLES_HUMAN2") != std::string::npos) {
    m_collspace->addRobotBody(m_robot->getJoint(1));
    for (unsigned int joint_id = 31; joint_id < m_robot->getNumberOfJoints();
         joint_id++) {
      m_collspace->addRobotBody(m_robot->getJoint(joint_id));
    }
  }

  if (m_add_human) {
    Scene* sc = global_Project->getActiveScene();

    for (unsigned int i = 0; i < sc->getNumberOfRobots(); i++) {
      Robot* rob = sc->getRobot(i);

      if ((m_robot != rob) &&
          rob->getName().find("HUMAN") != std::string::npos) {
        m_collspace->addRobot(rob);
      }
    }
    // if human then add ROBOT to the collision environment
    // instead of human partner
    if (m_robot->getName().find("HUMAN") != std::string::npos) {
      Move3D::Robot* robot = sc->getRobotByNameContaining("ROBOT");
      if (robot) m_collspace->addRobot(robot);
    }
  }

  // Add all moving and static obstacles
  m_collspace->addEnvPoints();

  // Adds the sampled points to the distance field
  m_collspace->propagateDistance();

  //    double time=0.0;
  //    ChronoTimeOfDayTimes(&time);
  //    ChronoTimeOfDayOff();
  //    cout << " collision space computed in : " << time << endl;
}

//****************************************************************
//* Default example
//****************************************************************

bool traj_optim_default_init() {
  if (!m_robot) {
    cout << "robot not initialized in file " << __FILE__ << " ,  "
         << __PRETTY_FUNCTION__ << endl;
    return false;
  }

  m_active_joints.clear();
  m_planner_joints.clear();

  m_active_joints = m_robot->getActiveJointsIds();
  m_planner_joints = m_active_joints;

  m_id_of_first_active_joint = 0;

  for (size_t i = 0; i < m_planner_joints.size(); i++) {
    cout << "Active joints : " << m_planner_joints[i] << endl;
  }

  //    for (int i=1; i<int(m_robot->getNumberOfJoints()); i++)
  //    {
  //        // Set the active joints (links)
  //        m_active_joints.push_back( i );

  //        // Set the planner joints
  //        m_planner_joints.push_back( i );
  //    }

  m_collspace = NULL;

  if (ENV.getBool(Env::isCostSpace)) {
    return (global_costSpace != NULL);
  }
  if (move3d_use_api_functions_collision_space()) {
    if (global_collisionSpace == NULL) {
      traj_optim_init_collision_space();
      traj_optim_init_collision_points();
    } else {
      m_collspace = global_collisionSpace;
    }
  }

  return true;
}

//****************************************************************
//* 2D Simple example
//****************************************************************

//! Initializes the optimization for a costspace
// --------------------------------------------------------
bool traj_optim_simple_init() {
  if (!m_robot) {
    cout << "robot not initialized in file " << __FILE__ << " ,  "
         << __PRETTY_FUNCTION__ << endl;
    return false;
  }

  // Set the active joints (links)
  m_active_joints.clear();
  m_active_joints.push_back(1);

  // Set the planner joints
  m_planner_joints.clear();
  m_planner_joints.push_back(1);

  ENV.setInt(Env::jntToDraw, 1);

  m_collspace = NULL;
  return (global_costSpace != NULL);
}

//****************************************************************
//* 2D Costmap example
//****************************************************************

//! Initializes the optimization for a costspace
// --------------------------------------------------------
bool traj_optim_costmap_init() {
  if (!m_robot) {
    cout << "robot not initialized in file " << __FILE__ << " ,  "
         << __PRETTY_FUNCTION__ << endl;
    return false;
  }

  // Set the active joints (links)
  m_active_joints.clear();
  m_active_joints.push_back(1);

  // Set the planner joints
  m_planner_joints.clear();
  m_planner_joints.push_back(1);

  ENV.setInt(Env::jntToDraw, 1);

  m_collspace = NULL;

  bool valid_costspace = global_costSpace->setCost("costMap2D");
  return valid_costspace;
}

//****************************************************************
//* Navigation example
//****************************************************************

//! Sets the point on the navigation DoF
void traj_optim_navigation_generate_points() {
  // Set the active joints (links)
  m_active_joints.clear();
  m_active_joints.push_back(1);

  // Set the planner joints
  m_planner_joints.clear();
  m_planner_joints.push_back(1);

  // Generate Bounding volumes for active joints
  BodySurfaceSampler* sampler = m_collspace->getBodySampler();

  // Get all joints active in the motion planning
  // and compute bounding cylinders
  std::vector<Joint*> joints;
  joints.clear();
  for (unsigned int i = 0; i < m_active_joints.size(); i++) {
    joints.push_back(m_robot->getJoint(m_active_joints[i]));
  }
  sampler->generateRobotBoudingCylinder(m_robot, joints);

  // Get all planner joint and compute collision points
  std::vector<int> planner_joints_id;
  for (unsigned int i = 0; i < m_planner_joints.size(); i++) {
    planner_joints_id.push_back(m_planner_joints[i]);
  }
  m_collision_points = sampler->generateRobotCollisionPoints(
      m_robot, m_active_joints, planner_joints_id);

  //  sampler->generateRobotBoudingCylinder( m_robot, m_robot->getAllJoints() );
  //  m_collision_points = sampler->generateAllRobotCollisionPoints( m_robot );
}

//****************************************************************
//* HRICS example
//****************************************************************

//! Sets the point on the navigation DoF
void traj_optim_hrics_generate_points() {
  // Set the active joints (links)
  m_active_joints.clear();
  m_active_joints.push_back(1);

  // Set the planner joints
  m_planner_joints.clear();
  m_planner_joints.push_back(1);

  // Generate Bounding volumes for active joints
  BodySurfaceSampler* sampler = m_collspace->getBodySampler();

  // Get all joints active in the motion planning
  // and compute bounding cylinders
  std::vector<Joint*> joints;
  joints.clear();
  for (unsigned int i = 0; i < m_active_joints.size(); i++) {
    joints.push_back(m_robot->getJoint(m_active_joints[i]));
  }
  sampler->generateRobotBoudingCylinder(m_robot, joints);

  // Get all planner joint and compute collision points
  std::vector<int> planner_joints_id;
  for (unsigned int i = 0; i < m_planner_joints.size(); i++) {
    planner_joints_id.push_back(m_planner_joints[i]);
  }
  m_collision_points = sampler->generateRobotCollisionPoints(
      m_robot, m_active_joints, planner_joints_id);

  //  sampler->generateRobotBoudingCylinder( m_robot, m_robot->getAllJoints() );
  //  m_collision_points = sampler->generateAllRobotCollisionPoints( m_robot );
}

//! initializes the collision space
// --------------------------------------------------------
void traj_optim_manip_init_joints() {
  m_collspace = NULL;

  if (m_robot->getName() == "PR2_ROBOT") {
    // Set the active joints (links)
    m_active_joints.clear();
    m_active_joints.push_back(6);
    m_active_joints.push_back(7);
    m_active_joints.push_back(8);
    m_active_joints.push_back(9);
    m_active_joints.push_back(10);
    m_active_joints.push_back(11);
    m_active_joints.push_back(12);
    m_active_joints.push_back(14);
    m_active_joints.push_back(15);

    // Set the planner joints
    m_planner_joints.clear();
    m_planner_joints.push_back(6);
    m_planner_joints.push_back(7);
    m_planner_joints.push_back(8);
    m_planner_joints.push_back(9);
    m_planner_joints.push_back(10);
    m_planner_joints.push_back(11);
    m_planner_joints.push_back(12);
  }

  if (m_robot->getName() == "JUSTIN_ROBOT") {
    // Set the active joints (links)
    m_active_joints.clear();
    m_active_joints.push_back(2);
    m_active_joints.push_back(3);
    m_active_joints.push_back(4);
    m_active_joints.push_back(5);
    m_active_joints.push_back(17);
    m_active_joints.push_back(18);
    m_active_joints.push_back(19);
    m_active_joints.push_back(20);
    m_active_joints.push_back(21);
    m_active_joints.push_back(22);

    m_active_joints.push_back(23);

    // Set the planner joints
    m_planner_joints.clear();
    m_planner_joints.push_back(2);
    m_planner_joints.push_back(3);
    m_planner_joints.push_back(4);
    m_planner_joints.push_back(5);
    m_planner_joints.push_back(18);
    m_planner_joints.push_back(19);
    m_planner_joints.push_back(20);
    m_planner_joints.push_back(21);
    m_planner_joints.push_back(22);
    m_planner_joints.push_back(23);
    m_planner_joints.push_back(24);
    //    m_planner_joints.clear();
    //    m_planner_joints.push_back( 2 );
    //    m_planner_joints.push_back( 3 );
    //    m_planner_joints.push_back( 4 );
    //    m_planner_joints.push_back( 30 );
  }
}

//! initializes the collision space
//! and points
// --------------------------------------------------------
void traj_optim_hrics_mobile_manip_init_joints() {
  // Set the active joints (links)
  m_active_joints.clear();
  m_active_joints.push_back(1);
  m_active_joints.push_back(6);
  m_active_joints.push_back(7);
  m_active_joints.push_back(8);
  m_active_joints.push_back(9);
  m_active_joints.push_back(10);
  m_active_joints.push_back(11);
  m_active_joints.push_back(12);

  m_active_joints.push_back(14);
  m_active_joints.push_back(15);

  // Set the planner joints
  m_planner_joints.clear();
  m_planner_joints.push_back(1);
  m_planner_joints.push_back(6);
  m_planner_joints.push_back(7);
  m_planner_joints.push_back(8);
  m_planner_joints.push_back(9);
  m_planner_joints.push_back(10);
  m_planner_joints.push_back(11);
  m_planner_joints.push_back(12);

  m_collspace = NULL;
}

//! initializes the collision space
//! and points
// --------------------------------------------------------
void traj_optim_hrics_human_trajectory_manip_init_joints() {
  // Set the active joints (links)

  if (m_robot->getJoint("rShoulderZ") != NULL)  // Kinect case
  {
    m_active_joints.clear();
    m_active_joints.push_back(1);  // Pelvis
    //    m_active_joints.push_back( 2 ); // TorsoX
    //    m_active_joints.push_back( 3 ); // TorsoY
    m_active_joints.push_back(4);   // TorsoZ
    m_active_joints.push_back(8);   // rShoulderX
    m_active_joints.push_back(9);   // rShoulderZ
    m_active_joints.push_back(10);  // rShoulderY
    //    m_active_joints.push_back( 11 ); // rArmTrans
    m_active_joints.push_back(12);  // rElbowZ
    //    active_joints_.push_back(14); // joint name : rWristX
    //    active_joints_.push_back(15); // joint name : rWristY
    m_active_joints.push_back(16);  // joint name : rWristZ

    // Set the planner joints
    m_planner_joints.clear();
    m_planner_joints.push_back(1);
    m_planner_joints.push_back(2);
    m_planner_joints.push_back(3);
    m_planner_joints.push_back(4);
    m_planner_joints.push_back(8);
    m_planner_joints.push_back(9);
    m_planner_joints.push_back(10);
    m_planner_joints.push_back(11);
    m_planner_joints.push_back(12);
    //    m_planner_joints.push_back( 13 );
  } else {
    m_active_joints.clear();
    m_planner_joints.clear();

    // HERE WE declare the human joint and joint limits
    // before the TRO reviews we did not have definitions of the joint
    // limits here and Pelvis was allways included to make 23 DoFs of the paper

    if (PlanEnv->getBool(PlanParam::trajStompMoveEndConfig)) {
      // TODO remove joint limits from here

      p3d_jnt_set_dof_rand_bounds(
          m_robot->getJoint("rShoulderTransX")->getP3dJointStruct(),
          0,
          .016,
          .020);
      p3d_jnt_set_dof_rand_bounds(
          m_robot->getJoint("rShoulderTransY")->getP3dJointStruct(),
          0,
          .32,
          .34);
      p3d_jnt_set_dof_rand_bounds(
          m_robot->getJoint("rShoulderTransZ")->getP3dJointStruct(),
          0,
          .24,
          .26);
      p3d_jnt_set_dof_rand_bounds(
          m_robot->getJoint("rArmTrans")->getP3dJointStruct(), 0, .38, .40);
      p3d_jnt_set_dof_rand_bounds(
          m_robot->getJoint("rForeArmTrans")->getP3dJointStruct(), 0, .23, .26);
    } else {
      m_active_joints.push_back(m_robot->getJoint("Pelvis")->getId());
      m_planner_joints.push_back(m_robot->getJoint("Pelvis")->getId());
    }

    //
    if (!PlanEnv->getBool(PlanParam::trajStompMoveEndConfig)) {
      m_active_joints.push_back(m_robot->getJoint("TorsoX")->getId());
      m_active_joints.push_back(m_robot->getJoint("TorsoZ")->getId());
      m_active_joints.push_back(m_robot->getJoint("TorsoY")->getId());
    }
    m_active_joints.push_back(m_robot->getJoint("rShoulderTransX")->getId());
    m_active_joints.push_back(m_robot->getJoint("rShoulderTransY")->getId());
    m_active_joints.push_back(m_robot->getJoint("rShoulderTransZ")->getId());
    m_active_joints.push_back(m_robot->getJoint("rShoulderY1")->getId());
    m_active_joints.push_back(m_robot->getJoint("rShoulderX")->getId());
    m_active_joints.push_back(m_robot->getJoint("rShoulderY2")->getId());
    m_active_joints.push_back(m_robot->getJoint("rArmTrans")->getId());
    m_active_joints.push_back(m_robot->getJoint("rElbowZ")->getId());
    m_active_joints.push_back(m_robot->getJoint("rElbowX")->getId());
    m_active_joints.push_back(m_robot->getJoint("rElbowY")->getId());
    m_active_joints.push_back(m_robot->getJoint("rForeArmTrans")->getId());
    //        m_active_joints.push_back( m_robot->getJoint( "rWristZ" )->getId()
    //        );
    //        m_active_joints.push_back( m_robot->getJoint( "rWristX" )->getId()
    //        );
    m_active_joints.push_back(m_robot->getJoint("rWristY")->getId());

    m_planner_joints.push_back(m_robot->getJoint("TorsoX")->getId());
    m_planner_joints.push_back(m_robot->getJoint("TorsoZ")->getId());
    m_planner_joints.push_back(m_robot->getJoint("TorsoY")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rShoulderTransX")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rShoulderTransY")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rShoulderTransZ")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rShoulderY1")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rShoulderX")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rShoulderY2")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rArmTrans")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rElbowZ")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rElbowX")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rElbowY")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rForeArmTrans")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rWristZ")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rWristX")->getId());
    m_planner_joints.push_back(m_robot->getJoint("rWristY")->getId());
  }

  m_collspace = NULL;
}

//****************************************************************
//* Common functions
//****************************************************************

//! Get current robot
//! Initializes the costspace and multi localpath
// --------------------------------------------------------
bool traj_optim_init_collision_spaces(traj_optim::ScenarioType sce,
                                      Robot* rob) {
  if (m_init == true) return true;

  m_robot = rob;

  cout << "Robot is : " << m_robot->getName() << " in " << __PRETTY_FUNCTION__
       << endl;

  if (m_robot == NULL) return false;

  switch (sce) {
    case traj_optim::Default:
      cout << "Init with default parameters" << endl;
      if (!traj_optim_default_init()) return false;
      break;

    case traj_optim::CostMap:

      cout << "Init with 2D costmap" << endl;
      if (!traj_optim_costmap_init()) return false;
      // PlanEnv->setDouble(PlanParam::trajOptimStdDev,0.1);
      // PlanEnv->setInt(PlanParam::nb_pointsOnTraj,50);
      // PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,10);
      // PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.001);
      break;

    case traj_optim::Simple:

      cout << "Init Simple Nav" << endl;
      if (!traj_optim_simple_init()) return false;
      // PlanEnv->setDouble(PlanParam::trajOptimStdDev,0.030000);
      // PlanEnv->setInt(PlanParam::nb_pointsOnTraj,50);
      // PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,10);
      // PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.000005);
      break;

    case traj_optim::Shelf:

      cout << "Init Shelf" << endl;
      cout << "Set robot, localpath and cntrts with " << m_robot->getName()
           << endl;

      if (move3d_use_api_functions()) {
        // traj_optim_set_MultiLP();
        // traj_optim_invalidate_cntrts();
        traj_optim_manip_init_joints();
        traj_optim_shelf_set_localpath_and_cntrts();

        if (!global_collisionSpace) {
          traj_optim_init_collision_space();
          traj_optim_init_collision_points();
        } else {
          m_collspace = global_collisionSpace;
        }
      } else {
        traj_optim_default_init();
      }

      // PlanEnv->setDouble(PlanParam::trajOptimStdDev,2);
      // PlanEnv->setInt(PlanParam::nb_pointsOnTraj,15);
      // PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,10);
      // PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.1);

      // PlanEnv->setDouble(PlanParam::trajOptimStdDev,2);
      // PlanEnv->setInt(PlanParam::nb_pointsOnTraj,50);
      // PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,30);
      // PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.01);
      break;

    case traj_optim::HumanAwareNav:

      cout << "Init HumanAwareNav" << endl;
      cout << "Set robot, localpath and cntrts with ";
      cout << m_robot->getName() << endl;

      traj_optim_set_MultiLP();
      traj_optim_invalidate_cntrts();
      traj_optim_navigation_set_localpath_and_cntrts();

      if (!HRICS_humanCostMaps) {
        cout << "No Collspace" << endl;
        return false;
      } else {
        m_collspace = global_collisionSpace;
        traj_optim_navigation_generate_points();
      }

      PlanEnv->setDouble(PlanParam::trajOptimStdDev, 3);
      PlanEnv->setInt(PlanParam::nb_pointsOnTraj, 30);
      PlanEnv->setDouble(PlanParam::trajOptimObstacWeight, 20);
      PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight, 0.1);
      break;

    case traj_optim::HumanAwareManip:

      cout << "Init HumanAwareManip" << endl;
      cout << "Set robot, localpath and cntrts with ";
      cout << m_robot->getName() << endl;

      traj_optim_set_MultiLP();

      if (m_robot->getName() == "PR2_ROBOT") {
        traj_optim_invalidate_cntrts();
        traj_optim_shelf_set_localpath_and_cntrts();
        traj_optim_manip_init_joints();

        // Init collspace
        if (!global_collisionSpace) {
          traj_optim_init_collision_space();
          traj_optim_init_collision_points();
        } else {
          m_collspace = global_collisionSpace;
        }
      } else {
        traj_optim_init_mlp_cntrts_and_fix_joints(m_robot);
        traj_optim_manip_init_joints();
      }

      // PlanEnv->setDouble(PlanParam::trajOptimStdDev,3);
      // PlanEnv->setInt(PlanParam::nb_pointsOnTraj,30);
      // PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,20);
      // PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.1);
      break;

    case traj_optim::HumanAwareMobileManip:

      cout << "Init HumanAwareMobileManip" << endl;
      cout << "Set robot, localpath and cntrts with ";
      cout << m_robot->getName() << endl;

      traj_optim_set_MultiLP();
      traj_optim_invalidate_cntrts();
      traj_optim_hrics_mobile_manip_localpath_and_cntrts();
      traj_optim_hrics_mobile_manip_init_joints();
      break;

    case traj_optim::HumanSimulation:

      cout << "Init Human Simulation (Trajectory costspace)" << endl;

      if (m_robot->getName() == "PR2_ROBOT") {
        traj_optim_invalidate_cntrts();
        traj_optim_shelf_set_localpath_and_cntrts();
        traj_optim_manip_init_joints();
      } else if (m_robot->getName() == "HERAKLES_HUMAN2") {
        traj_optim_hrics_human_trajectory_manip_init_joints();
      } else {
        cout << "Not defined for robot " << m_robot->getName() << endl;
      }

      // Init collspace
      if (!global_collisionSpace) {
        traj_optim_init_collision_space();
        traj_optim_init_collision_points();
      } else {
        m_collspace = global_collisionSpace;
      }

      // exit(0);

      break;

    case traj_optim::Navigation:

      cout << "Init Navigation" << endl;
      cout << "Set robot, localpath and cntrts with ";
      cout << m_robot->getName() << endl;

      traj_optim_set_MultiLP();
      traj_optim_invalidate_cntrts();
      traj_optim_navigation_set_localpath_and_cntrts();

      if (!global_collisionSpace) {
        cout << "No Collspace" << endl;
        return false;
      } else {
        m_collspace = global_collisionSpace;
        traj_optim_navigation_generate_points();
      }

      PlanEnv->setDouble(PlanParam::trajOptimStdDev, 3);
      PlanEnv->setInt(PlanParam::nb_pointsOnTraj, 30);
      PlanEnv->setDouble(PlanParam::trajOptimObstacWeight, 20);
      PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight, 0.1);
      break;

    case traj_optim::LegiblePlane:

      cout << "Init LegiblePlane" << endl;
      cout << "Set robot, localpath and cntrts with ";
      cout << m_robot->getName() << endl;

      if (!traj_optim_default_init()) return false;
      break;

      //        traj_optim_set_MultiLP();
      //        traj_optim_invalidate_cntrts();
      //        traj_optim_navigation_set_localpath_and_cntrts();
      break;
  }

  return true;
}

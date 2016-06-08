/*
 *  joint.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 12/05/10.
 *  Copyright 2010 LAAS-CNRS. All rights reserved.
 *
 */

#include "API/Device/robot.hpp"

#include "P3d-pkg.h"

#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <string>
#include <boost/function.hpp>

using namespace std;
using namespace Eigen;
using namespace Move3D;

// *****************************************************************************
// API FUNCTIONS
// *****************************************************************************

static boost::function<void*(Joint*, void*, std::string& name)>
    Move3DJointConstructor;
static boost::function<Vector3d(const Joint*)> Move3DJointGetVectorPos;
static boost::function<Transform3d(const Joint*)> Move3DJointGetMatrixPos;
static boost::function<void(Joint*, Configuration&, bool)> Move3DJointShoot;
static boost::function<double(const Joint*, int)> Move3DJointGetJointDoF;
static boost::function<double(const Joint*, int)> Move3DJointIsJointDoFAngular;
static boost::function<double(const Joint*, int)> Move3DJointIsJointDoFCircular;
static boost::function<void(const Joint*, int, double)> Move3DJointSetJointDoF;
static boost::function<bool(const Joint*, int)> Move3DJointIsJointUser;
static boost::function<void(const Joint*, int, double&, double&)>
    Move3DJointGetBound;
static boost::function<void(const Joint*, int, double&, double&)>
    Move3DJointGetRandBound;
static boost::function<int(const Joint*)> Move3DJointGetNbOfDoFs;
static boost::function<int(const Joint*)> Move3DJointGetIndexOfFirstDoF;
static boost::function<Joint*(const Joint*, Robot*)>
    Move3DJointGetPreviousJoint;
static boost::function<double(const Joint*)> Move3DJointDist;

// *****************************************************************************
// SETTERS
// *****************************************************************************

void move3d_set_fct_joint_constructor(
    boost::function<void*(Joint*, void*, std::string& name)> fct)
{
  Move3DJointConstructor = fct;
}
void move3d_set_fct_joint_get_vector_pos(
    boost::function<Vector3d(const Joint*)> fct)
{
  Move3DJointGetVectorPos = fct;
}
void move3d_set_fct_joint_get_matrix_pos(
    boost::function<Transform3d(const Joint*)> fct)
{
  Move3DJointGetMatrixPos = fct;
}
void move3d_set_fct_joint_joint_shoot(
    boost::function<void(Joint*, Configuration&, bool)> fct)
{
  Move3DJointShoot = fct;
}
void move3d_set_fct_joint_get_joint_dof(
    boost::function<double(const Joint*, int)> fct)
{
  Move3DJointGetJointDoF = fct;
}
void move3d_set_fct_joint_is_joint_dof_angular(
    boost::function<double(const Joint*, int)> fct)
{
  Move3DJointIsJointDoFAngular = fct;
}
void move3d_set_fct_joint_is_joint_dof_circular(
    boost::function<double(const Joint*, int)> fct)
{
  Move3DJointIsJointDoFCircular = fct;
}
void move3d_set_fct_joint_set_joint_dof(
    boost::function<void(const Joint*, int, double)> fct)
{
  Move3DJointSetJointDoF = fct;
}
void move3d_set_fct_joint_is_joint_user(
    boost::function<bool(const Joint*, int)> fct)
{
  Move3DJointIsJointUser = fct;
}
void move3d_set_fct_joint_get_bound(
    boost::function<void(const Joint*, int, double&, double&)> fct)
{
  Move3DJointGetBound = fct;
}
void move3d_set_fct_joint_get_bound_rand(
    boost::function<void(const Joint*, int, double&, double&)> fct)
{
  Move3DJointGetRandBound = fct;
}
void move3d_set_fct_joint_get_nb_of_dofs(boost::function<int(const Joint*)> fct)
{
  Move3DJointGetNbOfDoFs = fct;
}
void move3d_set_fct_joint_get_index_of_first_dof(
    boost::function<int(const Joint*)> fct)
{
  Move3DJointGetIndexOfFirstDoF = fct;
}
void move3d_set_fct_joint_get_previous_joint(
    boost::function<Joint*(const Joint*, Robot*)> fct)
{
  Move3DJointGetPreviousJoint = fct;
}
void move3d_set_fct_joint_joint_dist(boost::function<double(const Joint*)> fct)
{
  Move3DJointDist = fct;
}

// ****************************************************************************************************

Joint::Joint(Robot* R, void* jntPt, int id, bool copy) : m_Robot(R), m_id(id)
{
  m_Joint = Move3DJointConstructor(this, jntPt, m_Name);
}

Vector3d Joint::getVectorPos() const { return Move3DJointGetVectorPos(this); }

Transform3d Joint::getMatrixPos() const
{
  return Move3DJointGetMatrixPos(this);
}

Eigen::VectorXd Joint::getXYZPose() const
{
  Eigen::VectorXd xyz(6);

  Eigen::Transform3d T = getMatrixPos();

  xyz(0) = T.translation()(0);
  xyz(1) = T.translation()(1);
  xyz(2) = T.translation()(2);

  Eigen::Vector3d angles = T.rotation().eulerAngles(0, 1, 2);
  xyz(3) = angles(0);
  xyz(4) = angles(1);
  xyz(5) = angles(2);

  return xyz;
}

Eigen::Quaterniond Joint::getOrientation() const
{
  Transform3d T = getMatrixPos();
  return Eigen::Quaterniond(T.rotation());
}

void Joint::shoot(Configuration& q, bool sample_passive)
{
  Move3DJointShoot(this, q, sample_passive);
}

double Joint::getJointDof(int ithDoF) const
{
  return Move3DJointGetJointDoF(this, ithDoF);
}

void Joint::setJointDof(int ithDoF, double value)
{
  Move3DJointSetJointDoF(this, ithDoF, value);
}

bool Joint::isJointDofAngular(int ithDoF) const
{
  return Move3DJointIsJointDoFAngular(this, ithDoF);
}

bool Joint::isJointDofCircular(int ithDoF) const
{
  return Move3DJointIsJointDoFCircular(this, ithDoF);
}

bool Joint::isJointDofUser(int ithDoF) const
{
  return Move3DJointIsJointUser(this, ithDoF);
}

void Joint::getDofBounds(int ithDoF, double& vmin, double& vmax) const
{
  Move3DJointGetBound(this, ithDoF, vmin, vmax);
}

void Joint::getDofRandBounds(int ithDoF, double& vmin, double& vmax) const
{
  Move3DJointGetRandBound(this, ithDoF, vmin, vmax);
}

unsigned int Joint::getNumberOfDof() const
{
  return Move3DJointGetNbOfDoFs(this);
}

unsigned int Joint::getIndexOfFirstDof() const
{
  return Move3DJointGetIndexOfFirstDoF(this);
}

std::vector<unsigned int> Joint::getDofIndices() const
{
  std::vector<unsigned int> indices(getNumberOfDof());
  int index_of_first_dof = getIndexOfFirstDof();

  for (size_t i = 0; i < indices.size(); i++)
    indices[i] = index_of_first_dof + i;

  return indices;
}

void Joint::setConfigFromDofValues(Configuration& q)
{
  int nb_of_dofs = getNumberOfDof();
  int index_first_dof = getIndexOfFirstDof();

  for (int j = 0; j < nb_of_dofs; j++) {
    int k = index_first_dof + j;
    q[k] = getJointDof(j);
  }
}

Joint* Joint::getPreviousJoint()
{
  return Move3DJointGetPreviousJoint(this, m_Robot);
}

std::vector<Joint*> Joint::getAllPrevJoints()
{
  Joint* jnt(getPreviousJoint());

  std::vector<Joint*> prevJoints;
  prevJoints.clear();

  cout << "Prev Joints ---------" << endl;
  while (jnt != NULL) {
    cout << "jnt->m_id = " << jnt->m_id << endl;
    prevJoints.push_back(jnt);
    jnt = jnt->getPreviousJoint();
  }

  return prevJoints;
}

double Joint::getDist() { return Move3DJointDist(this); }

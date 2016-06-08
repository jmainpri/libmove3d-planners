/*
 *  HRICS_Natural.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "HRICS_natural.hpp"
#include "API/Grids/gridsAPI.hpp"
#include "planner/cost_space.hpp"
#include "utils/misc_functions.hpp"
#include "gestures/HRICS_record_motion.hpp"
#include "HRICS_parameters.hpp"
#include "HRICS_human_simulator.hpp"

#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Planner-pkg.h>

#include <boost/bind.hpp>

using namespace std;
using namespace HRICS;
using namespace Move3D;
using namespace Eigen;

MOVE3D_USING_SHARED_PTR_NAMESPACE

extern Eigen::Vector3d global_DrawnSphere;

extern Move3D::TwoDGrid* API_activeRobotGrid;

// Natural::Natural() :
//    m_leftArmCost(false),
//    m_BestPointsSorted(false),
//    m_Grid(NULL),
//    m_G(1000000.0)
//{
//    m_Robot =
//    global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");

//    cout << "Robot in natural Cost Space is : " << m_Robot->getName() << endl;

//    initGeneral();
//}

/***********************************************/
/* JUSTIN NORMAL */
#define JUSTIN_JOINT_SPINE 1;  //  1, 2,  3,  4
#define JUSTIN_JOINT_HEAD 6;   //  6,  7,  8

#define JUSTIN_JOINT_ARM_RIGTH_SHOULDER 9;  //  9,  10, 11
#define JUSTIN_JOINT_ARM_RIGTH_ELBOW 12;    // 12 , 13
#define JUSTIN_JOINT_ARM_RIGTH_WRIST 14;    // 14, 15, 16

#define JUSTIN_JOINT_ARM_LEFT_SHOULDER 17;  // 17, 18, 19
#define JUSTIN_JOINT_ARM_LEFT_ELBOW 20;     // 20, 21
#define JUSTIN_JOINT_ARM_LEFT_WRIST 22;     // 22, 23, 24

/***********************************************/
/* Achile NORMAL */
#define ACHILE_JOINT_SPINE 2;  //  2,  3,  4
#define ACHILE_JOINT_HEAD 5;   //  5,  6,  7

#define ACHILE_JOINT_ARM_RIGTH_SHOULDER 8;  //  8,  9, 10
#define ACHILE_JOINT_ARM_RIGTH_ELBOW 11;    // 11
#define ACHILE_JOINT_ARM_RIGTH_WRIST 12;    // 12, 13, 14

#define ACHILE_JOINT_ARM_LEFT_SHOULDER 15;  // 15, 16, 17
#define ACHILE_JOINT_ARM_LEFT_ELBOW 18;     // 18
#define ACHILE_JOINT_ARM_LEFT_WRIST 19;     // 19, 20, 21

/***********************************************/
// KINECT
#define HERAKLES_JOINT_SPINE 2;  //  2,  3,  4
#define HERAKLES_JOINT_HEAD 5;   //  5,  6,  7

#define HERAKLES_JOINT_ARM_RIGTH_SHOULDER 8;  //  8,  9, 10
#define HERAKLES_JOINT_ARM_RIGTH_ELBOW 12;    // 11
#define HERAKLES_JOINT_ARM_RIGTH_WRIST 14;    // 12, 13, 14

#define HERAKLES_JOINT_ARM_LEFT_SHOULDER 17;  // 15, 16, 17
#define HERAKLES_JOINT_ARM_LEFT_ELBOW 21;     // 18
#define HERAKLES_JOINT_ARM_LEFT_WRIST 23;     // 19, 20, 21

/***********************************************/
// BIOMECH
#define BIOMECH_JOINT_SPINE 2;  //  2,  3,  4
#define BIOMECH_JOINT_HEAD 5;   //  5,  6,  7

#define BIOMECH_JOINT_ARM_RIGTH_SHOULDER 11;  //  11,  12, 13
#define BIOMECH_JOINT_ARM_RIGTH_ELBOW 15;     // 15, 16, 17
#define BIOMECH_JOINT_ARM_RIGTH_WRIST 19;     // 19, 20, 21

#define BIOMECH_JOINT_ARM_LEFT_SHOULDER 22;  // 22, 23, 24
#define BIOMECH_JOINT_ARM_LEFT_ELBOW 26;     // 26
#define BIOMECH_JOINT_ARM_LEFT_WRIST 28;     // 28, 29, 30

/***********************************************/
/* OLDDUDE NORMAL */
#define OLDDUDE_JOINT_SPINE 2;  //  2,  3,  4
#define OLDDUDE_JOINT_HEAD 5;   //  5,  6,  7

#define OLDDUDE_JOINT_ARM_RIGTH_SHOULDER 8;  //  8,  9, 10
#define OLDDUDE_JOINT_ARM_RIGTH_ELBOW 11;    // 11
#define OLDDUDE_JOINT_ARM_RIGTH_WRIST 12;    // 12, 13, 14

#define OLDDUDE_JOINT_ARM_LEFT_SHOULDER 15;  // 15, 16, 17
#define OLDDUDE_JOINT_ARM_LEFT_ELBOW 18;     // 18
#define OLDDUDE_JOINT_ARM_LEFT_WRIST 19;     // 19, 20, 21

Natural::Natural(Move3D::Robot* R)
    : m_debug(false),
      m_leftArmCost(false),
      m_BestPointsSorted(false),
      m_Robot(R),
      m_Grid(NULL),
      m_G(1000000.0)
{
  initGeneral();
}

Natural::~Natural()
{
  /*if( m_Grid != NULL )
  {
      delete m_Grid;
  }*/
}

void Natural::initGeneral()
{
  map<string, Kinematic> kinos;

  kinos["Error"] = Default;
  kinos["ROBOT_JUSTIN"] = Justin;
  kinos["ACHILE"] = Achile;
  kinos["HERAKLES"] = Herakles;
  kinos["BIOMECH"] = Biomech;

  if (m_Robot->getName().find("ROBOT_JUSTIN") != string::npos) {
    m_KinType = Justin;
  } else if (m_Robot->getName().find("ACHILE") != string::npos) {
    m_KinType = Achile;
  } else if (m_Robot->getName().find("HERAKLES") != string::npos) {
    m_KinType = Biomech;
    //    m_KinType = Herakles;
  } else if (m_Robot->getName().find("OLDDUDE") != string::npos) {
    m_KinType = OldDude;
  } else if (m_Robot->getName().find("OLDDUDE") != string::npos) {
    m_KinType = OldDude;
  } else if (m_Robot->getName().find("BIOMECH") != string::npos) {
    m_KinType = Biomech;
  }

  confPtr_t q_curr = m_Robot->getCurrentPos();

  // Get  number of Dofs of all joints
  m_nbDof = 0;
  for (unsigned int i = 0; i < m_Robot->getNumberOfJoints(); i++)
    m_nbDof += m_Robot->getJoint(i)->getNumberOfDof();

  switch (m_KinType) {
    case Justin:
      cout << "KinType of HRICS::Natural is Justin ( " << m_Robot->getName()
           << " ) " << endl;
      initNaturalJustin();
#ifdef LIGHT_PLANNER
      m_IndexObjectDof = m_Robot->getObjectDof();
#endif
      m_computeNbOfIK = false;
      m_IsHuman = false;
      break;

    case Achile:
      cout << "KinType of HRICS::Natural is Achile ( " << m_Robot->getName()
           << " ) " << endl;
      initNaturalAchile();
      m_IndexObjectDof = 0;
      m_computeNbOfIK = false;
      m_IsHuman = true;
      // A meanwhile to make the soft work ...
      m_Agents = hri_create_agents();
      break;

    case Herakles:
      cout << "KinType of HRICS::Natural is Herakles ( " << m_Robot->getName()
           << " ) " << endl;
      initNaturalHerakles();
      m_IndexObjectDof = 0;
      m_computeNbOfIK = false;
      m_IsHuman = true;
      m_Agents = hri_create_agents();
      break;

    case Biomech:
      cout << "KinType of HRICS::Natural is Biomech ( " << m_Robot->getName()
           << " ) " << endl;
      initNaturalBiomech();
      m_IndexObjectDof = 0;
      m_computeNbOfIK = false;
      m_IsHuman = true;
      m_Agents = hri_create_agents();
      break;

    case OldDude:
      cout << "KinType of HRICS::Natural is OldDude ( " << m_Robot->getName()
           << " ) " << endl;
      initNaturalOldDude();
      m_IndexObjectDof = 0;
      m_computeNbOfIK = false;
      m_IsHuman = true;
      m_Agents = hri_create_agents();
      break;

    default:
      cout << "No proper robot has been selected in Natural cost function"
           << endl;
      break;
  }

  // Define cost functions
  cout << " add cost : "
       << "costHumanNatural" << endl;
  Move3D::global_costSpace->addCost(
      "costNatural", boost::bind(&HRICS::Natural::cost, this, _1));

  m_Robot->setAndUpdate(*q_curr);

  m_Grid = NULL;
  cout << "Object Dof is " << m_IndexObjectDof << endl;
}

void Natural::initConfigIndices()
{
  m_CONFIG_INDEX_SPINE = m_Robot->getJoint(m_JOINT_SPINE)->getIndexOfFirstDof();
  m_CONFIG_INDEX_HEAD = m_Robot->getJoint(m_JOINT_HEAD)->getIndexOfFirstDof();

  m_CONFIG_INDEX_ARM_RIGTH_SHOULDER =
      m_Robot->getJoint(m_JOINT_ARM_RIGTH_SHOULDER)->getIndexOfFirstDof();
  m_CONFIG_INDEX_ARM_RIGTH_ELBOW =
      m_Robot->getJoint(m_JOINT_ARM_RIGTH_ELBOW)->getIndexOfFirstDof();
  m_CONFIG_INDEX_ARM_RIGTH_WRIST =
      m_Robot->getJoint(m_JOINT_ARM_RIGTH_WRIST)->getIndexOfFirstDof();

  m_CONFIG_INDEX_ARM_LEFT_SHOULDER =
      m_Robot->getJoint(m_JOINT_ARM_LEFT_SHOULDER)->getIndexOfFirstDof();
  m_CONFIG_INDEX_ARM_LEFT_ELBOW =
      m_Robot->getJoint(m_JOINT_ARM_LEFT_ELBOW)->getIndexOfFirstDof();
  m_CONFIG_INDEX_ARM_LEFT_WRIST =
      m_Robot->getJoint(m_JOINT_ARM_LEFT_WRIST)->getIndexOfFirstDof();
}

void Natural::initNaturalJustin()
{
  m_JOINT_SPINE = JUSTIN_JOINT_SPINE;
  m_JOINT_HEAD = JUSTIN_JOINT_HEAD;

  m_JOINT_ARM_RIGTH_SHOULDER = JUSTIN_JOINT_ARM_RIGTH_SHOULDER;
  m_JOINT_ARM_RIGTH_ELBOW = JUSTIN_JOINT_ARM_RIGTH_ELBOW;
  m_JOINT_ARM_RIGTH_WRIST = JUSTIN_JOINT_ARM_RIGTH_WRIST;

  m_JOINT_ARM_LEFT_SHOULDER = JUSTIN_JOINT_ARM_LEFT_SHOULDER;
  m_JOINT_ARM_LEFT_ELBOW = JUSTIN_JOINT_ARM_LEFT_ELBOW;
  m_JOINT_ARM_LEFT_WRIST = JUSTIN_JOINT_ARM_LEFT_WRIST;

  initConfigIndices();

  configPt q;
  q = p3d_alloc_config(m_Robot->getP3dRobotStruct());

  q[6] = 1.573257;
  q[7] = -22.123896;
  q[8] = 31.659294;
  q[9] = -9.535398;
  q[10] = 0.000000;
  q[11] = 0.000000;
  q[12] = 5.014758;
  q[13] = -66.076698;
  q[14] = -15.044244;
  q[15] = 115.634224;
  q[16] = 93.608658;
  q[17] = -9.540314;
  q[18] = -3.672564;
  q[19] = -15.000000;
  q[20] = -46.000000;
  q[21] = -8.000000;
  q[22] = 119.000000;
  q[23] = 138.000000;
  q[24] = 62.000000;
  q[25] = 29.000000;

  m_q_Confort = confPtr_t(new Configuration(
      m_Robot, p3d_copy_config_deg_to_rad(m_Robot->getP3dRobotStruct(), q)));
}

void Natural::initNaturalAchile()
{
  /***********************************************/
  m_JOINT_SPINE = ACHILE_JOINT_SPINE;
  m_JOINT_HEAD = ACHILE_JOINT_HEAD;

  m_JOINT_ARM_RIGTH_SHOULDER = ACHILE_JOINT_ARM_RIGTH_SHOULDER;
  m_JOINT_ARM_RIGTH_ELBOW = ACHILE_JOINT_ARM_RIGTH_ELBOW;
  m_JOINT_ARM_RIGTH_WRIST = ACHILE_JOINT_ARM_RIGTH_WRIST;

  m_JOINT_ARM_LEFT_SHOULDER = ACHILE_JOINT_ARM_LEFT_SHOULDER;
  m_JOINT_ARM_LEFT_ELBOW = ACHILE_JOINT_ARM_LEFT_ELBOW;
  m_JOINT_ARM_LEFT_WRIST = ACHILE_JOINT_ARM_LEFT_WRIST;

  initConfigIndices();

  configPt q;
  q = p3d_alloc_config(m_Robot->getP3dRobotStruct());

  // Neutral Position NORMAL

  q[18] = 1.39626;

  //	q[18] = 1.33012;
  //	q[19] = 0.365646;
  //	q[20] = -0.12706;
  //	q[21] = 0.525519;
  //	q[22] = 0.17558;
  //	q[23] = -0.342085;
  //	q[24] = 0.0233874;

  q[25] = -1.39626;

  //	q[25] = -1.22784;
  //	q[26] = 0.482584;
  //	q[27] = 0.00436332;
  //	q[28] = -0.368439;
  //	q[29] = -0.210487;
  //	q[30] = 0;
  //	q[31] = -0.0935496;

  m_q_Confort = confPtr_t(new Configuration(
      m_Robot, p3d_copy_config(m_Robot->getP3dRobotStruct(), q)));
  m_Robot->setAndUpdate(*m_q_Confort);

  // Compute the rest posture heights
  m_leftArmCost = true;
  m_armHeightL = getUpperBodyHeigth(false);
  m_leftArmCost = false;
  m_armHeightR = getUpperBodyHeigth(false);

  /***********************************************
   * Wieghts for the Configuration Distance
   */

  p3d_destroy_config(m_Robot->getP3dRobotStruct(), q);
  q = p3d_alloc_config(m_Robot->getP3dRobotStruct());

  // set to 0
  for (int i = 0; i < m_nbDof; i++) {
    q[i] = 0;
  }

  //	q[12] = 100;	// Torso
  //	q[13] = 100;
  //	q[14] = 100;
  //
  //	q[15] = 0;		// Head
  //	q[16] = 0;
  //	q[17] = 0;
  //
  //	q[18] = 5;		// Right Shoulder
  //	q[19] = 1;
  //	q[20] = 1;
  //
  //	q[21] = 1;		// Right Elbow
  //
  //	q[22] = 1;		// Right Wrist
  //	q[23] = 1;
  //	q[24] = 1;
  //
  //	q[25] = 5;		// Left Shoulder
  //	q[26] = 1;
  //	q[27] = 1;
  //
  //	q[28] = 1;		// Left Elbow
  //
  //	q[29] = 1;		// Right Wrist
  //	q[30] = 1;
  //	q[31] = 1;

  q[m_CONFIG_INDEX_SPINE + 0] = 100;  // Torso
  q[m_CONFIG_INDEX_SPINE + 1] = 100;
  q[m_CONFIG_INDEX_SPINE + 2] = 100;

  q[m_CONFIG_INDEX_HEAD + 0] = 0;  // Head
  q[m_CONFIG_INDEX_HEAD + 1] = 0;
  q[m_CONFIG_INDEX_HEAD + 2] = 0;

  q[m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 0] = 5;  // Right Shoulder
  q[m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 1] = 1;
  q[m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 2] = 1;

  q[m_JOINT_ARM_RIGTH_ELBOW] = 1;  // Right Elbow

  q[m_CONFIG_INDEX_ARM_RIGTH_WRIST + 0] = 1;  // Right Wrist
  q[m_CONFIG_INDEX_ARM_RIGTH_WRIST + 1] = 1;
  q[m_CONFIG_INDEX_ARM_RIGTH_WRIST + 2] = 1;

  q[m_CONFIG_INDEX_ARM_LEFT_SHOULDER + 0] = 5;  // Left Shoulder
  q[m_CONFIG_INDEX_ARM_LEFT_SHOULDER + 1] = 1;
  q[m_CONFIG_INDEX_ARM_LEFT_SHOULDER + 2] = 1;

  q[m_CONFIG_INDEX_ARM_LEFT_ELBOW] = 1;  // Left Elbow

  q[m_CONFIG_INDEX_ARM_LEFT_WRIST + 0] = 1;  // Right Wrist
  q[m_CONFIG_INDEX_ARM_LEFT_WRIST + 1] = 1;
  q[m_CONFIG_INDEX_ARM_LEFT_WRIST + 2] = 1;

  m_q_ConfortWeigths = confPtr_t(new Configuration(
      m_Robot, p3d_copy_config(m_Robot->getP3dRobotStruct(), q)));

  m_mg.push_back(30);
  m_mg.push_back(4);
  m_mg.push_back(1);
}
/**
 ACHILE_JOINT_SPINE = 12
 ACHILE_JOINT_HEAD = 15
 ACHILE_JOINT_ARM_RIGTH_SHOULDER =  18
 ACHILE_JOINT_ARM_RIGTH_ELBOW =  21
 ACHILE_JOINT_ARM_RIGTH_WRIST =  22
 ACHILE_JOINT_ARM_LEFT_SHOULDER = 25
 ACHILE_JOINT_ARM_LEFT_ELBOW =  28
 ACHILE_JOINT_ARM_LEFT_WRIST =  29
 */

void Natural::initNaturalHerakles()
{
  /***********************************************/
  m_JOINT_SPINE = HERAKLES_JOINT_SPINE;
  m_JOINT_HEAD = HERAKLES_JOINT_HEAD;

  m_JOINT_ARM_RIGTH_SHOULDER = HERAKLES_JOINT_ARM_RIGTH_SHOULDER;
  m_JOINT_ARM_RIGTH_ELBOW = HERAKLES_JOINT_ARM_RIGTH_ELBOW;
  m_JOINT_ARM_RIGTH_WRIST = HERAKLES_JOINT_ARM_RIGTH_WRIST;

  m_JOINT_ARM_LEFT_SHOULDER = HERAKLES_JOINT_ARM_LEFT_SHOULDER;
  m_JOINT_ARM_LEFT_ELBOW = HERAKLES_JOINT_ARM_LEFT_ELBOW;
  m_JOINT_ARM_LEFT_WRIST = HERAKLES_JOINT_ARM_LEFT_WRIST;

  initConfigIndices();

  configPt q;
  q = p3d_alloc_config(m_Robot->getP3dRobotStruct());

  // Neutral Position NORMAL
  q[18] = 1.19817;
  q[19] = 0.106116;
  q[20] = -0.346273;
  q[21] = 0;
  q[22] = 0.479442;
  q[27] = -1.25158;
  q[28] = -0.106116;
  q[29] = -0.266337;
  q[30] = 0;
  q[31] = -0.452389;
  q[32] = 0;
  q[33] = -0.252898;

  m_q_Confort = confPtr_t(new Configuration(
      m_Robot, p3d_copy_config(m_Robot->getP3dRobotStruct(), q)));
  m_Robot->setAndUpdate(*m_q_Confort);

  // Compute the rest posture heights
  m_leftArmCost = true;
  m_armHeightL = getUpperBodyHeigth(false);
  m_leftArmCost = false;
  m_armHeightR = getUpperBodyHeigth(false);

  // Wieghts for the Configuration Distance
  p3d_destroy_config(m_Robot->getP3dRobotStruct(), q);
  q = p3d_alloc_config(m_Robot->getP3dRobotStruct());

  // set to 0
  for (int i = 0; i < m_nbDof; i++) {
    q[i] = 0;
  }

  q[m_CONFIG_INDEX_SPINE + 0] = 100;  // Torso
  q[m_CONFIG_INDEX_SPINE + 1] = 100;
  q[m_CONFIG_INDEX_SPINE + 2] = 100;

  q[m_CONFIG_INDEX_HEAD + 0] = 0;  // Head
  q[m_CONFIG_INDEX_HEAD + 1] = 0;
  q[m_CONFIG_INDEX_HEAD + 2] = 0;

  q[m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 0] = 5;  // Right Shoulder
  q[m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 1] = 1;
  q[m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 2] = 1;

  q[m_JOINT_ARM_RIGTH_ELBOW] = 1;  // Right Elbow

  q[m_CONFIG_INDEX_ARM_RIGTH_WRIST + 0] = 1;  // Right Wrist
  q[m_CONFIG_INDEX_ARM_RIGTH_WRIST + 1] = 1;
  q[m_CONFIG_INDEX_ARM_RIGTH_WRIST + 2] = 1;

  q[m_CONFIG_INDEX_ARM_LEFT_SHOULDER + 0] = 5;  // Left Shoulder
  q[m_CONFIG_INDEX_ARM_LEFT_SHOULDER + 1] = 1;
  q[m_CONFIG_INDEX_ARM_LEFT_SHOULDER + 2] = 1;

  q[m_CONFIG_INDEX_ARM_LEFT_ELBOW] = 1;  // Left Elbow

  q[m_CONFIG_INDEX_ARM_LEFT_WRIST + 0] = 1;  // Right Wrist
  q[m_CONFIG_INDEX_ARM_LEFT_WRIST + 1] = 1;
  q[m_CONFIG_INDEX_ARM_LEFT_WRIST + 2] = 1;

  m_q_ConfortWeigths = confPtr_t(new Configuration(
      m_Robot, p3d_copy_config(m_Robot->getP3dRobotStruct(), q)));

  m_mg.push_back(30);
  m_mg.push_back(4);
  m_mg.push_back(1);
}

void Natural::initNaturalBiomech()
{
  /***********************************************/
  m_JOINT_SPINE = BIOMECH_JOINT_SPINE;
  m_JOINT_HEAD = BIOMECH_JOINT_HEAD;

  m_JOINT_ARM_RIGTH_SHOULDER = BIOMECH_JOINT_ARM_RIGTH_SHOULDER;
  m_JOINT_ARM_RIGTH_ELBOW = BIOMECH_JOINT_ARM_RIGTH_ELBOW;
  m_JOINT_ARM_RIGTH_WRIST = BIOMECH_JOINT_ARM_RIGTH_WRIST;

  m_JOINT_ARM_LEFT_SHOULDER = BIOMECH_JOINT_ARM_LEFT_SHOULDER;
  m_JOINT_ARM_LEFT_ELBOW = BIOMECH_JOINT_ARM_LEFT_ELBOW;
  m_JOINT_ARM_LEFT_WRIST = BIOMECH_JOINT_ARM_LEFT_WRIST;

  initConfigIndices();

  m_active_dofs_.clear();
  m_active_dofs_.push_back(m_CONFIG_INDEX_SPINE + 0);  // Torso
  m_active_dofs_.push_back(m_CONFIG_INDEX_SPINE + 1);
  m_active_dofs_.push_back(m_CONFIG_INDEX_SPINE + 2);
  m_active_dofs_.push_back(m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 0);
  m_active_dofs_.push_back(m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 1);
  m_active_dofs_.push_back(m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 2);
  m_active_dofs_.push_back(m_CONFIG_INDEX_ARM_RIGTH_ELBOW + 0);  // Right Elbow
  m_active_dofs_.push_back(m_CONFIG_INDEX_ARM_RIGTH_ELBOW + 1);
  m_active_dofs_.push_back(m_CONFIG_INDEX_ARM_RIGTH_ELBOW + 2);
  m_active_dofs_.push_back(m_CONFIG_INDEX_ARM_RIGTH_WRIST + 0);  // Right Wrist
  m_active_dofs_.push_back(m_CONFIG_INDEX_ARM_RIGTH_WRIST + 1);
  m_active_dofs_.push_back(m_CONFIG_INDEX_ARM_RIGTH_WRIST + 2);

  if (HriEnv->getBool(HricsParam::ioc_set_dof_limits_from_experiments)) {
    setBiomechJointLimitsFromFile("");
  }

  Configuration q(*m_Robot->getNewConfig());

  //  Joint(8), Dof : 18, rShoulderTransX , (min = 0.0340089, max = 0.146555)
  //  Joint(9), Dof : 19, rShoulderTransY , (min = 0.0857289, max = 0.257622)
  //  Joint(10), Dof : 20, rShoulderTransZ , (min = -0.0822057, max = 0.182777)
  //  Joint(11), Dof : 21, rShoulderY1 , (min = -1.94149, max = 2.80703)
  //  Joint(12), Dof : 22, rShoulderX , (min = -1.75369, max = -0.00176006)
  //  Joint(13), Dof : 23, rShoulderY2 , (min = -2.09138, max = 2.65664)
  //  Joint(14), Dof : 24, rArmTrans , (min = 0.278929, max = 0.462408)
  //  Joint(15), Dof : 25, rElbowZ , (min = 0.28148, max = 2.01673)
  //  Joint(16), Dof : 26, rElbowX , (min = -0.608768, max = 0.32509)
  //  Joint(17), Dof : 27, rElbowY , (min = -3.14125, max = 3.14119)
  //  Joint(18), Dof : 28, rForeArmTrans , (min = 0.177062, max = 0.315494)
  //  Joint(19), Dof : 29, rWristZ , (min = -2.4236, max = 2.96851)
  //  Joint(20), Dof : 30, rWristX , (min = -1.38323, max = 1.49798)
  //  Joint(21), Dof : 31, rWristY , (min = -2.87235, max = 2.82653)
  //  Joint(22), Dof : 32, lShoulderX , (min = -1.5708, max = -1.5708)

  // Neutral Position NORMAL
  q[18] = 0.05;
  q[19] = 0.20;
  q[20] = 0.15;
  q[21] = -.593;
  q[22] = -.24;
  q[23] = 1.65;
  q[24] = 0.396;
  q[25] = 0.30;
  q[26] = -0.267664;
  q[27] = -0.245044;
  q[28] = 0.246;
  q[29] = 0.0892212;
  q[30] = -0.155823;
  q[31] = 0.111212;
  q[32] = -1.38796;

  //  q[m_Robot->getJoint("rShoulderTransX")->getIndexOfFirstDof()] = .018;
  //  q[m_Robot->getJoint("rShoulderTransY")->getIndexOfFirstDof()] = .33;
  //  q[m_Robot->getJoint("rShoulderTransZ")->getIndexOfFirstDof()] = .25;
  //  q[m_Robot->getJoint("rArmTrans")->getIndexOfFirstDof()] = .39;
  //  q[m_Robot->getJoint("lPoint")->getIndexOfFirstDof()] = .24;

  //    p3d_jnt_set_dof_rand_bounds( robot->getJoint( "rShoulderTransX"
  //    )->getP3dJointStruct(), 0, .016, .020 );
  //    p3d_jnt_set_dof_rand_bounds( robot->getJoint( "rShoulderTransY"
  //    )->getP3dJointStruct(), 0, .32, .34 );
  //    p3d_jnt_set_dof_rand_bounds( robot->getJoint( "rShoulderTransZ"
  //    )->getP3dJointStruct(), 0, .24, .26 );
  //    p3d_jnt_set_dof_rand_bounds( robot->getJoint( "rArmTrans"
  //    )->getP3dJointStruct(), 0, .38, .40 );
  //    p3d_jnt_set_dof_rand_bounds( robot->getJoint( "lPoint"
  //    )->getP3dJointStruct(), 0, .23, .25 );

  m_q_Confort = q.copy();
  // m_Robot->setInitPos(*m_q_Confort); // TODO remove q_init only for saving
  m_Robot->setAndUpdate(*m_q_Confort);

  // Compute the rest posture heights
  m_leftArmCost = true;
  m_armHeightL = getUpperBodyHeigth(false);
  m_leftArmCost = false;
  m_armHeightR = getUpperBodyHeigth(false);

  // Wieghts for the Configuration Distance
  q = *m_Robot->getNewConfig();

  // Set to 0
  for (int i = 0; i < m_nbDof; i++) {
    q[i] = 0;
  }

  q[m_CONFIG_INDEX_SPINE + 0] = 10;  // Torso
  q[m_CONFIG_INDEX_SPINE + 1] = 5;
  q[m_CONFIG_INDEX_SPINE + 2] = 5;

  q[m_CONFIG_INDEX_HEAD + 0] = 0;  // Head
  q[m_CONFIG_INDEX_HEAD + 1] = 0;
  q[m_CONFIG_INDEX_HEAD + 2] = 0;

  q[m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 0] = 10;  // Right Shoulder
  q[m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 1] = 10;
  q[m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 2] = 10;

  q[m_CONFIG_INDEX_ARM_RIGTH_ELBOW + 0] = 10;  // Right Elbow
  q[m_CONFIG_INDEX_ARM_RIGTH_ELBOW + 1] = 10;
  q[m_CONFIG_INDEX_ARM_RIGTH_ELBOW + 2] = 10;

  q[m_CONFIG_INDEX_ARM_RIGTH_WRIST + 0] = 20;  // Right Wrist
  q[m_CONFIG_INDEX_ARM_RIGTH_WRIST + 1] = 20;
  q[m_CONFIG_INDEX_ARM_RIGTH_WRIST + 2] = 20;

  q[m_CONFIG_INDEX_ARM_LEFT_SHOULDER + 0] = 1;  // Left Shoulder
  q[m_CONFIG_INDEX_ARM_LEFT_SHOULDER + 1] = 1;
  q[m_CONFIG_INDEX_ARM_LEFT_SHOULDER + 2] = 1;

  q[m_CONFIG_INDEX_ARM_LEFT_ELBOW] = 1;  // Left Elbow

  q[m_CONFIG_INDEX_ARM_LEFT_WRIST + 0] = 1;  // Right Wrist
  q[m_CONFIG_INDEX_ARM_LEFT_WRIST + 1] = 1;
  q[m_CONFIG_INDEX_ARM_LEFT_WRIST + 2] = 1;

  m_q_ConfortWeigths = q.copy();

  m_mg.push_back(30);
  m_mg.push_back(4);
  m_mg.push_back(1);
}

void Natural::initNaturalOldDude()
{
  /***********************************************/
  m_JOINT_SPINE = OLDDUDE_JOINT_SPINE;
  m_JOINT_HEAD = OLDDUDE_JOINT_HEAD;

  m_JOINT_ARM_RIGTH_SHOULDER = OLDDUDE_JOINT_ARM_RIGTH_SHOULDER;
  m_JOINT_ARM_RIGTH_ELBOW = OLDDUDE_JOINT_ARM_RIGTH_ELBOW;
  m_JOINT_ARM_RIGTH_WRIST = OLDDUDE_JOINT_ARM_RIGTH_WRIST;

  m_JOINT_ARM_LEFT_SHOULDER = OLDDUDE_JOINT_ARM_LEFT_SHOULDER;
  m_JOINT_ARM_LEFT_ELBOW = OLDDUDE_JOINT_ARM_LEFT_ELBOW;
  m_JOINT_ARM_LEFT_WRIST = OLDDUDE_JOINT_ARM_LEFT_WRIST;

  initConfigIndices();

  configPt q;
  q = p3d_alloc_config(m_Robot->getP3dRobotStruct());

  // Neutral Position NORMAL

  q[18] = 1.39626;
  q[25] = -1.39626;

  m_q_Confort = confPtr_t(new Configuration(
      m_Robot, p3d_copy_config(m_Robot->getP3dRobotStruct(), q)));
  m_Robot->setAndUpdate(*m_q_Confort);

  // Compute the rest posture heights
  m_leftArmCost = true;
  m_armHeightL = getUpperBodyHeigth(false);
  m_leftArmCost = false;
  m_armHeightR = getUpperBodyHeigth(false);

  // Wieghts for the Configuration Distance
  p3d_destroy_config(m_Robot->getP3dRobotStruct(), q);
  q = p3d_alloc_config(m_Robot->getP3dRobotStruct());

  // set to 0
  for (int i = 0; i < m_nbDof; i++) {
    q[i] = 0;
  }

  q[m_CONFIG_INDEX_SPINE + 0] = 100;  // Torso
  q[m_CONFIG_INDEX_SPINE + 1] = 100;
  q[m_CONFIG_INDEX_SPINE + 2] = 100;

  q[m_CONFIG_INDEX_HEAD + 0] = 0;  // Head
  q[m_CONFIG_INDEX_HEAD + 1] = 0;
  q[m_CONFIG_INDEX_HEAD + 2] = 0;

  q[m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 0] = 5;  // Right Shoulder
  q[m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 1] = 1;
  q[m_CONFIG_INDEX_ARM_RIGTH_SHOULDER + 2] = 1;

  q[m_JOINT_ARM_RIGTH_ELBOW] = 1;  // Right Elbow

  q[m_CONFIG_INDEX_ARM_RIGTH_WRIST + 0] = 1;  // Right Wrist
  q[m_CONFIG_INDEX_ARM_RIGTH_WRIST + 1] = 1;
  q[m_CONFIG_INDEX_ARM_RIGTH_WRIST + 2] = 1;

  q[m_CONFIG_INDEX_ARM_LEFT_SHOULDER + 0] = 5;  // Left Shoulder
  q[m_CONFIG_INDEX_ARM_LEFT_SHOULDER + 1] = 1;
  q[m_CONFIG_INDEX_ARM_LEFT_SHOULDER + 2] = 1;

  q[m_CONFIG_INDEX_ARM_LEFT_ELBOW] = 1;  // Left Elbow

  q[m_CONFIG_INDEX_ARM_LEFT_WRIST + 0] = 1;  // Right Wrist
  q[m_CONFIG_INDEX_ARM_LEFT_WRIST + 1] = 1;
  q[m_CONFIG_INDEX_ARM_LEFT_WRIST + 2] = 1;

  m_q_ConfortWeigths = confPtr_t(new Configuration(
      m_Robot, p3d_copy_config(m_Robot->getP3dRobotStruct(), q)));

  m_mg.push_back(30);
  m_mg.push_back(4);
  m_mg.push_back(1);
}

void Natural::setBiomechJointLimitsFromFile(std::string filename)
{
  //  std::ostringstream ss;
  //  ss << "/usr/local/jim_local/Dropbox/move3d/catkin_ws_move3d/src/"
  //     << "hrics-or-rafi/"
  //     << "python_module/bioik/user_experiment_data/selection/";

  std::stringstream ss;
  ss.str("");
  ss << std::string(getenv("HOME_MOVE3D")) + "/../move3d-launch/launch_files/";
  ss << "dof_limits/";

  test_set dataset = test_set(HriEnv->getInt(HricsParam::ioc_dataset));

  // Get joint limits from files
  Eigen::VectorXd max_v, min_v;
  if (dataset == icra_paper_sept || dataset == icra_paper_feb) {
    max_v = move3d_load_matrix_from_csv_file(ss.str() + "icra_max_dof.csv");
    min_v = move3d_load_matrix_from_csv_file(ss.str() + "icra_min_dof.csv");
  } else {
    max_v = move3d_load_matrix_from_csv_file(ss.str() + "max_dof.csv");
    min_v = move3d_load_matrix_from_csv_file(ss.str() + "min_dof.csv");
  }

  cout << "max_v size : " << max_v.size() << endl;
  cout << "min_v size : " << min_v.size() << endl;

  HRICS::RecordMotion motion_recorder(m_Robot);
  motion_recorder.useBioFormat(true);

  std::pair<double, confPtr_t> q_max = motion_recorder.getConfigBio(max_v);
  std::pair<double, confPtr_t> q_min = motion_recorder.getConfigBio(min_v);

  for (size_t i = 0; i < m_Robot->getNumberOfJoints(); i++) {
    for (size_t j = 0; j < m_Robot->getJoint(i)->getNumberOfDof(); j++) {
      Joint* move3d_joint = m_Robot->getJoint(i);
      int dof_index = move3d_joint->getIndexOfFirstDof() + j;

      const bool only_treat_active_dofs = true;
      if (only_treat_active_dofs &&
          std::find(m_active_dofs_.begin(), m_active_dofs_.end(), dof_index) ==
              m_active_dofs_.end()) {
        continue;
      }

      double min, max;
      move3d_joint->getDofRandBounds(j, min, max);

      const bool print_group = true;
      if (print_group) {
        cout << "Joint(" << i << "), Dof : " << dof_index << ", "
             << move3d_joint->getName() << " , ";
        cout << "(min = " << min << ", max = " << max << ")";
        cout << " , limits :  " << (*q_max.second)[dof_index];
        cout << " , " << (*q_min.second)[dof_index];
        cout << " , Is dof angular :  " << move3d_joint->isJointDofAngular(j);
        cout << " , Is dof circular : " << move3d_joint->isJointDofCircular(j);
        cout << endl;
      }

      // Check that it does not cover the whole joint
      if (move3d_joint->isJointDofCircular(j) &&
          std::fabs((*q_max.second)[dof_index] - M_PI) < 1e-3 &&
          std::fabs((*q_min.second)[dof_index] + M_PI) < 1e-3) {
        continue;
      }

      double slack = 0.05 * std::fabs((*q_max.second)[dof_index] -
                                      (*q_min.second)[dof_index]);

      p3d_jnt_set_dof_rand_bounds(move3d_joint->getP3dJointStruct(),
                                  j,
                                  (*q_min.second)[dof_index] - slack,
                                  (*q_max.second)[dof_index] + slack);

      p3d_jnt_set_dof_bounds(move3d_joint->getP3dJointStruct(),
                             j,
                             (*q_min.second)[dof_index] - slack,
                             (*q_max.second)[dof_index] + slack);

      cout << "new bounds : " << (*q_min.second)[dof_index] - slack << " , "
           << (*q_max.second)[dof_index] + slack << endl;
    }
  }

  m_q_limits_max = q_max.second;
  m_q_limits_min = q_min.second;
}

void Natural::initHumanBaseGrid(vector<double> box)
{
  //    cout << "Natural::initHumanBaseGrid()" << endl;

  vector<double> envSize(6);

  // configPt q = p3d_get_robot_config(m_Agents->humans[0]->robotPt);

  envSize[0] = /**(q+6) +*/ box[0];
  envSize[1] = /**(q+6) +*/ box[1];
  envSize[2] = /**(q+7) +*/ box[2];
  envSize[3] = /**(q+7) +*/ box[3];
  envSize[4] = /**(q+8) +*/ box[4];
  envSize[5] = /**(q+8) +*/ box[5];

  m_Grid = new NaturalGrid(0.1, envSize, this);
}

void Natural::printBodyPos()
{
  if (m_KinType == Achile) {
    //		cout << "ACHILE_JOINT_SPINE = " << m_Robot->getJoint(
    // ACHILE_JOINT_SPINE )->index_dof << endl ;
    //		cout << "ACHILE_JOINT_HEAD = " << m_Robot->getJoint(
    // ACHILE_JOINT_HEAD
    //)->index_dof << endl ;
    //
    //		cout << "ACHILE_JOINT_ARM_RIGTH_SHOULDER =  " <<
    // m_Robot->getJoint(
    // ACHILE_JOINT_ARM_RIGTH_SHOULDER )->index_dof << endl ;
    //		cout << "ACHILE_JOINT_ARM_RIGTH_ELBOW =  " << m_Robot->getJoint(
    // ACHILE_JOINT_ARM_RIGTH_ELBOW )->index_dof << endl ;
    //		cout << "ACHILE_JOINT_ARM_RIGTH_WRIST =  " << m_Robot->getJoint(
    // ACHILE_JOINT_ARM_RIGTH_WRIST )->index_dof << endl ;
    //
    //		cout << "ACHILE_JOINT_ARM_LEFT_SHOULDER = " <<
    // m_Robot->getJoint(
    // ACHILE_JOINT_ARM_LEFT_SHOULDER )->index_dof << endl ;
    //		cout << "ACHILE_JOINT_ARM_LEFT_ELBOW =  " << m_Robot->getJoint(
    // ACHILE_JOINT_ARM_LEFT_ELBOW )->index_dof << endl ;
    //		cout << "ACHILE_JOINT_ARM_LEFT_WRIST =  " << m_Robot->getJoint(
    // ACHILE_JOINT_ARM_LEFT_WRIST )->index_dof << endl ;

    if (PointsToDraw != NULL) {
      PointsToDraw->clear();

      if (PointsToDraw->size() == 0) {
        // Vector3d point(Vector3d::Zero());

        PointsToDraw->push_back(
            m_Robot->getJoint(m_JOINT_SPINE)->getVectorPos());
        PointsToDraw->push_back(
            m_Robot->getJoint(m_JOINT_HEAD)->getVectorPos());

        PointsToDraw->push_back(
            m_Robot->getJoint(m_JOINT_ARM_RIGTH_SHOULDER)->getVectorPos());
        PointsToDraw->push_back(
            m_Robot->getJoint(m_JOINT_ARM_RIGTH_ELBOW)->getVectorPos());
        PointsToDraw->push_back(
            m_Robot->getJoint(m_JOINT_ARM_RIGTH_WRIST)->getVectorPos());

        PointsToDraw->push_back(
            m_Robot->getJoint(m_JOINT_ARM_LEFT_SHOULDER)->getVectorPos());
        PointsToDraw->push_back(
            m_Robot->getJoint(m_JOINT_ARM_LEFT_ELBOW)->getVectorPos());
        PointsToDraw->push_back(
            m_Robot->getJoint(m_JOINT_ARM_LEFT_WRIST)->getVectorPos());
      }
    }
  }
}

void Natural::setRobotToConfortPosture()
{
  confPtr_t q_cur = m_Robot->getCurrentPos();

  m_IndexObjectDof = 6;
  (*m_q_Confort)[m_IndexObjectDof + 0] = (*q_cur)[m_IndexObjectDof + 0];
  (*m_q_Confort)[m_IndexObjectDof + 1] = (*q_cur)[m_IndexObjectDof + 1];
  (*m_q_Confort)[m_IndexObjectDof + 2] = (*q_cur)[m_IndexObjectDof + 2];
  (*m_q_Confort)[m_IndexObjectDof + 3] = (*q_cur)[m_IndexObjectDof + 3];
  (*m_q_Confort)[m_IndexObjectDof + 4] = (*q_cur)[m_IndexObjectDof + 4];
  (*m_q_Confort)[m_IndexObjectDof + 5] = (*q_cur)[m_IndexObjectDof + 5];

  m_Robot->setAndUpdate(*m_q_Confort);
}

//! Get the elemetary cost features
void Natural::getAllConfigFeatures(Eigen::VectorXd& features)
{
  confPtr_t q = m_Robot->getCurrentPos();
  getCustomDistConfig(*q, features);
}

//! Get the elemetary cost features
void Natural::getConfigCostFeatures(Eigen::VectorXd& features)
{
  features = Eigen::VectorXd::Zero(3);
  // confPtr_t q_Actual = m_Robot->getCurrentPos();

  // cout << "-------------------------------" << endl;
  //---------------------------------------------------
  // Joints Displacement
  double c_f_Joint_displacement = getJointDisplacement();

  //---------------------------------------------------
  // Energy
  double c_f_Energy = 0.1 * getEnergy();

  //---------------------------------------------------
  // Discomfort
  double c_f_Discomfort = 0.1 * getDiscomfort();

  //---------------------------------------------------

  features[0] = c_f_Joint_displacement;
  features[1] = c_f_Energy;
  features[2] = c_f_Discomfort;

  //    cout.precision(3);
  //    cout << std::scientific << "JDis = " << c_f_Joint_displacement;
  //    cout << std::scientific << " , Ener = " << c_f_Energy;
  //    cout << std::scientific << " , Disc = " << c_f_Discomfort << endl;
}

double Natural::getConfigCost()
{
  Move3D::confPtr_t q = m_Robot->getCurrentPos();
  return cost(*q);
}

/*!
 * Compute the Natural cost for a configuration
 * with weight
 */
double Natural::cost(Move3D::Configuration& q)
{
  double c_natural = 0.0;

  Eigen::VectorXd phi(Eigen::VectorXd::Zero(3));

  getConfigCostFeatures(phi);

  /**
   * Wieghted sum
   */
  double W_jd = ENV.getDouble(Env::coeffJoint);
  double W_en = ENV.getDouble(Env::coeffEnerg);
  double W_di = ENV.getDouble(Env::coeffConfo);

  c_natural = W_jd * phi[0] + W_en * phi[1] + W_di * phi[2];

  const bool print_cost = false;
  if (print_cost) {
    cout << "--------------------------" << endl;
    cout << "features : " << phi.transpose() << endl;
    cout << "cost1 (joint) : " << W_jd* phi[0] << endl;
    cout << "cost2 (energ) : " << W_en* phi[1] << endl;
    cout << "cost3 (confo) : " << W_di* phi[2] << endl;
    cout << "total natural cost : " << c_natural << endl;
  }

  // m_leftArmCost = true;
  // c_natural = basicNaturalArmCost(m_leftArmCost);
  //	cout << "NaturalCost = " << c_natural << endl;
  return c_natural;
}

/*!
 * Joint-displacement : This function evaluates the joint displacement
 * from the VRS project at Iwoa
 */
double Natural::getJointDisplacement()
{
  Eigen::VectorXd phi;
  return getCustomDistConfig(*m_Robot->getCurrentPos(), phi);
}

/*!
 * Energy : This function evaluates the joint displacement
 * from the VRS project at Iwoa
 */
double Natural::getEnergy()
{
  double Energy = 0.0;

  vector<double> DeltaHeigth = getUpperBodyHeigth();

  // 0 -> shoulder, 1 -> elbow, 2 -> wrist
  for (unsigned int i = 1; i < DeltaHeigth.size(); i++) {
    if (m_mg[i] > 0) {
      Energy += pow(m_mg[i], 2) * pow(DeltaHeigth[i], 2);
    }
  }

  return Energy;
}

/*!
 * Discomfort : This function evaluates the joint displacement
 * from the VRS project at Iwoa
 */
double Natural::getDiscomfort()
{
  return getJointLimits(*m_Robot->getCurrentPos());
}

/*!
 * Discomfort : This function evaluates the different in heigth
 * of each body regarding the confort position from the VRS project at Iwoa
 */
vector<double> Natural::getUpperBodyHeigth(bool useReference)
{
  int ShoulJoint, ElbowJoint, WristJoint;

  double L0 = 0.0;
  double L1 = 0.0;
  double L2 = 0.0;

  if (m_leftArmCost) {
    ShoulJoint = m_JOINT_ARM_LEFT_SHOULDER;
    ElbowJoint = m_JOINT_ARM_LEFT_ELBOW;
    WristJoint = m_JOINT_ARM_LEFT_WRIST;

    if (!m_armHeightL.empty()) {
      L0 = m_armHeightL[0];
      L1 = m_armHeightL[0] - m_armHeightL[1];
      L2 = m_armHeightL[0] - m_armHeightL[2];
    }
  } else {
    ShoulJoint = m_JOINT_ARM_RIGTH_SHOULDER;
    ElbowJoint = m_JOINT_ARM_RIGTH_ELBOW;
    WristJoint = m_JOINT_ARM_RIGTH_WRIST;

    if (!m_armHeightR.empty()) {
      L0 = m_armHeightR[0];
      L1 = m_armHeightR[0] - m_armHeightR[1];
      L2 = m_armHeightR[0] - m_armHeightR[2];
    }
  }

  vector<double> heigths;

  if (useReference) {
    heigths.push_back(L0 - m_Robot->getJoint(ShoulJoint)->getVectorPos()(2));
    heigths.push_back(L1 - m_Robot->getJoint(ShoulJoint)->getVectorPos()(2) +
                      m_Robot->getJoint(ElbowJoint)->getVectorPos()(2));
    heigths.push_back(L2 - m_Robot->getJoint(ShoulJoint)->getVectorPos()(2) +
                      m_Robot->getJoint(WristJoint)->getVectorPos()(2));
  } else {
    heigths.push_back(m_Robot->getJoint(ShoulJoint)->getVectorPos()(2));
    heigths.push_back(m_Robot->getJoint(ElbowJoint)->getVectorPos()(2));
    heigths.push_back(m_Robot->getJoint(WristJoint)->getVectorPos()(2));
  }

  return heigths;
}

/*!
 * Compute the classic square distance between two configurations
 * with weight
 *
 * Input:  The robot,
 *         the two configurations
 *
 * See : p3d_dist_config
 */
double Natural::getCustomDistConfig(Configuration& q, Eigen::VectorXd& phi)
{
  double l = 0., ljnt = 0.;
  Eigen::VectorXd phi_tmp = Eigen::VectorXd::Zero(m_nbDof);
  Eigen::VectorXd w = phi_tmp;  // Set to 0

  for (int i = 0; i < int(m_Robot->getJoints().size()); i++) {
    // Get move3d joint
    Joint* move3d_joint = m_Robot->getJoint(i);

    for (size_t j = 0; j < move3d_joint->getNumberOfDof(); j++) {
      double min, max;
      // robot_->getJoint( active_joints[i] )->getDofBounds(j,min,max);
      move3d_joint->getDofRandBounds(j, min, max);
      double range = max - min;

      if (range > 0.) {
        double dof_dist =
            p3d_jnt_calc_dof_dist(move3d_joint->getP3dJointStruct(),
                                  j,
                                  m_q_Confort->getConfigStruct(),
                                  q.getConfigStruct());
        int k = move3d_joint->getIndexOfFirstDof() + j;

        phi_tmp[k] = std::fabs(dof_dist / range);  // Already normalized here

        const bool print_dist = false;
        if (print_dist) {
          cout << "name : " << move3d_joint->getName() << " , range[" << k
               << "] : " << range << " , phi[" << k << "] : " << phi_tmp[k]
               << " , w[" << k << "] : " << (*m_q_ConfortWeigths)[k] << endl;
        }
      }
      // printf( "dof %s dist[%d] = %f\n",jntPt->getName().c_str(),
      // jntPt->getIndexOfFirstDof()+j, dof_dist);
    }
  }

  // Normalize (TODO: should not be there !!!)
  for (int i = 0; i < phi_tmp.size(); i++) {
    w[i] = (*m_q_ConfortWeigths)[i];
  }
  phi_tmp /= double(phi_tmp.size());
  l = w.transpose() * phi_tmp;

  // Set only active dofs in the phi vector
  phi = Eigen::VectorXd::Zero(m_active_dofs_.size());
  for (int i = 0; i < phi.size(); i++) {
    phi[i] = phi_tmp[m_active_dofs_[i]];
  }
  // l = sqrt(ljnt);
  // printf( "total dist = %f\n", l );

  return l;
}

double Natural::getJointLimits(Configuration& q)
{
  double QU, QL;
  QU = QL = 0.0;
  double Cost = 0.0;

  for (unsigned int i = 0; i < m_Robot->getNumberOfJoints(); i++) {
    Joint* jntPt = m_Robot->getJoint(i);

    for (unsigned int j = 0; j < jntPt->getNumberOfDof(); j++) {
      unsigned int k = jntPt->getIndexOfFirstDof() + j;

      double q_min, q_max;

      jntPt->getDofBounds(j, q_min, q_max);

      if (q_max == q_min) {
        continue;
      }

      double deltaU = 5.0 * (q_max - q[k]) / (q_max - q_min);
      double deltaL = 5.0 * (q[k] - q_min) / (q_max - q_min);

      //			cout << "deltaU = " << deltaU << endl;
      //			cout << "deltaL = " << deltaL << endl;

      QU = pow(0.5 * (sin(deltaU + 1.571) + 1), 100);
      QL = pow(0.5 * (sin(deltaL + 1.571) + 1), 100);

      //			cout << "QU = " << QU << endl;
      //			cout << "QL = " << QL << endl;

      Cost += (QU + QL);
    }
  }

  //	cout << "Joint Limits Cost : " << Cost << endl;
  return (/*m_G**/ Cost);
}

/*!
 *
 */
double Natural::basicNaturalArmCost(bool useLeftvsRightArm)
{
  int ShoulderIndex, ElbowIndex;
  int ShoulderJoint, ElbowJoint, WristJoint;

  if (useLeftvsRightArm) {
    ShoulderJoint = m_JOINT_ARM_LEFT_SHOULDER;
    ElbowJoint = m_JOINT_ARM_LEFT_ELBOW;
    WristJoint = m_JOINT_ARM_LEFT_WRIST;
  } else {
    ShoulderJoint = m_JOINT_ARM_RIGTH_SHOULDER;
    ElbowJoint = m_JOINT_ARM_RIGTH_ELBOW;
    WristJoint = m_JOINT_ARM_RIGTH_WRIST;
  }

  ShoulderIndex = m_Robot->getJoint(ShoulderJoint)->getIndexOfFirstDof();
  ElbowIndex = m_Robot->getJoint(ElbowJoint)->getIndexOfFirstDof();

  //	cout << "Get Shoulder index dof = " <<
  // m_Robot->getJoint(ShoulderJoint)->getIndexOfFirstDof() << endl;
  //	cout << "Get Elbow index dof = " <<
  // m_Robot->getJoint(ElbowJoint)->getIndexOfFirstDof() << endl;
  //	cout << "Get Wrist index dof = " <<
  // m_Robot->getJoint(WristJoint)->getIndexOfFirstDof() << endl;

  double deltaJoint = 0, potential = 0;

  double restq1 = (*m_q_Confort)[ShoulderIndex + 0];
  double restq2 = (*m_q_Confort)[ShoulderIndex + 1];
  double restq3 = (*m_q_Confort)[ShoulderIndex + 2];
  double restq4 = (*m_q_Confort)[ElbowIndex + 0];

  //	cout << "-----------------------------------------------" << endl;
  //	cout <<  "restq1 = " << (*m_q_Confort)[ShoulderIndex+0] << endl;
  //	cout <<  "restq2 = " << (*m_q_Confort)[ShoulderIndex+1] << endl;
  //	cout <<  "restq3 = " << (*m_q_Confort)[ShoulderIndex+2] << endl;
  //	cout <<  "restq4 = " << (*m_q_Confort)[ElbowIndex] << endl;

  //	q[18] = 1.33012;	min = -1.74;	max = 1.57; (max) dist = 10.956
  //	q[19] = 0.365646;	min -0.52;		max = 0.52; (min) dist =
  // 15.05
  //	q[20] = -0.12706;	min = -1.57;	max = 2.09; (max) dist = 4.88
  //	q[21] = 0.525519;	min = 0;		max = 2.44; (max) dist =
  // 3.68

  confPtr_t q = m_Robot->getCurrentPos();

  deltaJoint =
      (SQR((*q)[ShoulderIndex + 0] - restq1) +
       SQR((*q)[ShoulderIndex + 1] - restq2) +
       SQR((*q)[ShoulderIndex + 2] - restq3) + SQR((*q)[ElbowIndex] - restq4)) *
      6 / 34.57;  // Weird 6

  potential = (m_Robot->getJoint(ElbowJoint)->getVectorPos()(2) +
               m_Robot->getJoint(WristJoint)->getVectorPos()(2) - 2.43) /
              0.67;

  double Cost = (deltaJoint + potential) / 2;
  // double Cost = deltaJoint;

  //	if (Cost < 0) {
  //		Cost = 1.0;
  //	}
  // cout << "Cost = " << Cost << endl;
  return Cost;
}

/*!
 * Computes the cost for a
 * Workspace point
 * @param useLeftvsRightArm is true for left and false for right
 */
double Natural::getCost(const Vector3d& WSPoint,
                        bool useLeftvsRightArm,
                        bool withEffect)
{
  switch (m_KinType) {
    case Justin:
      if (!m_computeNbOfIK) {
        m_leftArmCost = useLeftvsRightArm;
        return cost(*m_Robot->getCurrentPos());
      } else {
        cout << "Natural::Warning::Computing the Number of ik cost" << endl;
        return getNumberOfIKCost(WSPoint);
      }
      break;

    case Achile:
#ifdef HRI_PLANNER
      if (computeIsReachableOnly(WSPoint, useLeftvsRightArm)) {
        m_leftArmCost = useLeftvsRightArm;
        return cost(*m_Robot->getCurrentPos());
      } else {
        cout << "Warning : compute cost of unreachable point" << endl;
        //			return numeric_limits<double>::max();
        return -1.0;
      }
#endif
      break;

    default:
      cout << "Warning : Not implemented" << endl;
      break;
  }
  return 0.0;
}

/*!
 * Cost of a workspace point
 */
double Natural::getWorkspaceCost(const Eigen::Vector3d& WSPoint)
{
  if (m_Grid->isReachable(WSPoint)) {
    return m_Grid->getCellCostAt(WSPoint);
  }

  return 1.5;
}

/*!
 * Is point reachable in workspace
 */
bool Natural::getWorkspaceIsReachable(const Eigen::Vector3d& WSPoint)
{
  return m_Grid->isReachable(WSPoint);
}

/*!
 * Computes the number of IK
 */
double Natural::getNumberOfIKCost(const Vector3d& WSPoint)
{
  confPtr_t q;

  double Cost = 0.0;
  const unsigned int NbDirections = 360;

  for (unsigned int i = 0; i < NbDirections; i++) {
    q = m_Robot->shoot();

    (*q)[m_IndexObjectDof + 0] = WSPoint[0];
    (*q)[m_IndexObjectDof + 1] = WSPoint[1];
    (*q)[m_IndexObjectDof + 2] = WSPoint[2];

    // q->getConfigStruct()[32] = 0.000000;
    // q->getConfigStruct()[33] = 0.000000;
    // q->getConfigStruct()[34] = -0.785398;

    if (q->setConstraintsWithSideEffect() && !q->isInCollision()) {
      // m_Cost += grid->getNaturalCostSpace()->getCost();
      // cout << "Center :" << endl << center << endl;
      // cout << rob->getName() << endl;
      // m_QStored = q;
      // m_CostIsComputed = true;
      Cost += 1.0;
      // m_QStored->print(true);
      // return 1.0;
    }
  }

  return Cost;
}

/*!
 * Compute Wether a point is reachable for a human
 */
bool Natural::computeIsReachableAndMove(const Vector3d& WSPoint,
                                        bool useLeftvsRightArm)
{
  //#ifdef HRI_PLANNER

  confPtr_t configStored = m_Robot->getCurrentPos();

  bool ik_succeed = false;
  const bool withSideEffect = true;

  if ((m_IsHuman && (m_Agents->humans_no > 0)) ||
      ((!m_IsHuman) && (m_Agents->robots_no > 0)))  // Humans ot Robots
  {
    // 2 - Select Task
    HRI_GIK_TASK_TYPE task =
        useLeftvsRightArm ? GIK_LATREACH : /*GIK_RAREACH*/ GIK_RATREACH;

    p3d_vector3 Tcoord;
    global_DrawnSphere = WSPoint;
    Tcoord[0] = WSPoint[0];
    Tcoord[1] = WSPoint[1];
    Tcoord[2] = WSPoint[2];

    configPt q;
    double distance_tolerance = 0.005;

    m_Robot->setAndUpdate(*m_q_Confort);

    if (m_IsHuman)  // Humans
    {
      q = p3d_get_robot_config(m_Agents->humans[0]->robotPt);
      ik_succeed = hri_agent_single_task_manip_move(
          m_Agents->humans[0], task, &Tcoord, distance_tolerance, &q);
      Configuration Q(m_Robot, q);

      if (true /*ik_succeed*/) {
        m_Robot->setAndUpdate(Q);

        if (!m_Robot->isInCollision()) {
          cout << "Config is OK in " << __PRETTY_FUNCTION__ << endl;
        } else {
          cout << "Config in collision in " << __PRETTY_FUNCTION__ << endl;
          ik_succeed = false;
          m_Robot->setAndUpdate(*configStored);
        }
      } else {
        m_Robot->setAndUpdate(*configStored);
        cout << "IK Failed in " << __PRETTY_FUNCTION__ << endl;
      }
    } else  // Robots
    {
      q = p3d_get_robot_config(m_Agents->robots[0]->robotPt);
      ik_succeed = hri_agent_single_task_manip_move(
          m_Agents->robots[0], task, &Tcoord, distance_tolerance, &q);
      p3d_set_and_update_this_robot_conf(m_Agents->robots[0]->robotPt, q);
    }

  }
  //	else
  //	if ( m_Agents->robots_no > 0) // Robots
  //	{
  //		q = p3d_get_robot_config(m_Agents->robots[0]->robotPt);
  //		ik_succeed =
  // hri_agent_single_task_manip_move(m_Agents->robots[0],
  // task, Tcoord, &q);
  //		//p3d_set_and_update_this_robot_conf(m_Agents->robots[0]->robotPt,q);
  //	}
  else {
    cout << "Warning: No Agent for GIK" << endl;
  }

  if (!withSideEffect) {
    m_Robot->setAndUpdate(*configStored);
  }

  return ik_succeed;
  //#else
  //    cout << "HRI_GIK : " << "not compiled" << endl;
  //    return false;
  //#endif
}

bool Natural::computeIsReachableOnly(const Vector3d& WSPoint,
                                     bool useLeftvsRightArm)
{
  confPtr_t configStored = m_Robot->getCurrentPos();
  bool IKSucceded = false;
  HRI_GIK_TASK_TYPE task;

  if (useLeftvsRightArm == true) {
    // task = GIK_LAREACH; // Left Arm GIK
    task = GIK_LATREACH;
  } else {
    // task = GIK_RAREACH; // Left Arm GIK
    task = GIK_RATREACH;
  }
  p3d_vector3 Tcoord;

  Tcoord[0] = WSPoint[0];
  Tcoord[1] = WSPoint[1];
  Tcoord[2] = WSPoint[2];

  if ((m_IsHuman && (m_Agents->humans_no > 0)) ||
      ((!m_IsHuman) && (m_Agents->robots_no > 0)))  // Humans ot Robots
  {
    configPt q;
    double distance_tolerance = 0.5;
    if (m_IsHuman) {
      q = p3d_get_robot_config(m_Agents->humans[0]->robotPt);
      cout << "agents->humans[0]->robotPt->name = "
           << m_Agents->humans[0]->robotPt->name << endl;
      IKSucceded = hri_agent_single_task_manip_move(
          m_Agents->humans[0], task, &Tcoord, distance_tolerance, &q);
      confPtr_t ptrQ(new Configuration(m_Robot, q));

      if (IKSucceded) {
        if (!ptrQ->isInCollision()) {
          ptrQ->print();
          IKSucceded = false;
          cout << "Config in collision in " << __PRETTY_FUNCTION__ << endl;
        }
      } else {
        //          cout << "IK Failed in " << __PRETTY_FUNCTION__ << endl;
      }
    }
  } else {
    cout << "Warning: No Agent for GIK" << endl;
  }
  m_Robot->setAndUpdate(*configStored);
  return IKSucceded;
}

Transform3d Natural::getGridOriginMatrix()
{
  Transform3d OriginPos(Transform3d::Identity());

  Vector3d trans;
  Matrix3d rot;

  switch (m_KinType) {
    case Achile:
    case Herakles:
      cout << "Set Achile Offset" << endl;

      trans[0] = 0;
      trans[1] = 0;
      trans[2] = 1.10;

      OriginPos.translation() = trans;

      rot = Eigen::AngleAxisd(0, Vector3d::UnitX()) *
            Eigen::AngleAxisd(0, Vector3d::UnitY()) *
            Eigen::AngleAxisd(0, Vector3d::UnitZ());

      OriginPos.linear() = rot;
      break;

    default:
      break;
  }

  return OriginPos;
}

NaturalGrid* Natural::computeNaturalGrid()
{
  vector<double> envSize(6);
  envSize[0] = XYZ_ENV->box.x1;
  envSize[1] = XYZ_ENV->box.x2;
  envSize[2] = XYZ_ENV->box.y1;
  envSize[3] = XYZ_ENV->box.y2;
  envSize[4] = XYZ_ENV->box.z1;
  envSize[5] = XYZ_ENV->box.z2;

  m_Grid = new NaturalGrid(ENV.getDouble(Env::CellSize), envSize, this);

  return m_Grid;
}

void Natural::computeAllCellCost()
{
  m_Grid->resetCellCost();
  // m_Grid->initReachable();
  m_Grid->computeAllCellCost();
}

void computeAllReachableCellCost() {}

/*!
 * Natural cell comparator
 */
class NaturalCellComparator
{
 public:
  bool operator()(NaturalCell* first, NaturalCell* second)
  {
    return (first->getCost() < second->getCost());
  }

} NaturalCellCompObject;

/*!
 * Get the sorted cells
 */
vector<Vector3d> Natural::getSortedReachableWSPoint()
{
  if (!m_BestPointsSorted) {
    m_SortedCells = m_Grid->getAllReachableCells();
    sort(m_SortedCells.begin(), m_SortedCells.end(), NaturalCellCompObject);
    m_BestPointsSorted = true;
  }

  vector<Vector3d> WSPoints(m_SortedCells.size());

  for (unsigned int i = 0; i < WSPoints.size(); i++) {
    WSPoints[i] = m_SortedCells[i]->getWorkspacePoint();
  }

  return WSPoints;
}

vector<pair<double, Vector3d> > Natural::getReachableWSPoint()
{
  vector<NaturalCell*> cells = m_Grid->getAllReachableCells();
  vector<pair<double, Vector3d> > WSPoints(cells.size());

  for (unsigned int i = 0; i < WSPoints.size(); i++) {
    WSPoints[i].first = cells[i]->getCost();
    WSPoints[i].second = cells[i]->getWorkspacePoint();
  }

  return WSPoints;
}

void Natural::setRobotColorFromConfiguration(bool toSet)
{
  if (!m_Robot->getUseLibmove3dStruct()) return;

  if (toSet) {
    double cost = this->cost(*m_Robot->getCurrentPos());

    double colorvector[4];

    colorvector[0] = 0.0;   // red
    colorvector[1] = 0.0;   // green
    colorvector[2] = 0.0;   // blue
    colorvector[3] = 0.01;  // transparency

    GroundColorMixGreenToRed(colorvector, cost);

    g3d_set_custom_color_draw(m_Robot->getP3dRobotStruct(), toSet);
    g3d_set_custom_color_vect(colorvector);
  } else {
    g3d_set_custom_color_draw(m_Robot->getP3dRobotStruct(), toSet);
  }
}

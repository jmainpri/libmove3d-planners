/*
 *  HRICS_Natural.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "HRICS_Natural.hpp"
#include "Grids/gridsAPI.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

Natural::Natural() : 
m_leftArmCost(false),
m_BestPointsSorted(false),
m_Grid(NULL),
m_G(1000000.0)
{
	m_Robot = global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");
	
	cout << "Robot in natural Cost Space is : " << m_Robot->getName() << endl;
	
	initGeneral();
}

Natural::Natural(Robot* R) :
m_debug(false),
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
	map<string,Kinematic> kinos;
	
	kinos["Error"] = Default;
	kinos["ROBOT_JUSTIN"] = Justin;
	kinos["HUMAN_ACHILE"] = Achile;
	
	m_KinType = kinos[m_Robot->getName()];
	
#ifdef HRI_PLANNER
	m_Agents = hri_create_agents();
#endif
	
	switch (m_KinType) 
	{
		case Justin:
			cout << "KinType of HRICS::Natural is Justin ( " << m_Robot->getName() << " ) "<< endl;
			initNaturalJustin();
#ifdef LIGHT_PLANNER
			m_IndexObjectDof = m_Robot->getObjectDof();
#endif
			m_computeNbOfIK = false;
			m_IsHuman = false;
			break;
			
		case Achile:
			cout << "KinType of HRICS::Natural is Achile ( " << m_Robot->getName() << " ) "<< endl;
			initNaturalAchile();
			m_IndexObjectDof = NULL;
			m_computeNbOfIK = false;
			m_IsHuman = true;
			break;
			
		default:
			cout << "No proper robot has been selected in Natural cost function" << endl;
			break;
	}
	
	m_Grid = NULL;
	cout << "Object Dof is " << m_IndexObjectDof << endl;	
}

/***********************************************/
const int JUSTIN_JOINT_SPINE = 1;					//  1, 2,  3,  4
const int JUSTIN_JOINT_HEAD = 6;					//  6,  7,  8

const int JUSTIN_JOINT_ARM_RIGTH_SHOULDER = 9;		//  9,  10, 11
const int JUSTIN_JOINT_ARM_RIGTH_ELBOW = 12;		// 12 , 13
const int JUSTIN_JOINT_ARM_RIGTH_WRIST = 14;		// 14, 15, 16

const int JUSTIN_JOINT_ARM_LEFT_SHOULDER = 17;		// 17, 18, 19
const int JUSTIN_JOINT_ARM_LEFT_ELBOW = 20;			// 20, 21
const int JUSTIN_JOINT_ARM_LEFT_WRIST = 22;			// 22, 23, 24 

/***********************************************/
const int JUSTIN_CONFIG_INDEX_SPINE = 18;					//  2,  3,  4
const int JUSTIN_CONFIG_INDEX_HEAD = 15;					//  5,  6,  7

const int JUSTIN_CONFIG_INDEX_ARM_RIGTH_SHOULDER = 18;		//  8,  9, 10
const int JUSTIN_CONFIG_INDEX_ARM_RIGTH_ELBOW = 21;			// 11
const int JUSTIN_CONFIG_INDEX_ARM_RIGTH_WRIST = 22;			// 12, 13, 14

const int JUSTIN_CONFIG_INDEX_ARM_LEFT_SHOULDER = 25;		// 15, 16, 17
const int JUSTIN_CONFIG_INDEX_ARM_LEFT_ELBOW = 28;			// 18
const int JUSTIN_CONFIG_INDEX_ARM_LEFT_WRIST = 29;			// 19, 20, 21 

void Natural::initNaturalJustin()
{
	/***********************************************/
	m_JOINT_SPINE = JUSTIN_JOINT_SPINE;
	m_JOINT_HEAD = JUSTIN_JOINT_HEAD;
	
	m_JOINT_ARM_RIGTH_SHOULDER = JUSTIN_JOINT_ARM_RIGTH_SHOULDER;
	m_JOINT_ARM_RIGTH_ELBOW = JUSTIN_JOINT_ARM_RIGTH_ELBOW;
	m_JOINT_ARM_RIGTH_WRIST = JUSTIN_JOINT_ARM_RIGTH_WRIST;
	
	m_JOINT_ARM_LEFT_SHOULDER = JUSTIN_JOINT_ARM_LEFT_SHOULDER;
	m_JOINT_ARM_LEFT_ELBOW = JUSTIN_JOINT_ARM_LEFT_ELBOW;
	m_JOINT_ARM_LEFT_WRIST = JUSTIN_JOINT_ARM_LEFT_WRIST; 
	
	/***********************************************/
	m_CONFIG_INDEX_SPINE = JUSTIN_CONFIG_INDEX_SPINE;
	m_CONFIG_INDEX_HEAD = JUSTIN_CONFIG_INDEX_HEAD;
	
	m_CONFIG_INDEX_ARM_RIGTH_SHOULDER = JUSTIN_CONFIG_INDEX_ARM_RIGTH_SHOULDER;
	m_CONFIG_INDEX_ARM_RIGTH_ELBOW = JUSTIN_CONFIG_INDEX_ARM_RIGTH_ELBOW;
	m_CONFIG_INDEX_ARM_RIGTH_WRIST = JUSTIN_CONFIG_INDEX_ARM_RIGTH_WRIST;
	
	m_CONFIG_INDEX_ARM_LEFT_SHOULDER = JUSTIN_CONFIG_INDEX_ARM_LEFT_SHOULDER;
	m_CONFIG_INDEX_ARM_LEFT_ELBOW = JUSTIN_CONFIG_INDEX_ARM_LEFT_ELBOW;
	m_CONFIG_INDEX_ARM_LEFT_WRIST = JUSTIN_CONFIG_INDEX_ARM_LEFT_WRIST; 
	
	configPt q;
	q = p3d_alloc_config(m_Robot->getRobotStruct());
	
	q[0] = 0.000000;
	q[1] = 0.000000;
	q[2] = 0.000000;
	q[3] = 0.000000;
	q[4] = 0.000000;
	q[5] = 0.000000;
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
	
	m_q_Confort = shared_ptr<Configuration>(new Configuration(
					m_Robot,
					p3d_copy_config_deg_to_rad(m_Robot->getRobotStruct(),q)));
	
}
/***********************************************/
const int ACHILE_JOINT_SPINE = 2;					//  2,  3,  4
const int ACHILE_JOINT_HEAD = 5;					//  5,  6,  7

const int ACHILE_JOINT_ARM_RIGTH_SHOULDER = 8;		//  8,  9, 10
const int ACHILE_JOINT_ARM_RIGTH_ELBOW = 11;		// 11
const int ACHILE_JOINT_ARM_RIGTH_WRIST = 12;		// 12, 13, 14

const int ACHILE_JOINT_ARM_LEFT_SHOULDER = 15;		// 15, 16, 17
const int ACHILE_JOINT_ARM_LEFT_ELBOW = 18;			// 18
const int ACHILE_JOINT_ARM_LEFT_WRIST = 19;			// 19, 20, 21 

/***********************************************/
const int ACHILE_CONFIG_INDEX_SPINE = 12;					//  2,  3,  4
const int ACHILE_CONFIG_INDEX_HEAD = 15;					//  5,  6,  7

const int ACHILE_CONFIG_INDEX_ARM_RIGTH_SHOULDER = 18;		//  8,  9, 10
const int ACHILE_CONFIG_INDEX_ARM_RIGTH_ELBOW = 21;		// 11
const int ACHILE_CONFIG_INDEX_ARM_RIGTH_WRIST = 22;		// 12, 13, 14

const int ACHILE_CONFIG_INDEX_ARM_LEFT_SHOULDER = 25;		// 15, 16, 17
const int ACHILE_CONFIG_INDEX_ARM_LEFT_ELBOW = 28;			// 18
const int ACHILE_CONFIG_INDEX_ARM_LEFT_WRIST = 29;			// 19, 20, 21 

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
	
	/***********************************************/
	m_CONFIG_INDEX_SPINE = ACHILE_CONFIG_INDEX_SPINE;
	m_CONFIG_INDEX_HEAD = ACHILE_CONFIG_INDEX_HEAD;
	
	m_CONFIG_INDEX_ARM_RIGTH_SHOULDER = ACHILE_CONFIG_INDEX_ARM_RIGTH_SHOULDER;
	m_CONFIG_INDEX_ARM_RIGTH_ELBOW = ACHILE_CONFIG_INDEX_ARM_RIGTH_ELBOW;
	m_CONFIG_INDEX_ARM_RIGTH_WRIST = ACHILE_CONFIG_INDEX_ARM_RIGTH_WRIST;
	
	m_CONFIG_INDEX_ARM_LEFT_SHOULDER = ACHILE_CONFIG_INDEX_ARM_LEFT_SHOULDER;
	m_CONFIG_INDEX_ARM_LEFT_ELBOW = ACHILE_CONFIG_INDEX_ARM_LEFT_ELBOW;
	m_CONFIG_INDEX_ARM_LEFT_WRIST = ACHILE_CONFIG_INDEX_ARM_LEFT_WRIST; 
	
	configPt q;
	q = p3d_alloc_config(m_Robot->getRobotStruct());
	
	/***********************************************
	 * Neutral Position
	 */
	
	q[0] = 0;
	q[1] = 0;
	q[2] = 0;
	q[3] = 0;
	q[4] = 0;
	q[5] = 0;
	
	q[6] = 0;
	q[7] = 0;
	q[8] = 0;
	q[9] = 0;
	q[10] = 0;
	q[11] = 0;
	
	q[12] = 0;
	q[13] = 0;
	q[14] = 0;
	q[15] = 0;
	q[16] = 0;
	q[17] = 0;
	q[18] = 1.33012;
	q[19] = 0.365646;
	q[20] = -0.12706;
	q[21] = 0.525519;
	q[22] = 0.17558;
	q[23] = -0.342085;
	q[24] = 0.0233874;
	q[25] = -1.22784;
	q[26] = 0.482584;
	q[27] = 0.00436332;
	q[28] = -0.368439;
	q[29] = -0.210487;
	q[30] = 0;
	q[31] = -0.0935496;
	q[32] = 0;
	q[33] = 0;
	q[34] = 0;
	q[35] = 0;
	q[36] = 0;
	q[37] = 0;
	q[38] = 0;
	q[39] = 0;
	q[40] = 0;
	q[41] = 0;
	q[42] = 0;
	q[43] = 0;
	q[44] = 0;
	q[45] = 0;
	
	m_q_Confort = shared_ptr<Configuration>(
											new Configuration(m_Robot,p3d_copy_config(m_Robot->getRobotStruct(),q)));
	
	
	/***********************************************
	 * Wieghts for the Configuration Distance
	 */
	
	q[0] = 0;		// Null Joint
	q[1] = 0;
	q[2] = 0;
	q[3] = 0;
	q[4] = 0;
	q[5] = 0;
	
	q[6] = 0;		// Free Flyer
	q[7] = 0;
	q[8] = 0;
	q[9] = 0;
	q[10] = 0;
	q[11] = 0;
	
	q[12] = 100;	// Torso
	q[13] = 100;
	q[14] = 100;
	
	q[15] = 0;		// Head
	q[16] = 0;
	q[17] = 0;
	
	q[18] = 5;		// Right Shoulder
	q[19] = 1;
	q[20] = 1;
	
	q[21] = 1;		// Right Elbow
	
	q[22] = 1;		// Right Wrist
	q[23] = 1;
	q[24] = 1;
	
	q[25] = 5;		// Left Shoulder 
	q[26] = 1;
	q[27] = 1;
	
	q[28] = 1;		// Left Elbow
	
	q[29] = 1;		// Right Wrist
	q[30] = 1;
	q[31] = 1;
	
	q[32] = 0;		// Legs
	q[33] = 0;
	q[34] = 0;
	q[35] = 0;
	q[36] = 0;
	q[37] = 0;
	q[38] = 0;
	q[39] = 0;
	q[40] = 0;
	q[41] = 0;
	q[42] = 0;
	q[43] = 0;
	q[44] = 0;
	q[45] = 0;
	
	m_q_ConfortWeigths = shared_ptr<Configuration>(
												   new Configuration(m_Robot,p3d_copy_config(m_Robot->getRobotStruct(),q)));
	
	m_mg.push_back( 30 );
	m_mg.push_back(  4 );
	m_mg.push_back(  1 );
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
void Natural::printBodyPos()
{
	if ( m_KinType == Achile )
	{
		//		cout << "ACHILE_JOINT_SPINE = " << m_Robot->getJoint( ACHILE_JOINT_SPINE )->index_dof << endl ;
		//		cout << "ACHILE_JOINT_HEAD = " << m_Robot->getJoint( ACHILE_JOINT_HEAD )->index_dof << endl ;
		//		
		//		cout << "ACHILE_JOINT_ARM_RIGTH_SHOULDER =  " << m_Robot->getJoint( ACHILE_JOINT_ARM_RIGTH_SHOULDER )->index_dof << endl ;
		//		cout << "ACHILE_JOINT_ARM_RIGTH_ELBOW =  " << m_Robot->getJoint( ACHILE_JOINT_ARM_RIGTH_ELBOW )->index_dof << endl ;
		//		cout << "ACHILE_JOINT_ARM_RIGTH_WRIST =  " << m_Robot->getJoint( ACHILE_JOINT_ARM_RIGTH_WRIST )->index_dof << endl ;
		//		
		//		cout << "ACHILE_JOINT_ARM_LEFT_SHOULDER = " << m_Robot->getJoint( ACHILE_JOINT_ARM_LEFT_SHOULDER )->index_dof << endl ;
		//		cout << "ACHILE_JOINT_ARM_LEFT_ELBOW =  " << m_Robot->getJoint( ACHILE_JOINT_ARM_LEFT_ELBOW )->index_dof << endl ;
		//		cout << "ACHILE_JOINT_ARM_LEFT_WRIST =  " << m_Robot->getJoint( ACHILE_JOINT_ARM_LEFT_WRIST )->index_dof << endl ;
		
		if( PointsToDraw != NULL )
		{
			PointsToDraw->clear();
			
			if ( PointsToDraw->size() == 0 ) 
			{
				//Vector3d point(Vector3d::Zero());
				
				PointsToDraw->push_back( m_Robot->getJoint( ACHILE_JOINT_SPINE )->getVectorPos() );
				PointsToDraw->push_back( m_Robot->getJoint( ACHILE_JOINT_HEAD )->getVectorPos() );
				
				PointsToDraw->push_back( m_Robot->getJoint( ACHILE_JOINT_ARM_RIGTH_SHOULDER )->getVectorPos() );
				PointsToDraw->push_back( m_Robot->getJoint( ACHILE_JOINT_ARM_RIGTH_ELBOW )->getVectorPos() );
				PointsToDraw->push_back( m_Robot->getJoint( ACHILE_JOINT_ARM_RIGTH_WRIST )->getVectorPos() );
				
				PointsToDraw->push_back( m_Robot->getJoint( ACHILE_JOINT_ARM_LEFT_SHOULDER )->getVectorPos() );
				PointsToDraw->push_back( m_Robot->getJoint( ACHILE_JOINT_ARM_LEFT_ELBOW )->getVectorPos() );
				PointsToDraw->push_back( m_Robot->getJoint( ACHILE_JOINT_ARM_LEFT_WRIST )->getVectorPos() );
			}
		}
	}
}

/*!
 * Compute the Natural cost for a configuration
 * with weight
 */
double Natural::getConfigCost()
{
	double c_natural = 0.0;
	
	//shared_ptr<Configuration> q_Actual = m_Robot->getCurrentPos();
	
	cout << "-------------------------------" << endl;
	//---------------------------------------------------
	// Joints Displacement
	double c_f_Joint_displacement = getJointDisplacement();
	cout << "JDis = " << c_f_Joint_displacement << endl;
	//---------------------------------------------------
	// Energy
	double c_f_Energy = getEnergy(); 
	cout << "Ener = " << c_f_Energy << endl;
	//---------------------------------------------------
	// Discomfort
	double c_f_Discomfort = getDiscomfort();
	cout << "Disc = " << c_f_Discomfort << endl;
	//---------------------------------------------------
	
	
	/**
	 * Wieghted sum
	 */
	double W_jd = ENV.getDouble(Env::coeffJoint);
	double W_en = ENV.getDouble(Env::coeffEnerg);
	double W_di = ENV.getDouble(Env::coeffConfo);
	
	c_natural = W_jd*c_f_Joint_displacement + W_en*c_f_Energy + W_di*c_f_Discomfort;
	
	//m_leftArmCost = true;
	//c_natural = basicNaturalArmCost(m_leftArmCost);
	cout << "Cost = " << c_natural << endl;
	return c_natural;
}

/*!
 * Joint-displacement : This function evaluates the joint displacement
 * from the VRS project at Iwoa
 */
double Natural::getJointDisplacement()
{
	return 0.1*getCustomDistConfig(*m_Robot->getCurrentPos());
}

/*!
 * Energy : This function evaluates the joint displacement
 * from the VRS project at Iwoa
 */
double Natural::getEnergy()
{
	double Energy = 0.0;
	
	vector<double> DeltaHeigth = getUpperBodyHeigth();
	
	for (unsigned int i=1; i<DeltaHeigth.size(); i++) 
	{
		if( m_mg[i] > 0)
		{
			Energy += pow(m_mg[i],2)*pow(DeltaHeigth[i],2);
		}
	}
	
	return 0.005*Energy;
}

/*!
 * Discomfort : This function evaluates the joint displacement
 * from the VRS project at Iwoa
 */
double Natural::getDiscomfort()
{
	return 0.005*getJointLimits(*m_Robot->getCurrentPos());
}

/*!
 * Discomfort : This function evaluates the different in heigth 
 * of each body regarding the confort position from the VRS project at Iwoa
 */
vector<double> Natural::getUpperBodyHeigth()
{
	int ShoulJoint,	ElbowJoint, WristJoint;
	
	if( m_leftArmCost )
	{
		ShoulJoint = m_JOINT_ARM_LEFT_SHOULDER;
		ElbowJoint = m_JOINT_ARM_LEFT_ELBOW;
		WristJoint = m_JOINT_ARM_LEFT_WRIST;
	}
	else 
	{
		ShoulJoint = m_JOINT_ARM_RIGTH_SHOULDER;
		ElbowJoint = m_JOINT_ARM_RIGTH_ELBOW;
		WristJoint = m_JOINT_ARM_RIGTH_WRIST;
	}
	
	vector<double> heigths;
	
	heigths.push_back( m_Robot->getJoint(ShoulJoint)->getVectorPos()(2) );
	heigths.push_back( m_Robot->getJoint(ElbowJoint)->getVectorPos()(2) );
	heigths.push_back( m_Robot->getJoint(WristJoint)->getVectorPos()(2) );
	
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
double Natural::getCustomDistConfig(Configuration& q) 
{
	double l = 0., ljnt = 0.;
	int njnt = m_Robot->getRobotStruct()->njoints;
	
	for (int i=0; i<=njnt; i++) 
	{
		p3d_jnt* jntPt = m_Robot->getRobotStruct()->joints[i];
		
		for (int j=0; j<jntPt->dof_equiv_nbr; j++) 
		{
			double W = (*m_q_ConfortWeigths)[jntPt->index_dof+j];
			
			double dof_dist = p3d_jnt_calc_dof_dist(jntPt, j, 
													m_q_Confort->getConfigStruct(), 
													q.getConfigStruct());
			//printf("dof_dist[%d] = %f\n",jntPt->index_dof + j,dof_dist);
			ljnt += W*SQR(dof_dist);
		}
	}
	l = sqrt(ljnt);
	
	return l;
}

double Natural::getJointLimits(Configuration& q)
{
	double QU,QL;
	
	QU = QL = 0.0;
	
	double Cost=0.0;
	
	for (unsigned int i=0; i<m_Robot->getNumberOfJoints(); i++) 
	{
		Joint* jntPt = m_Robot->getJoint(i);
		
		for (unsigned int j=0; j<jntPt->getNumberOfDof(); j++) 
		{
			unsigned int k = jntPt->getIndexOfFirstDof()+j;
			
			double q_min,q_max;
			
			jntPt->getDofBounds(j,q_min,q_max);
			
			if (q_max == q_min) {
				continue;
			}
			
			double deltaU = 5.0*(q_max-q[k])/(q_max-q_min);
			double deltaL = 5.0*(q[k]-q_min)/(q_max-q_min);
			
//			cout << "deltaU = " << deltaU << endl;
//			cout << "deltaL = " << deltaL << endl;
			
			QU = pow(0.5*(sin(deltaU+1.571)+1),100);
			QL = pow(0.5*(sin(deltaL+1.571)+1),100);
			
//			cout << "QU = " << QU << endl;
//			cout << "QL = " << QL << endl;
			
			Cost += (QU + QL);
		}
	}
	
//	cout << "Joint Limits Cost : " << Cost << endl;
	return (/*m_G**/Cost);
}

/*!
 *
 */
double Natural::basicNaturalArmCost(bool useLeftvsRightArm)
{
	int ShoulderIndex,	ElbowIndex;
	int ShoulderJoint,	ElbowJoint, WristJoint;
	
	if( useLeftvsRightArm )
	{
		ShoulderJoint = m_JOINT_ARM_LEFT_SHOULDER;
		ElbowJoint = m_JOINT_ARM_LEFT_ELBOW;
		WristJoint = m_JOINT_ARM_LEFT_WRIST;
	}
	else 
	{
		ShoulderJoint = m_JOINT_ARM_RIGTH_SHOULDER;
		ElbowJoint = m_JOINT_ARM_RIGTH_ELBOW;
		WristJoint = m_JOINT_ARM_RIGTH_WRIST;
	}
	
	ShoulderIndex = m_Robot->getJoint(ShoulderJoint)->getJointStruct()->index_dof;
	ElbowIndex = m_Robot->getJoint(ElbowJoint)->getJointStruct()->index_dof;
	
//	cout << "Get Shoulder index dof = " << m_Robot->getJoint(ShoulderJoint)->getJointStruct()->index_dof << endl;
//	cout << "Get Elbow index dof = " << m_Robot->getJoint(ElbowJoint)->getJointStruct()->index_dof << endl;
//	cout << "Get Wrist index dof = " << m_Robot->getJoint(WristJoint)->getJointStruct()->index_dof << endl;

	double deltaJoint=0, potential = 0;
	
	double restq1 = (*m_q_Confort)[ShoulderIndex+0];
	double restq2 = (*m_q_Confort)[ShoulderIndex+1];
	double restq3 = (*m_q_Confort)[ShoulderIndex+2];
	double restq4 = (*m_q_Confort)[ElbowIndex+0];
	
//	cout << "-----------------------------------------------" << endl;
//	cout <<  "restq1 = " << (*m_q_Confort)[ShoulderIndex+0] << endl;
//	cout <<  "restq2 = " << (*m_q_Confort)[ShoulderIndex+1] << endl;
//	cout <<  "restq3 = " << (*m_q_Confort)[ShoulderIndex+2] << endl;
//	cout <<  "restq4 = " << (*m_q_Confort)[ElbowIndex] << endl;
	
//	q[18] = 1.33012;	min = -1.74;	max = 1.57; (max) dist = 10.956
//	q[19] = 0.365646;	min -0.52;		max = 0.52; (min) dist = 15.05
//	q[20] = -0.12706;	min = -1.57;	max = 2.09; (max) dist = 4.88
//	q[21] = 0.525519;	min = 0;		max = 2.44; (max) dist = 3.68
	
	shared_ptr<Configuration> q = m_Robot->getCurrentPos();
	
	deltaJoint = (SQR((*q)[ShoulderIndex+0]-restq1) + 
                SQR((*q)[ShoulderIndex+1]-restq2) +
                SQR((*q)[ShoulderIndex+2]-restq3) + 
                SQR((*q)[ElbowIndex]-restq4))*6/34.57; // Weird 6
	
	potential = (m_Robot->getJoint(ElbowJoint)->getVectorPos()(2) +
               m_Robot->getJoint(WristJoint)->getVectorPos()(2)-2.43)/0.67;
	
	double Cost = (deltaJoint+potential)/2;
	//double Cost = deltaJoint;
	
//	if (Cost < 0) {
//		Cost = 1.0;
//	}
	//cout << "Cost = " << Cost << endl;
	return Cost;
}

/*
double Natural::akinRightArmReachCost()
{
	double cost=0, potential = 0;
	
	double restq1 = (*m_q_Confort)[ACHILE_CONFIG_INDEX_ARM_RIGTH_SHOULDER+0];
	double restq2 = (*m_q_Confort)[ACHILE_CONFIG_INDEX_ARM_RIGTH_SHOULDER+1];
	double restq3 = (*m_q_Confort)[ACHILE_CONFIG_INDEX_ARM_RIGTH_SHOULDER+2];
	double restq4 = (*m_q_Confort)[ACHILE_CONFIG_INDEX_ARM_RIGTH_ELBOW+0];
	
	shared_ptr<Configuration> q = m_Robot->getCurrentPos();
			
	cost = (SQR((*q)[ACHILE_CONFIG_INDEX_ARM_RIGTH_SHOULDER+0]-restq1) + 
			SQR((*q)[ACHILE_CONFIG_INDEX_ARM_RIGTH_SHOULDER+1]-restq2) +
			SQR((*q)[ACHILE_CONFIG_INDEX_ARM_RIGTH_SHOULDER+2]-restq3) + 
			SQR((*q)[ACHILE_CONFIG_INDEX_ARM_RIGTH_ELBOW]-restq4)-0.76)/23.4;
	
	potential = (m_Robot->getJoint(ACHILE_JOINT_ARM_RIGTH_ELBOW)->getVectorPos()(2) +
				 m_Robot->getJoint(ACHILE_JOINT_ARM_RIGTH_WRIST)->getVectorPos()(2)-2.43)/0.67;
		
	return cost+potential/2;
}

double Natural::akinLeftArmReachCost()
{
	double cost=0, potential = 0;
	
	double restq1 = (*m_q_Confort)[ACHILE_CONFIG_INDEX_ARM_LEFT_SHOULDER+0];
	double restq2 = (*m_q_Confort)[ACHILE_CONFIG_INDEX_ARM_LEFT_SHOULDER+1];
	double restq3 = (*m_q_Confort)[ACHILE_CONFIG_INDEX_ARM_LEFT_SHOULDER+2];
	double restq4 = (*m_q_Confort)[ACHILE_CONFIG_INDEX_ARM_LEFT_ELBOW+0];
	
	shared_ptr<Configuration> q = m_Robot->getCurrentPos();
	
	cost = (SQR((*q)[ACHILE_CONFIG_INDEX_ARM_LEFT_SHOULDER+0]-restq1) + 
			SQR((*q)[ACHILE_CONFIG_INDEX_ARM_LEFT_SHOULDER+1]-restq2) +
			SQR((*q)[ACHILE_CONFIG_INDEX_ARM_LEFT_SHOULDER+2]-restq3) + 
			SQR((*q)[ACHILE_CONFIG_INDEX_ARM_LEFT_ELBOW]-restq4)-0.76)/23.4;
	
	potential = (m_Robot->getJoint(ACHILE_JOINT_ARM_LEFT_ELBOW)->getVectorPos()(2) +
				 m_Robot->getJoint(ACHILE_JOINT_ARM_LEFT_WRIST)->getVectorPos()(2)-2.43)/0.67;
	
	return cost+potential/2;
}
*/

/*!
 * Computes the cost for a 
 * Workspace point
 * @param useLeftvsRightArm is true for left and false for right
 */
double Natural::getCost(const Vector3d& WSPoint, bool useLeftvsRightArm , bool withEffect)
{
	switch (m_KinType) 
	{
			
		case Justin:
			if(!m_computeNbOfIK)
			{
				m_leftArmCost = useLeftvsRightArm;
				return getConfigCost();
			}
			else 
			{
				cout << "Natural::Warning::Computing the Number of ik cost" << endl;
				return getNumberOfIKCost(WSPoint);
			}
			break;
			
			
		case Achile:
#ifdef HRI_PLANNER
			if( computeIsReachable(WSPoint,useLeftvsRightArm) )
			{
				m_leftArmCost = useLeftvsRightArm;
				return getConfigCost();
			}
			else {
				cout << "Warning : compute cost of unreachable point" << endl;
				return numeric_limits<double>::max();
			}
#endif
			break;
			
			
		default:
			cout << "Warning : Not implemented" << endl;
			break;
	}
	return NULL;
}

/*!
 * Cost of a workspace point
 */
double Natural::getCostInGrid(const Eigen::Vector3d& WSPoint)
{
	if ( m_Grid->isReachable(WSPoint) ) 
	{
		return m_Grid->getCellCostAt(WSPoint);
	}
	
	return 1.5;
}

/*!
 * Computes the number of IK
 */
double Natural::getNumberOfIKCost(const Vector3d& WSPoint)
{
	shared_ptr<Configuration> q;
	
	double Cost = 0.0;
	const unsigned int NbDirections = 360;
	
	for (unsigned int i=0; i<NbDirections; i++) 
	{
		q = m_Robot->shoot();
		
		(*q)[m_IndexObjectDof+0] = WSPoint[0];
		(*q)[m_IndexObjectDof+1] = WSPoint[1];
		(*q)[m_IndexObjectDof+2] = WSPoint[2];
		
		//			q->getConfigStruct()[32] = 0.000000;
		//			q->getConfigStruct()[33] = 0.000000;
		//			q->getConfigStruct()[34] = -0.785398;
		
		if( q->setConstraintsWithSideEffect() && !q->isInCollision() )
		{
			//m_Cost += grid->getNaturalCostSpace()->getCost();
			//cout << "Center :" << endl << center << endl;
			//cout << rob->getName() << endl;
			//m_QStored = q;
			//m_CostIsComputed = true;
			Cost += 1.0;
			//m_QStored->print(true);
			//return 1.0;
		}
	}
	
	return Cost;
}

/*! 
 * Compute Wether a point is reachable
 */
bool Natural::computeIsReachable(const Vector3d& WSPoint,bool useLeftvsRightArm)
{
#ifdef HRI_PLANNER
	shared_ptr<Configuration> configStored = m_Robot->getCurrentPos();
	
	bool IKSucceded;
	const bool withSideEffect = true;
	
	// 2 - Select Task
	HRI_GIK_TASK_TYPE task;
	
	if (useLeftvsRightArm == true) 
	{
		//task = GIK_LAREACH; // Left Arm GIK
		task = GIK_LATREACH;
	}
	else 
	{
		//task = GIK_RAREACH; // Left Arm GIK
		task = GIK_RATREACH;
	}
	
	configPt q;
	
	p3d_vector3 Tcoord;
	
	Tcoord[0] = WSPoint[0];
	Tcoord[1] = WSPoint[1];
	Tcoord[2] = WSPoint[2];
	
	if(	(m_IsHuman && (m_Agents->humans_no > 0)) || ( (!m_IsHuman) && (m_Agents->robots_no > 0))) // Humans ot Robots
	{
		q = p3d_get_robot_config(m_Robot->getRobotStruct());
		
		HRI_AGENT* agent;
		
		if (m_IsHuman) 
		{
			agent = m_Agents->humans[0];
		}
		else 
		{
			agent = m_Agents->robots[0];
		}

		double approach_distance = 1.0;
		IKSucceded =  hri_agent_single_task_manip_move(agent, task, &Tcoord, approach_distance, &q);
		
		shared_ptr<Configuration> ptrQ(new Configuration(m_Robot,q));
		
		if ( IKSucceded ) 
		{
			//shared_ptr<Configuration> ptrQ(new Configuration(m_Robot,q));
			
			if( /*ptrQ->setConstraintsWithSideEffect() &&*/ !ptrQ->isInCollision() )
			{
				m_Robot->setAndUpdateHumanArms(*ptrQ);
			}
			else 
			{
				IKSucceded = false;
				//Cost = 0.0; 
				cout << "Config in collision in " << __func__ << endl;
			}
		}
		else 
		{
			//Cost = 36;
			cout << "IK Failed in " << __func__ << endl;
		}
		
		
	}
	//	else 
	//	if ( m_Agents->robots_no > 0) // Robots
	//	{
	//		q = p3d_get_robot_config(m_Agents->robots[0]->robotPt);
	//		IKSucceded = hri_agent_single_task_manip_move(m_Agents->robots[0], task, Tcoord, &q);
	//		//p3d_set_and_update_this_robot_conf(m_Agents->robots[0]->robotPt,q);
	//	}
	else 
	{
		cout << "Warning: No Agent for GIK" << endl;
	}
	
	if(!withSideEffect)
	{
		m_Robot->setAndUpdate(*configStored);
	}
	
	return IKSucceded;
#else
	cout << "HRI_GIK : " << "not compiled" << endl;
	return false;
#endif
}


Transform3d Natural::getGridOriginMatrix()
{
	Transform3d OriginPos(Transform3d::Identity());
	
	Vector3d trans;
	Matrix3d rot;
	
	switch (m_KinType) 
	{
		case Achile:
			
			cout << "Set Achile Offset" << endl;
			
			trans[0] = 0;
			trans[1] = 0;
			trans[2] = 1.10;
			
			OriginPos.translation() = trans;
			
			rot =	Eigen::AngleAxisd(0, Vector3d::UnitX())
          *	Eigen::AngleAxisd(0, Vector3d::UnitY())
          *	Eigen::AngleAxisd(0, Vector3d::UnitZ());
			
			OriginPos.linear() = rot;
			break;
			
		default:
			break;
	}
	
	return OriginPos;
}

NaturalGrid* Natural::computeNaturalGrid()
{
	vector<double>  envSize(6);
    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
    envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;
	
	m_Grid = new NaturalGrid(ENV.getDouble(Env::CellSize),envSize,this);
	
	return m_Grid;
}

void Natural::computeAllCellCost()
{
	m_Grid->resetCellCost(); 
	//m_Grid->initReachable();
	m_Grid->computeAllCellCost();
}

void computeAllReachableCellCost()
{
	
}

/*!
 * Natural cell comparator
 */
class NaturalCellComparator
{	
public:
	
    bool operator()(NaturalCell* first, NaturalCell* second)
    {
        return ( first->getCost() < second->getCost() );
    }
	
} NaturalCellCompObject;

/*!
 * Get the sorted cells
 */
vector< Vector3d > Natural::getSortedReachableWSPoint()
{
	if (!m_BestPointsSorted) 
	{
		m_SortedCells = m_Grid->getAllReachableCells();
		sort(m_SortedCells.begin(), m_SortedCells.end(), NaturalCellCompObject);
		m_BestPointsSorted = true;
	}
	
	vector< Vector3d > WSPoints(m_SortedCells.size());
	
	for (unsigned int i=0; i<WSPoints.size(); i++) 
	{
		WSPoints[i] = m_SortedCells[i]->getWorkspacePoint();
	}

	return WSPoints;
}

vector< pair<double,Vector3d> > Natural::getReachableWSPoint()
{
	vector< NaturalCell*> cells = m_Grid->getAllReachableCells();
	vector< pair<double,Vector3d> > WSPoints(cells.size());
	
	for (unsigned int i=0; i<WSPoints.size(); i++) 
	{
		WSPoints[i].first	= cells[i]->getCost();
		WSPoints[i].second  = cells[i]->getWorkspacePoint();
	}
	
	return WSPoints;
}

/*!
 *
 */
//void Natural::compute



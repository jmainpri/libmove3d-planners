//
// C++ Implementation: configuration
//
// Description:
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "API/ConfigSpace/configuration.hpp"
#include "API/Device/robot.hpp"

#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"

#include "planner/cost_space.hpp"
#include "planner/Greedy/CollisionSpace.hpp"

using namespace std;
using namespace tr1;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

//constructor and destructor
Configuration::Configuration(Robot* R) :
  _flagInitQuaternions(false),
  _CollisionTested(false),
  _InCollision(true),
  _CostTested(false),
  _Cost(0.0),
  _Robot(R)
{

    if(_Robot==NULL)
    {
        _Configuration = NULL;
    }
    else
    {
        _Configuration = p3d_alloc_config(_Robot->getRobotStruct());
    }
}

Configuration::Configuration(Robot* R, double* C, bool noCopy) :
  _flagInitQuaternions(false),
  _CollisionTested(false),
  _InCollision(true),
  _CostTested(false),
  _Cost(0.0),
  _Robot(R)
{
    if(_Robot==NULL)
    {
        _Configuration = NULL;
    }
    else
    {
        if(C==NULL)
        {
            _Configuration = C;
        }
        else
        {
					_Configuration = noCopy ? C : p3d_copy_config(_Robot->getRobotStruct(), C);
	  //            this->initQuaternions();
        }
    }
}

Configuration::Configuration(const Configuration& conf) :
  _flagInitQuaternions(conf._flagInitQuaternions),
  _CollisionTested(conf._CollisionTested),
  _InCollision(conf._InCollision),
  _CostTested(conf._CostTested),
  _Cost(conf._Cost),
  _Robot(conf._Robot)
{
    if(_Robot==NULL)
    {
        _Configuration = NULL;
    }
    else
    {
        _Configuration = p3d_copy_config(_Robot->getRobotStruct(), conf._Configuration);
    }
}

Configuration& Configuration::operator= (const Configuration& source)
{
  _flagInitQuaternions = source._flagInitQuaternions;
  _CollisionTested = source._CollisionTested;
  _InCollision = source._InCollision;
  _CostTested = source._CostTested;
  _Cost = source._Cost;
  _Robot = source._Robot;
  
  if(_Robot==NULL)
  {
    _Configuration = NULL;
  }
  else
  {
    p3d_copy_config_into(_Robot->getRobotStruct(), source._Configuration, &_Configuration) ;
  }
  
  return *this;
}

Configuration::~Configuration()
{
    this->Clear();
}

void Configuration::Clear()
{
    if(_Configuration != NULL)
    {
        p3d_destroy_config(_Robot->getRobotStruct(), _Configuration);
    }
}


void Configuration::convertToRadian()
{
    configPt q = p3d_alloc_config(_Robot->getRobotStruct());
    p3d_convert_config_deg_to_rad(_Robot->getRobotStruct(),_Configuration,&q);
    p3d_destroy_config(_Robot->getRobotStruct(),_Configuration);
    _Configuration = q;
}

shared_ptr<Configuration> Configuration::getConfigInDegree()
{
    configPt q = p3d_alloc_config(_Robot->getRobotStruct());
    p3d_convert_config_rad_to_deg(_Robot->getRobotStruct(),_Configuration,&q);
    return (shared_ptr<Configuration> (new Configuration(_Robot,q,true)));
}


//Accessors
Robot* Configuration::getRobot()
{
    return _Robot;
}

configPt Configuration::getConfigStruct()
{
    return _Configuration;
}

void Configuration::setConfiguration(configPt C)
{
    _Configuration = C;
}

void Configuration::setConfiguration(Configuration& C)
{
    _Configuration = C.getConfigStruct();
}

/*bool Configuration::isQuatInit()
{
    return _flagInitQuaternions;
}


Eigen::Quaterniond Configuration::getQuaternion()
{
    return _Quaternions;
}*/


/**
  * InitsQuaternion from eulers Angle
  */
/*void Configuration::initQuaternions()
{
    for(int i = 0; i <= (_Robot->getRobotStruct())->njoints; i++)
    {
        if( _Robot->getRobotStruct()->joints[i]->type == P3D_FREEFLYER)
        {
            _QuatDof = _Robot->getRobotStruct()->joints[i]->index_dof+3;

            Matrix3d m;
            m =     Eigen::AngleAxisd(_Configuration[_QuatDof+0], Vector3d::UnitX())
                    *   Eigen::AngleAxisd(_Configuration[_QuatDof+1], Vector3d::UnitY())
                    *   Eigen::AngleAxisd(_Configuration[_QuatDof+2], Vector3d::UnitZ());

            _Quaternions = Eigen::AngleAxisd(m);
        }
    }
    _flagInitQuaternions = true;
}*/

/**
  * InitsQuaternion from ExternQuaternion and indexDof
  */
/*void Configuration::initQuaternions(int quatDof,Eigen::Quaternion<double> quat)
{
    _QuatDof = quatDof;
    _Quaternions = quat;
    _flagInitQuaternions = true;
}*/


/**
* this conversion uses conventions as described on page:
*   http://www.euclideanspace.com/maths/geometry/rotations/euler/index.htm
*   Coordinate System: right hand
*   Positive angle: right hand
*   Order of euler angles: heading first, then attitude, then bank
*   matrix row column ordering:
*   [m00 m01 m02]
*   [m10 m11 m12]
*   [m20 m21 m22]*/
/*void Configuration::setQuaternionsToEuler()
{
    Matrix3d m = _Quaternions.toRotationMatrix();

    double* heading =   _Configuration+_QuatDof+0;
    double* attitude =  _Configuration+_QuatDof+1;
    double* bank =      _Configuration+_QuatDof+2;

    // Assuming the angles are in radians.
    if (m(1,0) > 0.998) // singularity at north pole
    {
        *heading = atan2(m(0,2),m(2,2));
        *attitude = M_PI_2;
        *bank = 0;
    }
    else if (m(1,0) < -0.998) // singularity at south pole
    {
        *heading = atan2(m(0,2),m(2,2));
        *attitude = -M_PI_2;
        *bank = 0;
    }
    else
    {
        *heading = atan2(-m(2,0),m(0,0));
        *bank = atan2(-m(1,2),m(1,1));
        *attitude = asin(m(1,0));
    }
}*/

/**
  * Computes the distance
  *  between configurations
  */
double Configuration::dist(Configuration& Conf)
{
	return p3d_dist_config(_Robot->getRobotStruct(), 
												 _Configuration, 
												 Conf.getConfigStruct());
	/*
	double ljnt = 0.;
	int njnt = _Robot->getRobotStruct()->njoints;
	p3d_jnt * jntPt;
	int* IsConstraintedDof = _Robot->getRobotStruct()->cntrt_manager->in_cntrt;
	int k = 0;
	
	bool ActivQuaterion = _flagInitQuaternions && Conf._flagInitQuaternions;
	
	for (int i = 0; i <= njnt; i++) 
	{
		jntPt = _Robot->getRobotStruct()->joints[i];
		
		for (int j = 0; j < jntPt->dof_equiv_nbr; j++)
		{
			k = jntPt->index_dof + j;
			if (IsConstraintedDof[k] != DOF_PASSIF)
			{
				if ( ActivQuaterion && ( (k) >= _QuatDof ) && ( (k) < (_QuatDof+3) ))
				{
					
				}
				else
				{
					double dist_n = SQR(p3d_jnt_calc_dof_dist_2(jntPt, j, _Configuration, Conf.getConfigStruct()));
					cout << "Joint "  << k << "  is = " << dist_n << endl;
					ljnt += dist_n;
				}
			}
		}
	}


//    if(ActivQuaterion)
//    {
//        ljnt += SQR(_Quaternions.angularDistance(Conf._Quaternions));
//    }

    double dist = sqrt(ljnt);
//    cout << " Dist is = "  << dist << endl;
    return dist;*/
}

double Configuration::dist(Configuration& q, int distChoice)
{
    switch (distChoice)
    {
    case ACTIVE_CONFIG_DIST:
        return (p3d_ActiveDistConfig(_Robot->getRobotStruct(), _Configuration,
                                     q.getConfigStruct()));
        //  case LIGAND_PROTEIN_DIST:
        //    return(bio_compute_ligand_dist(_Robot->getRobotStruct(), _Configuration, q.getConfigurationStruct()));
        //    break;
    case MOBILE_FRAME_DIST:
        cout
                << "Warning: the MOBILE_FRAME_DIST can't be directly returned from the configurations"
                << endl;
        // hrm_mob_frame_dist(robotPt, mob_frame_ref,ListNode->N->rel_mob_frame);
#ifdef LIGHT_PLANNER
	case ONLY_ROBOT_BASE:
		{
			double ljnt=0.0;
			p3d_jnt* jntPt= _Robot->getBaseJnt();
			for(int j=0; j<jntPt->dof_equiv_nbr; j++) 
			{
				int k = jntPt->index_dof + j;
				
				if ( (p3d_jnt_get_dof_is_user(jntPt, j) && p3d_jnt_get_dof_is_active_for_planner(jntPt,j)) &&
					(_Robot->getRobotStruct()->cntrt_manager->in_cntrt[k] != 2) ) 
				{
					ljnt += SQR(p3d_jnt_calc_dof_dist(jntPt, j, _Configuration, q.getConfigStruct()));
				} 
			}
			return ljnt;
		}
#endif			
    case GENERAL_CSPACE_DIST:
    default:
        return (this->dist(q));
    }
}

bool Configuration::isInCollision()
{
    if(!_CollisionTested)
    {
        this->getRobot()->setAndUpdate(*this);
        _CollisionTested = true;
        //_InCollision = p3d_col_test();      
      if( global_CollisionSpace )
      {
        _InCollision = global_CollisionSpace->isRobotColliding();
        
      }
      else
      {
        _InCollision = p3d_col_test_robot(_Robot->getRobotStruct(), JUST_BOOL);
      }
//        shared_ptr<Configuration> q_cur_robot  = _Robot->getCurrentPos();
//        cout << "6" << " : "<<_Configuration[6] << endl;
//        cout << "7" << " : "<<_Configuration[7] << endl;
//        cout << "---------" << endl;
    }

		return _InCollision;
}

bool Configuration::isOutOfBounds()
{
	return(p3d_isOutOfBounds(_Robot->getRobotStruct(), _Configuration ));
}

void Configuration::setAsNotTested()
{
	_CollisionTested = false;
}

double Configuration::distEnv()
{
    this->getRobot()->setAndUpdate(*this);
    int settings = get_kcd_which_test();
    set_kcd_which_test((p3d_type_col_choice) (40 + 3));
    // 40 = KCD_ROB_ENV
    // 3 = DISTANCE_EXACT
    p3d_col_test_choice();
    // Collision detection with other robots only

    int nof_bodies = _Robot->getRobotStruct()->no;

    double* distances = new double[nof_bodies];

    p3d_vector3 *body = new p3d_vector3[nof_bodies];
    p3d_vector3 *other = new p3d_vector3[nof_bodies];

    p3d_kcd_closest_points_robot_environment(_Robot->getRobotStruct(),
                                             body,other,distances);
    // Get robot closest points to human for each body

    set_kcd_which_test((p3d_type_col_choice) settings);

    int i = (int)(std::min_element(distances,distances+nof_bodies-1 )-distances);

    return distances[i];
}


double Configuration::getActiveDoF(unsigned int ith)
{
	unsigned int nbDoF(0);
	
	for(unsigned int i=0;i<_Robot->getNumberOfJoints();i++)
	{
		p3d_jnt* jntPt = _Robot->getJoint(i)->getJointStruct();
		
		for(int j=0; j<jntPt->dof_equiv_nbr; j++) 
		{
			int k = jntPt->index_dof + j;
			
			if (
					(p3d_jnt_get_dof_is_user(jntPt, j) && p3d_jnt_get_dof_is_active_for_planner(jntPt,j)) &&
					(_Robot->getRobotStruct()->cntrt_manager->in_cntrt[k] != 2) ) 
			{
				if( ith == nbDoF)
				{
					return _Configuration[k];
				}
				nbDoF++;
			}
		}
	}
	
	return nbDoF;
}

void Configuration::setActiveDoF(unsigned int ith, double value)
{
	unsigned int nbDoF(0);
	
	for(unsigned int i=0;i<_Robot->getNumberOfJoints();i++)
	{
		p3d_jnt* jntPt = _Robot->getJoint(i)->getJointStruct();
		
		for(int j=0; j<jntPt->dof_equiv_nbr; j++) 
		{
			int k = jntPt->index_dof + j;
			
			if (
					(p3d_jnt_get_dof_is_user(jntPt, j) && p3d_jnt_get_dof_is_active_for_planner(jntPt,j)) &&
					(_Robot->getRobotStruct()->cntrt_manager->in_cntrt[k] != 2) ) 
			{
				if( ith == nbDoF)
				{
					//cout << "ith = " << k << endl;
					_Configuration[k] = value;
					return;
				}
				nbDoF++;
			}
		}
	}
}

bool Configuration::equal(Configuration& Conf)
{
    if(_Configuration==Conf.getConfigStruct())
    {
        if(_Configuration==NULL)
        {
            return false;
        }
      
      return true;
    }
    else
    {
        if(_Configuration==NULL || Conf.getConfigStruct()==NULL)
        {
            return false;
        }
    }
	
    return (p3d_equal_config(_Robot->getRobotStruct(), _Configuration,
                             Conf.getConfigStruct()));
}

//copie la Configuration courante dans une nouvelle Configuration
shared_ptr<Configuration> Configuration::copy()
{
    return (shared_ptr<Configuration> (new Configuration(_Robot,
                                                         p3d_copy_config(_Robot->getRobotStruct(), _Configuration))));
}

void Configuration::copyPassive(Configuration& C)
{
    for (int i(0); i <= _Robot->getRobotStruct()->njoints; i++)
    {
        p3d_jnt* joint(_Robot->getRobotStruct()->joints[i]);
        for (int j(0); j < joint->dof_equiv_nbr; j++)
        {
            int k = joint->index_dof + j;
            if ((!p3d_jnt_get_dof_is_user(joint, j))
                || (!p3d_jnt_get_dof_is_active_for_planner(joint, j)))
                C[k] = (*this)[k];
        }
    }
}

shared_ptr<Configuration> Configuration::add(Configuration& C)
{
    shared_ptr<Configuration> ptrQ(new Configuration(_Robot));

    p3d_addConfig(_Robot->getRobotStruct(), _Configuration,
                  C.getConfigStruct(), ptrQ->getConfigStruct() );

    return ptrQ;
}

shared_ptr<Configuration> Configuration::sub(Configuration& C)
{
	shared_ptr<Configuration> ptrQ(new Configuration(_Robot));
	
    p3d_subConfig(_Robot->getRobotStruct(), _Configuration,
                  C.getConfigStruct(), ptrQ->getConfigStruct() );
	
    return ptrQ;
}

Configuration& Configuration::mult(double coeff)
{
	Configuration* q = new Configuration(*this);
	
	int njnt = _Robot->getRobotStruct()->njoints;
	int k = 0;
	for (int i = 0; i <= njnt; i++) 
	{
		p3d_jnt *jntPt = _Robot->getRobotStruct()->joints[i];
		for (int j = 0; j < jntPt->dof_equiv_nbr; j++) 
		{
			(*q)[k] *= coeff;
			k ++;
		}
	}
	
	return (*q);
}

Eigen::VectorXd Configuration::getEigenVector()
{
	unsigned int nbDof=0;
	unsigned int njnt = _Robot->getNumberOfJoints();

	// Get  number of Dofs of all joints
	for (unsigned int i = 0; i < njnt; i++) 
		nbDof += _Robot->getJoint(i)->getNumberOfDof();
	
	VectorXd q(nbDof);

	// Get the values of the dofs
	for (unsigned int i = 0; i < nbDof; i++) 
	{
		q(i) = _Configuration[i];
	}
	
	return q;
}

Eigen::VectorXd Configuration::getEigenVector(const int& startIndex, const int& endIndex)
{	
	VectorXd q(endIndex - startIndex);
  
	// Get the values of the dofs
	for (int i = startIndex; i < endIndex; i++) 
	{
		q(i) = _Configuration[i];
	}
	
	return q;
}

void Configuration::setFromEigenVector(const Eigen::VectorXd& conf)
{	
	// Get the values of the dofs
	for (int i = 0; i < conf.size(); i++) 
	{
		_Configuration[i] = conf(i);
	}
}

void Configuration::setFromEigenVector(const Eigen::VectorXd& conf, const int& startIndex, const int& endIndex)
{
  // Get the values of the dofs
	for (int i = startIndex; i < endIndex; i++) 
	{
		_Configuration[i] = conf(i);
	}
}

bool Configuration::setConstraintsWithSideEffect()
{
	Configuration q(_Robot,p3d_get_robot_config(_Robot->getRobotStruct()), true);
	
    bool respect = _Robot->setAndUpdate(*this);
	
    if(respect)
    {
        this->Clear();
        _Configuration = p3d_get_robot_config(_Robot->getRobotStruct());
    }
	
    return respect;
}

bool Configuration::setConstraints()
{
  Configuration q(_Robot,p3d_get_robot_config(_Robot->getRobotStruct()), true);

    bool respect = _Robot->setAndUpdate(*this);

    if(respect)
    {
        this->Clear();
        _Configuration = p3d_get_robot_config(_Robot->getRobotStruct());
    }

    _Robot->setAndUpdate(q);

    return respect;
}

#ifdef LIGHT_PLANNER
Vector3d Configuration::getTaskPos()
{
// TODO: fix the problem of having a function 
// that modifies the configuration in cost computation
//    if(!ENV.getBool(Env::isInverseKinematics))
//    {
//        this->setConstraints();
//    }
    Vector3d taskPos;
    int objDof = _Robot->getObjectDof();
	//cout << "Robot object dof = " << objDof << endl;
    taskPos[0] = _Configuration[objDof+0];
    taskPos[1] = _Configuration[objDof+1];
    taskPos[2] = _Configuration[objDof+2];
    return taskPos;
}
#endif

double Configuration::cost()
{
    if(!_CostTested)
    {
        _Cost = global_costSpace->cost(*this);
        _CostTested = true;
//				cout << "Cost = " << _Cost << endl;
    }
	
	return _Cost;
}

void Configuration::setCostAsNotTested()
{
	_CostTested = false;
}

void Configuration::print(bool withPassive)
{

    cout << "Print Configuration; Robot: " << _Robot->getRobotStruct() << endl;

    //	print_config(_Robot->getRobotStruct(),_Configuration);

//    configPt degConf = getConfigInDegree()->getConfigStruct();
//
//    for (int i = 0; i < _Robot->getRobotStruct()->nb_dof; i++)
//    {
//        //	    cout << "q["<<i<<"]"<<" = "<< _Configuration[i] << endl;
//        cout << "degConf["<< i <<"] = " << degConf[i] << endl;
//    }

    //	int nb_dof;
    //
    //	if(robotPt != NULL){
    //		nb_dof = mR->getP3dRob()->nb_user_dof;
    //	}

    //	for(int i=0; i<nb_dof;i++){
    //		PrintInfo(("q[%d] = %f\n", i, q[i]));
    //	}


    int njnt = _Robot->getRobotStruct()->njoints, k;
    p3d_jnt * jntPt;
    for(int i=0; i<=njnt; i++)
    {
        jntPt = _Robot->getRobotStruct()->joints[i];
        for(int j=0; j<jntPt->dof_equiv_nbr; j++)
        {
            k = jntPt->index_dof + j;
            if (withPassive || ( p3d_jnt_get_dof_is_user(jntPt, j)
                /*&& (p3d_jnt_get_dof_is_active_for_planner(jntPt,j) */
                && (_Robot->getRobotStruct()->cntrt_manager->in_cntrt[k] != DOF_PASSIF )))
                {
                cout << "q["<<k<<"] = "<<_Configuration[k]<<endl;
            }
        }
    }
    cout << "\n--------------------------------" << endl;
}

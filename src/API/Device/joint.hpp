/*
 *  joint.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 12/05/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */
#ifndef JOINT_HPP
#define JOINT_HPP

#include "API/ConfigSpace/configuration.hpp"

#include "p3d_matrix.h"

class Robot;

#ifndef _DEVICE_H
struct jnt;
#endif

/*!
 @ingroup ROBOT
 @brief This class holds a Joint and is associated with a Body (Link)
 It's the basic element of a kinematic chain
 */
class Joint 
{	
public:
	/**
   * Constructor
   * @param The p3d_jnt that is used
   */
  Joint(Robot* R, jnt* jntPt , int id = -1, bool copy = false );
	
  /**
   * Destructor of the class
   */
  ~Joint();
  
  /**
   * Returns the name of the joint
   */
  std::string getName() const { return m_Name; }
	
	/**
	 * Get the Joint structue
	 */
	jnt* getJointStruct() const { return m_Joint; }
	
	/**
	 * Get the Matrix abs_pos of the Joint 
	 */
	Eigen::Transform3d	getMatrixPos() const;
	
	/**
	 * Get the Vector abs_pos of the Joint 
	 */
	Eigen::Vector3d		getVectorPos() const;
	
	/**
	 * Get the p3d abs pos
	 */
	p3d_matrix4*		getAbsPos();
	
	/**
	 * Random shoot the joint
	 */
	void            shoot(Configuration& q,bool sample_passive=false);
	
	/**
	 * Returns the Joint Dof
	 */
	double          getJointDof(int ithDoF) const;
	
	/**
	 * Set the Joint Dof
	 */
	void            setJointDof(int ithDoF, double value);
  
  /**
	 * True if Joint Dof is user
	 */
	bool            isJointDofUser(int ithDoF) const;
	
	/**
	 * Get Min Max dof
	 */
	void            getDofBounds(int ithDoF, double& vmin, double& vmax) const;
	
	/**
	 * Get Number of DoF
	 */
	unsigned int		getNumberOfDof() const;
	
	/**
	 * Get Dof Pos in Configuration
	 */
	unsigned int		getIndexOfFirstDof() const;
  
  /**
   * Get the id in the joint structure of the robot
   */
  int getId() const { return m_id; }
	
	/**
	 * Set the config from the DoF values
	 */
	void            setConfigFromDofValues(Configuration& q);
  
  /**
   * Get Previous joints
   */
  Joint* getPreviousJoint();

	
private:
	Robot*				m_Robot;
	jnt*          m_Joint; /*!< The p3d structure for the Joint*/
  std::string		m_Name; /*!< The Joint's Name */
	bool          m_copy; /*!< Is true if the p3d_jnt copies and not only points to the structure */
  int           m_id;   /*!< id with which it was initilized */
	
};

#endif

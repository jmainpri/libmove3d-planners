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
class Joint 
{	
public:
    /**
   * Constructor
   * @param The p3d_jnt that is used
   */
    Joint(Move3D::Robot* R, void* jntPt , int id = -1, bool copy = false );

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
    jnt* getP3dJointStruct() const { return static_cast<jnt*>( m_Joint ); }

    /**
     * Get the Matrix abs_pos of the Joint
     */
    Eigen::Transform3d getMatrixPos() const;

    /**
     * Get the Vector abs_pos of the Joint
     */
    Eigen::Vector3d	getVectorPos() const;

    /**
     * Random shoot the joint
     */
    void shoot(Configuration& q,bool sample_passive=false);

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


private:
    Robot*			m_Robot;
    void*           m_Joint; /*!< The p3d structure for the Joint*/
    std::string		m_Name; /*!< The Joint's Name */
    bool            m_copy; /*!< Is true if the p3d_jnt copies and not only points to the structure */
    int             m_id;   /*!< id with which it was initilized */

};

}

void move3d_set_fct_joint_constructor( boost::function<void*( Move3D::Joint*, void*, std::string& name )> fct );
void move3d_set_fct_joint_get_vector_pos( boost::function<Eigen::Vector3d( const Move3D::Joint* )> fct ) ;
void move3d_set_fct_joint_get_matrix_pos( boost::function<Eigen::Transform3d( const Move3D::Joint* )> fct );
void move3d_set_fct_joint_joint_shoot( boost::function<void( Move3D::Joint*, Move3D::Configuration&, bool )> fct );
void move3d_set_fct_joint_get_joint_dof( boost::function<double( const Move3D::Joint*, int )> fct );
void move3d_set_fct_joint_set_joint_dof( boost::function<void( const Move3D::Joint*, int, double )> fct );
void move3d_set_fct_joint_is_joint_user( boost::function<bool( const Move3D::Joint*, int )> fct );
void move3d_set_fct_joint_get_bound( boost::function<void( const Move3D::Joint*, int, double&, double& )> fct );
void move3d_set_fct_joint_get_bound_rand( boost::function<void( const Move3D::Joint*, int, double&, double& )> fct );
void move3d_set_fct_joint_get_nb_of_joints( boost::function<int( const Move3D::Joint* )> fct );
void move3d_set_fct_joint_get_index_of_first_dof( boost::function<int( const Move3D::Joint* )> fct );
void move3d_set_fct_joint_get_previous_joint( boost::function<Move3D::Joint*( const Move3D::Joint*, Move3D::Robot* )> fct );
void move3d_set_fct_joint_joint_dist( boost::function<double( const Move3D::Joint* )> fct );

#endif

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

#ifndef HRICS_NATURAL_HPP
#define HRICS_NATURAL_HPP

#include "hri_costspace/grid/HRICS_natural_grid.hpp"
#include "hri_costspace/grid/HRICS_natural_cell.hpp"

//#include "hri_costspace/Grid/HRICS_two_d_grid.hpp"


#ifdef HRI_PLANNER
#include <libmove3d/hri/hri.h>
#endif

/**
 @ingroup HRICS
 */
namespace HRICS
{
/**
  * Natural Motion and Arm Confort
  */
class Natural
{
public:

    Natural( Move3D::Robot* R );
    ~Natural();

    //! Initilize the parameters for the Natural Cost space
    void initGeneral();
    void initNaturalJustin();
    void initNaturalAchile();
    void initNaturalHerakles();
    void initNaturalBiomech();
    void initNaturalOldDude();
    void initHumanBaseGrid(std::vector<double> box);

    //! get the robot
    Move3D::Robot* getRobot() { return m_Robot; }

    //! returns true of the robot is a human
    bool IsHuman() { return m_IsHuman; }

    //! Print position of the joints considered
    void printBodyPos();

    //! Get the elemetary cost features
    void getConfigCostFeatures( Eigen::VectorXd& features );

    //! Get the cost of the current configuration
    double getConfigCost();

    //! Get the cost of a point in the grid
    double getWorkspaceCost(const Eigen::Vector3d& WSPoint);

    //! Get the reachable workspace points
    std::vector< std::pair<double,Eigen::Vector3d> > getReachableWSPoint();

    //! Compute if the Workspace Point is Reachable Move the robot
    bool computeIsReachableAndMove( const Eigen::Vector3d& WSPoint, bool leftArm);

    //! Compute if the Workspace Point is Reachable Move the robot
    bool computeIsReachableOnly(const Eigen::Vector3d& WSPoint, bool leftArm);

    //! Set the robot to the comfort posture
    void setRobotToConfortPosture();

    //! Return the comfort posture
    Move3D::confPtr_t getComfortPosture() { return m_q_Confort->copy(); }

    //! get the cost of a point in the workspace
    double getCost(const Eigen::Vector3d& WSPoint, bool useLeftvsRightArm, bool withEffect = false);

    //! get the grid origin
    Eigen::Transform3d getGridOriginMatrix();

    //! returns true of the point in the workspace is reachable
    bool getWorkspaceIsReachable(const Eigen::Vector3d& WSPoint);

    //! set the robot color from configuration confort
    //! if pass false as argument the robot returns to normal color
    void setRobotColorFromConfiguration(bool toSet=true);

    //! get sorted reachable points
    std::vector<Eigen::Vector3d> getSortedReachableWSPoint();

    // Compute the natural grid
    NaturalGrid* computeNaturalGrid();

    //! Set the reachability grid
    void setGrid(NaturalGrid* grid) { m_Grid = grid; }

    //! get the object index dof
    int getObjectDof() { return m_IndexObjectDof; }

    //! get the grid
    NaturalGrid* getGrid() { return m_Grid; }

private:


    /**
      * Get the 3 component of natural
      * cost space
      */
    double getJointDisplacement();
    double getEnergy();
    double getDiscomfort();

    /**
      * Others
      */
    double basicNaturalArmCost(bool useLeftvsRightArm);

    std::vector<double> getUpperBodyHeigth(bool useReference = true);
    double getCustomDistConfig( Move3D::Configuration& q);
    double getJointLimits( Move3D::Configuration& q);

    /**
      * Simple number of IK Cost
      */
    double getNumberOfIKCost(const Eigen::Vector3d& WSPoint);

    /**
     * compute a NaturalGrid (calling class NaturalGrid)
     * by using Env variable.
     */

    void computeAllCellCost();
    void computeAllReachableCellCost();

    void initConfigIndices();


    bool                m_debug;
    int                 m_IndexObjectDof;
    bool                m_computeNbOfIK;
    bool                m_leftArmCost;
    bool                m_BestPointsSorted;
    Move3D::Robot*      m_Robot;
    NaturalGrid*        m_Grid;

    enum Kinematic
    {
        Default,
        Justin,
        Achile,
        Herakles,
        Biomech,
        OldDude
    };

    bool			m_IsHuman;

#ifdef HRI_PLANNER
    HRI_AGENTS*		m_Agents;
#endif

    Kinematic		m_KinType;

    /***********************************************/
    int m_JOINT_SPINE;
    int m_JOINT_HEAD;

    int m_JOINT_ARM_RIGTH_SHOULDER;
    int m_JOINT_ARM_RIGTH_ELBOW;
    int m_JOINT_ARM_RIGTH_WRIST;

    int m_JOINT_ARM_LEFT_SHOULDER;
    int m_JOINT_ARM_LEFT_ELBOW;
    int m_JOINT_ARM_LEFT_WRIST;

    /***********************************************/
    int m_CONFIG_INDEX_SPINE;
    int m_CONFIG_INDEX_HEAD;

    int m_CONFIG_INDEX_ARM_RIGTH_SHOULDER;
    int m_CONFIG_INDEX_ARM_RIGTH_ELBOW;
    int m_CONFIG_INDEX_ARM_RIGTH_WRIST;

    int m_CONFIG_INDEX_ARM_LEFT_SHOULDER;
    int m_CONFIG_INDEX_ARM_LEFT_ELBOW;
    int m_CONFIG_INDEX_ARM_LEFT_WRIST;


    /**
      * @brief The Confort configuration
      */
    Move3D::confPtr_t m_q_Confort;

    /**
      * @brief Weights associated to confort
      */
    Move3D::confPtr_t m_q_ConfortWeigths;

    /**
      * Weigth associated to the joints limits function
      */
    double								m_G;

    /**
      * Mass times gravity constant = potential energy
      * @bref Mass (Weigts) associated with each arm
      */
    std::vector<double>					m_mg;
    
    /**
     * Height of arms at rest
     */
    std::vector<double>        m_armHeightL;
    std::vector<double>        m_armHeightR;

    Move3D::confPtr_t m_q_Init;
    Move3D::confPtr_t m_q_Goal;

    /**
      * Sorted Cells
      */
    std::vector< NaturalCell* >			m_SortedCells;
};
}

#endif

#ifndef HRICS_PLANNER_H
#define HRICS_PLANNER_H

/*
 *  HRICS_CostSpace.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 04/12/09.
 *  Copyright 2009 mainprice@laas.fr All rights reserved.
 *
 */

#include "planner/planner.hpp"
#include "planner/Diffusion/RRT.hpp"

#include "HRICS_distance.hpp"
#include "HRICS_visibility.hpp"
#include "HRICS_natural.hpp"
#include "grid/HRICS_grid.hpp"
#include "grid/HRICS_grid_state.hpp"
#include "grid/HRICS_agent_grid.hpp"

/**
 @defgroup HRICS Hri Cost space
 */

/**
 @ingroup HRICS
 */
namespace HRICS
{
/*!
     * Base class for the HRICS motion planners
     */
class HumanAwareMotionPlanner : public Move3D::Planner
{

public:
    HumanAwareMotionPlanner() :
        Planner(),
        m_DistanceSpace(NULL),
        m_VisibilitySpace(NULL),
        m_ReachableSpace(NULL),
        m_NaturalSpace(NULL) { }

    HumanAwareMotionPlanner(Move3D::Robot* rob, Move3D::Graph* graph) :
        Planner(rob,graph),
        m_DistanceSpace(NULL),
        m_VisibilitySpace(NULL),
        m_ReachableSpace(NULL),
        m_NaturalSpace(NULL) { }

    ~HumanAwareMotionPlanner()
    {
        delete m_DistanceSpace;
        delete m_VisibilitySpace;
    }

    Distance*	getDistance() { return m_DistanceSpace; }
    Visibility* getVisibility() { return m_VisibilitySpace; }
    Natural*	getNaturality() { return m_NaturalSpace; }
    Natural*	getReachability() { return m_ReachableSpace; }


    void setDistance(Distance* dist)
    {
        if( m_DistanceSpace )
        {
            delete m_DistanceSpace;
        }
        m_DistanceSpace = dist;
    }

    void setVisibility(Visibility* visib)
    {
        if( m_VisibilitySpace )
        {
            delete m_VisibilitySpace;
        }
        m_VisibilitySpace = visib;
    }

    void setNatural(Natural* nat)
    {
        if( m_NaturalSpace )
        {
            delete m_NaturalSpace;
        }
        m_NaturalSpace = nat;
    }

    void setReachability(Natural* nat)
    {
        if( m_ReachableSpace )
        {
            delete m_ReachableSpace;
        }
        m_ReachableSpace = nat;
    }

    unsigned int run() { return 0; }

protected:
    /**
      * Distance and Visibility
      * cost spaces
      */
    Distance*			m_DistanceSpace;
    Visibility*		    m_VisibilitySpace;
    Natural*			m_ReachableSpace;
    Natural*			m_NaturalSpace;

};

/*!
 * Workspace Motion Planners
 */
class Workspace : public HumanAwareMotionPlanner
{

public :

    /**
      * Constructors & Destructors
      */
    Workspace();
    Workspace(Move3D::Robot* rob, Move3D::Graph* graph);
    ~Workspace();

    /**
      * Init Associated Objects
      */
    void initGrid();
    void deleteGrid();
    
    void initAgentGrids(double cellsize);
    void deleteAgentGrids();
    std::vector<AgentGrid*> getAgentGrids() { return m_HumanGrids; }
    
    void initDistance();
    void initVisibility();
    void initReachable();
    void initNatural();
    void initOtpPlanner();
#ifdef HRI_PLANNER
    void setAgents( HRI_AGENTS* agents ) { m_Agents = agents; }
#endif

    /**
      * Get Robot and Human
      */
    Move3D::Robot* getHuman(){ return mHumans[0]; }
    std::vector<Move3D::Robot*> getHumans(){ return mHumans; }
    Move3D::Robot* getRobot(){ return _Robot; }
    Eigen::Vector3d getVisball();
    
    /**
      * Computes A* in Grid
      */
    bool computeAStarIn3DGrid();
    double pathCost();
    void draw3dPath();

    /**
      * Distance to 3D path
      */
    double distanceToEntirePath();
    double distanceToCellPath();

    /**
      * Getters
      */
    Grid* getGrid() { return m3DGrid; }
    std::vector<Eigen::Vector3d> get3DPath() { return m3DPath; }
    std::vector<Move3D::ThreeDCell*> getCellPath() { return m3DCellPath; }
    int getIndexObjectDof()
    {
        if(mIndexObjectDof==0)
        {
            std::cout << "Workspace::Warning::mIndexObjectDof == 0" << std::endl;
        }
        return mIndexObjectDof;
    }

    /**
      * Run RRT
      */
    bool initHriRRT();

    /**
      * Choose best transfer point from
      * the natural costspace associated to the human
      */
    void deactivateOnlyBaseCollision();
    void activateOnlyBaseCollision();
    
    /**
     * Test if Jido (Base?) is in collision in this configuration
     */
    bool sampleRobotBase(Move3D::confPtr_t q_base, const Eigen::Vector3d& WSPoint);

    /**
     * find a position where the robot can probably pick the objects on the list
     */
    bool findGrapingPosition(std::string robot_name, std::vector<std::string> objects);
    
    /**
     * Test if Jido is in collision in this configuration (?)
     */
    bool transPFromBaseConf(Move3D::confPtr_t q_base, std::vector< Eigen::Vector3d > points );
    bool baseInSight(Move3D::confPtr_t q_base);

    /**
     * Set the OTP from given point
     */
    void setCurrentOTP(const Eigen::Vector3d& p) { m_OTP = p; }
    
    /**
     * Return the OTP
     */
    Eigen::Vector3d getCurrentOTP() { return m_OTP; }
    
    /**
     * testColision for the given configuration
     */
    bool testCol(Move3D::confPtr_t q_base);

    /**
     * set Repo Pose of PR2
     */
    void initPR2RepoConf();

    /**
     * set PR2 to Give configuration
     */
    void initPR2GiveConf();

    /**
     * set PR2 and Human to testing position
     */
    void initPR2AndHumanTest();

    /**
     * Compute the position of the Robot Arm considering the current configuration
     */
    void computePR2GIK(bool move);

    /**
     * change Robot position to test PR2
     */
    void ChangeRobotPos(double value);

    /**
     * Draws the current OTP
     */
    void drawCurrentOTP();
    
    /**
     * Test straight lines between target point and the point of the trajectory
     */
    double* testTransferPointToTrajectory( const Eigen::Vector3d& WSPoint, Move3D::Trajectory& traj, unsigned int& id);


    /**
     * Choses a good transfer point
     */
    bool chooseBestTransferPoint(Eigen::Vector3d& transfPoint, bool move, unsigned int threshold);

    /**
     * Compute a transfert point from a loaded grid, without taking into acount a possible colision.
     * If a trandfert point is found, return true and put the vector into transfPoint else return false
     */
    bool computeBestTransferPoint(Eigen::Vector3d& transfPoint);

    /**
     * Compute a transfert point from a loaded grid, taking into acount a possible colision.
     * If a trandfert point is found, return true and put the vector into transfPoint else return false
     */
    bool computeBestFeasableTransferPoint(Eigen::Vector3d& transfPoint);

    /**
     * compute an object transfert point.
     * the first variable make the human go to the posture or just show it
     * the second one allow to choose different computing fonctions taking into account or not the environment
     */
    bool ComputeTheObjectTransfertPoint(bool Move, int type, Eigen::Vector3d& transfPoint, int threshold);

    /**
     * compute an object transfert point.
     * The pose if the left or right hand of Herakles is used depending
     * on the flag given as input
     * @param use the rightHand if true
     * @return a position vector in world coordiate
     */
    Eigen::Vector3d computeOTPFromHandPose( bool rightHand );

    /**
     * @brief setAndUpdateHumanArms
     * @param q
     */
    bool setAndUpdateHumanArms( Move3D::Configuration& q );

private:

    void solveAStar( State* start, State* goal );

    //! Humans in the scene
    std::vector<Move3D::Robot*>     mHumans;

    //! index of the Dof from which the robot holds
    //! the object
    int mIndexObjectDof;
    
    //! OPT (object transfer point)
    Eigen::Vector3d m_OTP;
    
    //! 3d grid to compute a Workspace path
    Grid*                   m3DGrid;
    std::vector<AgentGrid*> m_HumanGrids;

#ifdef HRI_PLANNER
    HRI_AGENTS*		m_Agents;
#endif

    //! 3d path internals
    bool mPathExist;
    std::vector<Eigen::Vector3d>   m3DPath;
    std::vector<Move3D::ThreeDCell*>  m3DCellPath;
};
}

#endif

//
//  HRICS_humanCostSpace.h
//  libmove3d-motion
//
//  Created by Jim Mainprice on 30/11/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#ifndef HUMAN_COSTSPACE_HPP
#define HUMAN_COSTSPACE_HPP

#include "API/Device/robot.hpp"
#include "grid/HRICS_agent_grid.hpp"
#include "grid/HRICS_natural_cell.hpp"

/**
 @ingroup HRICS
 */
namespace HRICS
{

class HumanCostSpace
{

public:

    HumanCostSpace();
    HumanCostSpace( Move3D::Robot* rob, std::vector<Move3D::Robot*> humans, Natural* costspace, double cellSize );

    ~HumanCostSpace();
    
    Move3D::Robot* getRobot() { return m_Robot; }
    
    double getCost( Move3D::Configuration& q);
    double getCompleteCost( Move3D::Configuration& q, std::vector<double>& cost_sum);
    double getPointCost( Move3D::Configuration& q);
    
    bool getHandoverPointList(std::vector<Eigen::Vector3d>& points, bool recompute_cells, int arm_type);
    
    void computeAllCellCost();
    void testCostFunction();
    
    void loadAgentGrids(const std::string& filename);
    void saveAgentGrids();
    
    void drawDistances();
    void drawReachableGrid();
    
    AgentGrid* getAgentGrid( Move3D::Robot* agent );
    
private:

    bool initElementarySpaces();
    void deleteElementarySpaces();
    
    bool initHumanGrids( double cellSize );
    void deleteHumanGrids();
    
    bool initPr2();
    bool initJustin();
    bool initGreyTape();
    
    double groupingCost();
    
    Move3D::Robot*                m_Robot;
    std::vector<Move3D::Joint*>   m_CostJoints;
    std::vector<Move3D::Robot*>   m_Humans;
    std::vector<AgentGrid*>       m_Grids;
    Move3D::Joint*                m_PlatformJoint;
    std::vector< std::pair<double,NaturalCell*> > m_ReachableCells;
    
    enum PlanningType { NAVIGATION = 0, MANIPULATION = 1, MOBILE_MANIP = 2 } m_planning_type;
    
    Distance*			m_DistanceSpace;
    Visibility*         m_VisibilitySpace;
    Natural*			m_ReachableSpace;
};
}

#endif

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
#include "Grid/HRICS_AgentGrid.hpp"
#include "Grid/HRICS_NaturalCell.hpp"

/**
 @ingroup HRICS
 */
namespace HRICS
{
class HumanCostSpace
{
public:
    HumanCostSpace();
    HumanCostSpace( Robot* rob, std::vector<Robot*> humans, Natural* costspace, double cellSize );

    ~HumanCostSpace();
    
    Robot* getRobot() { return m_Robot; }
    
    double getCost(Configuration& q);
    double getCompleteCost(Configuration& q, std::vector<double>& cost_sum);
    double getPointCost(Configuration& q);
    
    bool getHandoverPointList(std::vector<Eigen::Vector3d>& points, bool recompute_cells, int arm_type);
    
    void computeAllCellCost();
    void testCostFunction();
    
    void loadAgentGrids(const std::string& filename);
    void saveAgentGrids();
    
    void drawDistances();
    void drawReachableGrid();
    
    AgentGrid* getAgentGrid(Robot* agent);
    
private:
    bool initElementarySpaces();
    void deleteElementarySpaces();
    
    bool initHumanGrids( double cellSize );
    void deleteHumanGrids();
    
    bool initPr2();
    bool initJustin();
    bool initGreyTape();
    
    double groupingCost();
    
    Robot*                        m_Robot;
    std::vector<Joint*>           m_CostJoints;
    std::vector<Robot*>           m_Humans;
    std::vector<AgentGrid*>       m_Grids;
    Joint*                        m_PlatformJoint;
    std::vector< std::pair<double,NaturalCell*> > m_ReachableCells;
    
    enum PlanningType { NAVIGATION = 0, MANIPULATION = 1, MOBILE_MANIP = 2 } m_planning_type;
    
    Distance*			m_DistanceSpace;
    Visibility*		m_VisibilitySpace;
    Natural*			m_ReachableSpace;
};
}

#endif

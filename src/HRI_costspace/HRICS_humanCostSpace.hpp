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
    
    double getCost(Configuration& q);
		
    void computeAllCellCost();
    void testCostFunction();
    
    void saveAgentGrids();
    void loadAgentGrids();
    
	private:
    bool initElementarySpaces();
    void deleteElementarySpaces();
    
    bool initHumanGrids( double cellSize );
    void deleteHumanGrids();
    
    bool initPr2();
    bool initGreyTape();
    
    double groupingCost();
    
		Robot*                        m_Robot;
    std::vector<Joint*>           m_CostJoints;
    std::vector<Robot*>           m_Humans;
    std::vector<AgentGrid*>       m_Grids;
    
    enum PlanningType { NAVIGATION = 0, MANIPULATION = 1, MOBILE_MANIP = 2 } m_planning_type;
    
    Distance*			m_DistanceSpace;
		Visibility*		m_VisibilitySpace;
		Natural*			m_ReachableSpace;
	};
}

#endif

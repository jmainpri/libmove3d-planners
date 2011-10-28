/*
 *  HRICS_AgentCell.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef HRICS_HUMAN_CENTERED_GRID_H_
#define HRICS_HUMAN_CENTERED_GRID_H_

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Grids/ThreeDGrid.hpp"

#include "Graphic-pkg.h"

namespace HRICS
{
  class AgentGrid;
  
	class AgentCell : public API::ThreeDCell
	{
	public:
		AgentCell();
		AgentCell(int i, Eigen::Vector3i pos , Eigen::Vector3d corner, AgentGrid* grid);
		//AgentCell(const AgentCell& cell);
		
		~AgentCell();
		
		double getCost();
		double getCost(bool leftHand);
		void setCost(double Cost) { m_Cost = Cost; }
		void setBlankCost();
    
    double getDistance() { return m_Distance; }
    double getVisibility() { return m_Visiblity; }
    double getReachability() { return m_Reachability; }

    void computeDistance();
    void computeVisibility();
		void computeReachability();

		void resetReachable();
		bool getIsLeftArmReachable();
		
		Eigen::Vector3d getWorkspacePoint();
		
		void setIsReachable(bool reach) { m_IsReachable = reach; }
		void setIsReachableWithLA(bool reach) { m_IsReachWithLeftArm = reach; }
		void setIsReachableWithRA(bool reach) { m_IsReachWithRightArm = reach; }
		
		bool isReachable() { return m_IsReachable; }
		bool isReachableWithLA() { return m_IsReachWithLeftArm; }
		bool isReachableWithRA() { return m_IsReachWithRightArm; }
    
    void resetExplorationStatus();
		
		void createDisplaylist();
		
		void draw(bool transform);
		
		bool writeToXml(xmlNodePtr cur);
		bool readCellFromXml(xmlNodePtr cur);
		
		int setRobotToStoredConfig();
    
  protected:
		
		Eigen::Vector3i getCoord() { return m_Coord; }
		
		bool getOpen() { return m_Open; }
		void setOpen() { m_Open = true; }
		
		bool getClosed() { return m_Closed; }
		void setClosed() { m_Closed = true; }
		
	private:
		
    Eigen::Vector3d m_Center;
		Eigen::Vector3i m_Coord;
		
		double* m_v0; double* m_v1; double* m_v2; double* m_v3;
		double* m_v4; double* m_v5; double* m_v6; double* m_v7;
		
		bool m_Open;
		bool m_Closed;
		
		bool m_IsCostComputed;
		double m_Cost;
		
		bool m_IsReachable;
		bool m_IsReachWithLeftArm;
		bool m_IsReachWithRightArm;
		
    double m_Distance;
    double m_Visiblity;
    double m_Reachability;
    
		unsigned int m_NbDirections;
		
		std::tr1::shared_ptr<Configuration> m_QStored;
		
		GLint m_list;
	};
	
  
  class Distance;
  class Visibility;
	class Natural;
  
	class AgentGrid : public API::ThreeDGrid
	{
	public:
		AgentGrid();
		AgentGrid( std::vector<int> size );
		AgentGrid(double pace, std::vector<double> envSize,
              Robot* robot, Distance* distCostSpace,Visibility* VisiCostSpace, Natural* NatuCostSpace);
		AgentGrid(const AgentGrid& grid);
		
    ~AgentGrid();
    
		void setGridOrigin();
		
		API::ThreeDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z );
		
    int robotConfigInCell(int i);
    void draw();
    
    Robot*      getRobot();
    
    Distance*     getDistance();
    Visibility*   getVisibility();
    Natural*      getNatural();
    
    void computeAllCellCost();
		void computeReachability();
    
		
		Eigen::Transform3d getTransformFromRobotPos();
		Eigen::Vector3d getTranformedToRobotFrame(const Eigen::Vector3d& WSPoint);
		bool isInReachableGrid(const Eigen::Vector3d& WSPoint);
		
		bool isReachable(const Eigen::Vector3d& WSPoint);
		bool isReachableWithRA(const Eigen::Vector3d& WSPoint);
		bool isReachableWithLA(const Eigen::Vector3d& WSPoint);
		
		double getCellCostAt(const Eigen::Vector3d& WSPoint);
		
		std::vector<Eigen::Vector3d> getBox();
		void resetCellCost();
		void resetReachability();
		void initReachable();
		
		AgentGrid* mergeWith(AgentGrid* otherGrid);
		std::vector<AgentCell*> getAllReachableCells();
		std::vector<std::pair<double,AgentCell*> > getAllReachableCellsSorted();
		std::vector<AgentCell*> getAllReachableCells(double CostThreshold);
		
		Eigen::Transform3d getRobotOrigin() { return m_RobotOriginPos; }
		//std::tr1::shared_ptr<Configuration> getActualConfig() { return m_ActualConfig; }
		
    
		bool writeToXmlFile(std::string docname);
		bool loadFromXmlFile(std::string docname);
    
	private:
    Robot*                            m_Robot;
    
    Distance*                         m_DistanceCostSpace;
    Visibility*                       m_VisibilityCostSpace;
		Natural*													m_NaturalCostSpace;
    
    std::vector<AgentCell*>           m_DangerCells;
    std::vector<AgentCell*>           m_VisibilityCells;
    std::vector<AgentCell*>           m_ReachableCells;
    
		bool															m_firstDisplay;
		std::tr1::shared_ptr<Configuration> m_ActualConfig;
    std::tr1::shared_ptr<Configuration> m_LastConfig;
		Eigen::Transform3d								m_RobotOriginPos;
	};
}

#endif

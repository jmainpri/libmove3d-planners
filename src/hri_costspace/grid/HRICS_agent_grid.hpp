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

#include <libmove3d/include/Graphic-pkg.h>

namespace HRICS
{
class AgentGrid;

class AgentCell : public Move3D::ThreeDCell
{
public:
    AgentCell();
    AgentCell(int i, Eigen::Vector3i pos , Eigen::Vector3d corner, AgentGrid* grid);
    //AgentCell(const AgentCell& cell);

    ~AgentCell();

    double getCost();
    void setCost(double Cost) { m_Cost = Cost; }
    void setBlankCost();

    double getDistance();
    double getVisibility();
    double getReachability();
    double getCombined();

    void computeDistance();
    void computeVisibility();
    void computeReachability();
    void computeCombined();

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

    void drawOnePoint( bool withTransform );
    void draw(bool transform);
    int setRobotToStoredConfig();

    bool writeToXml(xmlNodePtr cur);
    bool readCellFromXml(xmlNodePtr cur);

protected:

    Eigen::Vector3i getCoord() { return m_Coord; }

    bool getOpen() { return m_Open; }
    void setOpen() { m_Open = true; }

    bool getClosed() { return m_Closed; }
    void setClosed() { m_Closed = true; }

private:

    double radius;

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
    double m_Combined;

    unsigned int m_NbDirections;

    Move3D::confPtr_t m_QStored;

    GLint m_list;
};


class Distance;
class Visibility;
class Natural;

class AgentGrid : public Move3D::ThreeDGrid
{
public:
    AgentGrid();
    AgentGrid( Move3D::Robot* robot, Distance* distCostSpace,Visibility* VisiCostSpace, Natural* NatuCostSpace );
    AgentGrid( std::vector<int> size );
    AgentGrid( double pace, std::vector<double> envSize,
               Move3D::Robot* robot, Distance* distCostSpace,Visibility* VisiCostSpace, Natural* NatuCostSpace);
    AgentGrid( const AgentGrid& grid );

    ~AgentGrid();

    Move3D::ThreeDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z );

    Move3D::Robot*        getRobot();

    Distance*     getDistance();
    Visibility*   getVisibility();
    Natural*      getNatural();

    Eigen::Transform3d    getTransformFromRobotPos();
    Eigen::Vector3d       getTranformedToRobotFrame(const Eigen::Vector3d& WSPoint);

    bool isReachable(const Eigen::Vector3d& WSPoint);

    double getCellCostAt(const Eigen::Vector3d& WSPoint);
    double getCompleteCellCostAt(const Eigen::Vector3d& WSPoint, std::vector<double>& costs);

    std::vector<Eigen::Vector3d> getBox();
    void resetCellCost();
    void resetReachability();
    void initReachable();

    int robotConfigInCell(int i);

    void computeReachability();
    void computeAllCellCost();
    void computeCellVectors();
    void computeRadius();
    void computeCostCombination();

    void draw();

private:

    Move3D::Robot*                    m_Robot;

    double                            m_Radius;

    Distance*                         m_DistanceCostSpace;
    Visibility*                       m_VisibilityCostSpace;
    Natural*                          m_NaturalCostSpace;

    std::vector<AgentCell*>           m_DangerCells;
    std::vector<AgentCell*>           m_VisibilityCells;
    std::vector<AgentCell*>           m_ReachableCells;
    std::vector<AgentCell*>           m_CombinedCells;

    bool                              m_firstDisplay;
    Move3D::confPtr_t                 m_ActualConfig;
    Move3D::confPtr_t                 m_LastConfig;
    Eigen::Transform3d                m_RobotOriginPos;

};
}

#endif

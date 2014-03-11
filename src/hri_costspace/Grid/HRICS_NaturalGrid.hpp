/*
 *  HRICS_NaturalCell.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef HRICS_NATURALGRID_H_
#define HRICS_NATURALGRID_H_

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Grids/ThreeDGrid.hpp"

namespace HRICS
{
class Natural;
class NaturalCell;

class NaturalGrid : public Move3D::ThreeDGrid
{
public:
    NaturalGrid();
    NaturalGrid( std::vector<int> size );
    NaturalGrid(double pace, std::vector<double> envSize,Natural* costSpace);
    NaturalGrid(const NaturalGrid& grid);

    void setGridOrigin();

    Move3D::ThreeDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z );
    void computeAllCellCost();
#ifdef HRI_PLANNER
    /*
         * compute all reacheable point in the grid : either with right or left hand
         */
    void computeReachability();
#endif
    int robotConfigInCell(int i);

    Eigen::Transform3d getTransformFromRobotPos();
    Eigen::Vector3d getTranformedToRobotFrame(const Eigen::Vector3d& WSPoint);
    bool isInReachableGrid(const Eigen::Vector3d& WSPoint);

    bool isReachable(const Eigen::Vector3d& WSPoint);
    bool isReachableWithRA(const Eigen::Vector3d& WSPoint);
    bool isReachableWithLA(const Eigen::Vector3d& WSPoint);

    double getCellCostAt(const Eigen::Vector3d& WSPoint);

    void drawVector( const std::vector< std::pair<double,NaturalCell*> >& cells );
    void draw();
    std::vector<Eigen::Vector3d> getBox();
    void resetCellCost();
    void resetReachability();
    void initReachable();

    NaturalGrid* mergeWith(NaturalGrid* otherGrid);
    std::vector<NaturalCell*> getAllReachableCells();
    std::vector<NaturalCell*> getAllReachableCellsOneArm(bool right_arm);
    std::vector<std::pair<double,NaturalCell*> > getAllReachableCellsSorted();
    std::vector<NaturalCell*> getAllReachableCells(double CostThreshold);

    void setNaturalCostSpace(Natural* NCS) { m_NaturalCostSpace = NCS; setGridOrigin(); }
    Natural* getNaturalCostSpace() { return m_NaturalCostSpace; }
    Eigen::Transform3d getRobotOrigin() { return m_RobotOriginPos; }
    //MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> getActualConfig() { return m_ActualConfig; }

    Move3D::Robot* getRobot();

    bool writeToXmlFile(std::string docname);
    bool loadFromXmlFile(std::string docname);


private:
    Natural*										m_NaturalCostSpace;
    bool											m_firstDisplay;
    Move3D::confPtr_t                               m_ActualConfig;
    Eigen::Transform3d								m_RobotOriginPos;
};
}

#endif

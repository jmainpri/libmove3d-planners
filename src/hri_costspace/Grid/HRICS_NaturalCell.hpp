/*
 *  HRICS_NaturalCell.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef HRICS_NATURALCELL_H_
#define HRICS_NATURALCELL_H_

#include "HRICS_NaturalGrid.hpp"

#include <libmove3d/include/Graphic-pkg.h>

namespace HRICS
{
class NaturalCell : public Move3D::ThreeDCell
{

public:
    NaturalCell();
    NaturalCell(int i, Eigen::Vector3i pos , Eigen::Vector3d corner, NaturalGrid* grid);
    //NaturalCell(const NaturalCell& cell);

    ~NaturalCell() { }

    double getCost();
    double getCost(bool leftHand);
    void setCost(double Cost) { m_Cost = Cost; }
    void setBlankCost();

#ifdef HRI_PLANNER
    void computeReachability();
#endif
    void resetReachable();
    bool getIsLeftArmReachable();

    Eigen::Vector3d getWorkspacePoint();

    void setIsReachable(bool reach) { m_IsReachable = reach; }
    void setIsReachableWithLA(bool reach) { m_IsReachWithLeftArm = reach; }
    void setIsReachableWithRA(bool reach) { m_IsReachWithRightArm = reach; }

    bool isReachable() { return m_IsReachable; }
    bool isReachableWithLA() { return m_IsReachWithLeftArm; }
    bool isReachableWithRA() { return m_IsReachWithRightArm; }

    Eigen::Vector3i getCoord() { return m_Coord; }

    bool getOpen() { return m_Open; }
    void setOpen() { m_Open = true; }

    bool getClosed() { return m_Closed; }
    void setClosed() { m_Closed = true; }
    
    void setUseExternalCost(bool use) { m_use_external_cost=use;  }
    void setExternalCost(double cost) { m_external_cost=cost; }

    void resetExplorationStatus();

    void createDisplaylist();

    void draw();

    bool writeToXml(xmlNodePtr cur);
    bool readCellFromXml(xmlNodePtr cur);

    int setRobotToStoredConfig();

private:

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

    bool m_use_external_cost;
    double m_external_cost;

    unsigned int m_NbDirections;

    Move3D::confPtr_t m_QStored;

    GLint m_list;
};
}


#endif

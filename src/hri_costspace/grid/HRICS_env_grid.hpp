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
#ifndef HRICS_EnvGRID_HPP
#define HRICS_EnvGRID_HPP

#include "API/Grids/gridsAPI.hpp"
#include "API/Device/robot.hpp"

namespace HRICS
{	
class EnvCell;
/**
     @ingroup HRICS
     @brief Plannar HRI Grid
     */
class EnvGrid : public Move3D::TwoDGrid
{
public:

    EnvGrid();
    EnvGrid(double pace, std::vector<double> envSize, bool isHumanCentered);
    EnvGrid(double pace, std::vector<double> envSize, bool isHumanCentered, Move3D::Robot* robot, Move3D::Robot* human);

    //getters and setters
    void setRobot( Move3D::Robot* R) { mRobot = R; }
    Move3D::Robot* getRobot() { return mRobot; }

    void setHuman( Move3D::Robot* R) { mHuman = R; }
    Move3D::Robot* getHuman() { return mHuman; }

    Move3D::Robot* getHumanCylinder() {return humCyl;}
    Move3D::Robot* getRobotCylinder() {return robotCyl;}

    double getNbCellX() {return _nbCellsX;}
    double getNbCellY() {return _nbCellsY;}

    double getHumanMaxDist() {return m_humanMaxDist;}
    double getRobotMaxDist() { return m_robotMaxDist;}

    void setAsNotSorted() {gridIsSorted = false;}

    std::vector<EnvCell*> getHumanAccessibleCells() {return m_HumanAccessible;}

    void dumpVar();

    /**
      * initialisation of the grid ( computing distances and taking obstacle into account)
      */
    void init(std::pair<double,double> minMax);

    /**
      * initialising the part of the grid needed for fused grid use
      */
    void initGrid(Eigen::Vector3d humanPos);

    /**
      * recompute the grid cost if human move
      */
    void recomputeGridWhenHumanMove(Eigen::Vector3d humanPos);

    /**
      * Compute the crown arround the human for reacheability
      */
    void computeHumanRobotReacheability(std::pair<double,double> minMax);

    /*!
                 * \brief Virtual function that creates a new Cell
                 *
                 * \param integer index
                 * \param integer x
                 * \param integer y
                 * \param integer z
                 */
    Move3D::TwoDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y );

    /**
      * using the method of distance propagation to compute distances
      */
    void computeDistances(EnvCell* cell, bool isHuman);

    /**
      * using the method of distance propagation to compute distances for robot
      */
    void  computeRobotDistances(EnvCell* cell);

    /**
      * using the method of distance propagation to compute distances for human
      */
    void  computeHumanDistances(EnvCell* cell);

    /**
      * drawing the grid
      */
    void draw();

    /**
      * call setBlankCost() in each cell
      */
    void setCellsToblankCost();

    /**
      * call resetexplorationstatus() in each cell
      */
    void initAllCellState();

    /**
      * call resetTrajs() in each cell
      */
    void initAllTrajs();

    /**
      * call resetexplorationstatus() in each cell
      */
    void initAllReachability();

    /**
      * sort the cells of the grid
      */
    std::vector<std::pair<double,EnvCell*> > getSortedGrid();

private:
    /**
      * the Robot
      */
    Move3D::Robot* mRobot;

    /**
      * the human
      */
    Move3D::Robot* mHuman;

    /**
      * the robot cylinder
      */
    Move3D::Robot* robotCyl;

    /**
      * the human cylinder
      */
    Move3D::Robot* humCyl;

    /**
      * the distance maximum that can walk the human in the grid
      */
    double m_humanMaxDist;

    /**
      * the maximum distance that the robot can navigate in the grid
      */
    double m_robotMaxDist;

    /**
      * the list of cell where the human is accessible
      */
    std::vector<EnvCell*> m_HumanAccessible;

    /**
      * the list of cells where the robot is accessible
      */
    std::vector<EnvCell*> m_RobotAccessible;

    /**
      * the list of sorted cells
      */
    std::vector<std::pair<double,EnvCell*> > sortedGrid;

    /**
      * if the grid is already sorted
      */
    bool gridIsSorted;
};

/**
     @ingroup HRICS
     @brief Plannar HRI Cell
     */
class EnvCell : public Move3D::TwoDCell
{

public:
    EnvCell();
    EnvCell(int i, Eigen::Vector2i coord, Eigen::Vector2d corner, EnvGrid* grid);

    ~EnvCell() { }

    //#######################
    //getters et setters ####
    //#######################

    //human distance computing
    void setHumanDist(double value) {m_humanDist = value; }
    double getHumanDist() {return m_humanDist; }
    void setHumanDistIsComputed() {m_humanDistIsComputed = true;}
    bool isHumanDistComputed() {return m_humanDistIsComputed;}

    // robot distance computing
    void setRobotDist(double value) {m_robotDist = value; }
    double getRobotDist() {return m_robotDist; }
    void setRobotDistIsComputed() {m_robotDistIsComputed = true;}
    bool isRobotDistComputed() {return m_robotDistIsComputed;}

    // distance propagation algorithm
    bool getOpen() { return _Open; }
    void setOpen() { _Open = true; }

    bool getClosed() { return _Closed; }
    void setClosed() { _Closed = true; }

    void resetExplorationStatus() { _Open = false; _Closed = false; }

    // Trajectories

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > getHumanVectorTraj() {return humanVectorTraj;}
    void setHumanVectorTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > _humanVectorTraj) {humanVectorTraj = _humanVectorTraj; }

    std::vector<EnvCell*> getHumanTraj() {return humanTraj;}
    void setHumanTraj(std::vector<EnvCell*> _humanTraj) {humanTraj = _humanTraj; }

    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > getRobotVectorTraj() {return robotVectorTraj;}
    void setRobotVectorTraj(std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > _RobotVectorTraj) {robotVectorTraj = _RobotVectorTraj; }

    std::vector<EnvCell*> getRobotTraj() {return robotTraj;}
    void setRobotTraj(std::vector<EnvCell*> _robotTraj) {robotTraj = _robotTraj; }

    //accessibility
    bool isHumAccessible() {return m_isHumanAccessible;}
    void setHumAccessible(bool value) {m_isHumanAccessible = value;}

    bool isRobAccessible() {return m_isRobotAccessible;}
    void setRobAccessible(bool value) {m_isRobotAccessible = value;}

    // human robot reachability (the crown)
    void setHumanRobotReacheable(std::vector<EnvCell*> value) {initHumanRobotReacheable = value;}
    std::vector<EnvCell*> getHumanRobotReacheable() {return initHumanRobotReacheable;}
    void addToHumanRobotReacheable(EnvCell* cell) {initHumanRobotReacheable.push_back(cell);}
    void clearHumanRobotReacheable() {initHumanRobotReacheable.clear();}

    void setCurrentHumanRobotReacheable(std::vector<EnvCell*> value) {currentHumanRobotReacheable = value;}
    std::vector<EnvCell*> getCurrentHumanRobotReacheable() {return currentHumanRobotReacheable;}
    void addToCurrentHumanRobotReacheable(EnvCell* cell) {currentHumanRobotReacheable.push_back(cell);}
    void clearCurrentHumanRobotReacheable() {currentHumanRobotReacheable.clear();}

    //special grid usage (slices)
    std::pair<double,EnvCell*> getRobotBestPos(){return robotBestPos;}
    void setRobotBestPos(std::pair<double,EnvCell*> value){robotBestPos = value;}

    void setAngleForHumanComming(double value) {angleForHumanComming = value;}
    double getAngleForHumanComming(){return angleForHumanComming;}

    //other
    void setBlankCost() { mCostIsComputed = false; }
    void setCost(double value) {mCost = value; }
    double getCost() {return mCost; }

    Eigen::Vector2i getCoord() { return _Coord; }

    std::vector<double> getRandomVector(){return randomVectorPoint;}



    //#######################
    // others ###############
    //#######################

    /**
                  * test if there is a collision with a BB (the cylinders) for the human
                  */
    void computeHumanReach();

    /**
                  * test if there is a collision with a BB (the cylinders) for the robot
                  */
    void computeRobotReach();

    /**
                  * compute partial cost
                  */
    double computeCost();

    /**
                  * compute best robot pos
                  */
    bool computeBestRobotPos();

    /**
                  * reset the reacheability computing
                  */
    void resetReacheability();

    /**
                  * reset the Trajectories computing
                  */
    void resetTraj();

    /**
                  * find the neighbors of this cell (used when computing distance propagation)
                  */
    std::vector<EnvCell*> getNeighbors(bool isHuman);

    /**
                  * compute distance between two cells
                  */
    double computeDist(EnvCell* neighCell);

    /**
                  * add a point to the random vector in order to draw it (the red arrows)
                  */
    void addPoint(double Rz);

    /**
                  * return the crown arround the cell taking the min and max value
                  */
    std::vector<EnvCell*> getCrown(double min, double max);

private:
    /**
                  * the x y coord of the cell
                  */
    Eigen::Vector2i _Coord;

    /**
                  * the status of the cell
                  */
    bool _Open;

    /**
                  * the status of the cell
                  */
    bool _Closed;

    /**
                  * if the cost is computed
                  */
    bool mCostIsComputed;

    /**
                  * the computed cost
                  */
    double mCost;

    /**
                  * if the cell is accessible for the human
                  */
    bool m_isHumanAccessible;

    /**
                  * if the cell is accessible for the robot
                  */
    bool m_isRobotAccessible;

    /**
                  * if the accessibility is computed
                  */
    bool m_reachComputed;

    /**
                  * if the human distance is computed
                  */
    bool m_humanDistIsComputed;

    /**
                  * the human distance
                  */
    double m_humanDist;

    /**
                  * the human trajectory (with cells)
                  */
    std::vector<EnvCell*> humanTraj;

    /**
                  * the human trajectory (with vector2D)
                  */
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > humanVectorTraj;

    /**
                  * if the robot distance is computed
                  */
    bool m_robotDistIsComputed;

    /**
                  * the robot distance
                  */
    double m_robotDist;

    /**
                  * the robot trajectory (with cells)
                  */
    std::vector<EnvCell*> robotTraj;

    /**
                  * the robot trajectory (with vector2D)
                  */
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > robotVectorTraj;

    /**
                  *the vector of the tested points
                  */
    std::vector<double> randomVectorPoint;

    /**
                  * the human robot reacheability (crown) initialy initialised
                  */
    std::vector<EnvCell*> initHumanRobotReacheable;

    /**
                  * the human robot reacheability (crown) currently used
                  */
    std::vector<EnvCell*> currentHumanRobotReacheable;

    /**
                  * the robot best position in the crown
                  */
    std::pair<double,EnvCell*> robotBestPos;

    /**
                  * the angle from witch the human come, used in directed slice grid usage
                  */
    double angleForHumanComming;

};
}



#endif // HRICS_TWODGRID_HPP

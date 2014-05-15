#ifndef HRICS_TWODGRID_HPP
#define HRICS_TWODGRID_HPP

#include "API/Grids/gridsAPI.hpp"
#include "API/Search/AStar/AStar.hpp"
#include "API/Device/robot.hpp"

namespace HRICS
{	
/**
     @ingroup HRICS
     @brief Plannar HRI Grid
     */
class PlanGrid : public Move3D::TwoDGrid
{
public:
    PlanGrid( Move3D::Robot* R, double pace, std::vector<double> envSize);

    Move3D::TwoDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y );
    
    void writeToOBPlane();
    void draw();
    void setRobotToStoredConfig();
    void reset();

    Move3D::Robot* getRobot() { return mRobot; }

private:
    Move3D::Robot* mRobot;
};

/**
     @ingroup HRICS
     @brief Plannar HRI Cell
     */
class PlanCell : public Move3D::TwoDCell
{

public:
    PlanCell(int i, Eigen::Vector2i coord, Eigen::Vector2d corner, PlanGrid* grid);

    ~PlanCell() { }

    Eigen::Vector2i getCoord() { return _Coord; }
    
    double getCost(); /* { std::cout << " Warning not implemented"  << std::endl; }*/
    void resetCost() { mCostIsComputed = false; }

    bool getOpen() { return _Open; }
    void setOpen() { _Open = true; }
    bool getClosed() { return _Closed; }
    void setClosed() { _Closed = true; }
    void resetExplorationStatus() { _Open = false; _Closed = false; }
    
    bool isValid();
    void resetIsValid() { mIsCellTested = false; }
    
private:
    
    Move3D::confPtr_t setRobotAtCenter();

    Eigen::Vector2i _Coord;

    bool _Open;
    bool _Closed;

    bool mCostIsComputed;
    double mCost;
    
    bool mIsCellTested;
    bool mIsValid;
};

/**
     @ingroup HRICS
     @brief Plannar HRI State
     */
class PlanState : public Move3D::State
{
public:
    PlanState() {}
    PlanState( Eigen::Vector2i cell, PlanGrid* grid);
    PlanState( PlanCell* cell , PlanGrid* grid);

    std::vector<Move3D::State*> getSuccessors(Move3D::State* s);

    bool isLeaf();		/* leaf control for an admissible heuristic function; the test of h==0*/
    bool equal(Move3D::State* other);
    bool isValid();

    void setClosed(std::vector<PlanState*>& closedStates,std::vector<PlanState*>& openStates);
    bool isColsed(std::vector<PlanState*>& closedStates);

    void setOpen(std::vector<PlanState*>& openStates);
    bool isOpen(std::vector<PlanState*>& openStates);

    void reset();
    void print();

    PlanCell* getCell() { return _Cell; }

protected:
    double computeLength( Move3D::State *parent);       /* g */
    double computeHeuristic( Move3D::State *parent = NULL, Move3D::State* goal = NULL);    /* h */

private:
    PlanGrid* _Grid;
    PlanCell* _Cell;
    PlanCell* _PreviousCell;
};
}

#endif // HRICS_TWODGRID_HPP

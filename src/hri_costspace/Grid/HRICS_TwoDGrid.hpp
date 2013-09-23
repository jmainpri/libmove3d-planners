#ifndef HRICS_TWODGRID_HPP
#define HRICS_TWODGRID_HPP

#include "API/planningAPI.hpp"
#include "API/Grids/gridsAPI.hpp"

namespace HRICS
{	
/**
     @ingroup HRICS
     @brief Plannar HRI Grid
     */
class PlanGrid : public API::TwoDGrid
{
public:
    PlanGrid(Robot* R, double pace, std::vector<double> envSize);

    API::TwoDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y );
    
    void writeToOBPlane();
    void draw();
    void setRobotToStoredConfig();
    void reset();

    Robot* getRobot() { return mRobot; }

private:
    Robot* mRobot;
};

/**
     @ingroup HRICS
     @brief Plannar HRI Cell
     */
class PlanCell : public API::TwoDCell
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
    
    confPtr_t setRobotAtCenter();

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
class PlanState : public API::State
{
public:
    PlanState() {}
    PlanState( Eigen::Vector2i cell, PlanGrid* grid);
    PlanState( PlanCell* cell , PlanGrid* grid);

    std::vector<API::State*> getSuccessors(API::State* s);

    bool isLeaf();		/* leaf control for an admissible heuristic function; the test of h==0*/
    bool equal(API::State* other);
    bool isValid();

    void setClosed(std::vector<PlanState*>& closedStates,std::vector<PlanState*>& openStates);
    bool isColsed(std::vector<PlanState*>& closedStates);

    void setOpen(std::vector<PlanState*>& openStates);
    bool isOpen(std::vector<PlanState*>& openStates);

    void reset();
    void print();

    PlanCell* getCell() { return _Cell; }

protected:
    double computeLength(API::State *parent);       /* g */
    double computeHeuristic(API::State *parent = NULL ,API::State* goal = NULL);    /* h */

private:
    PlanGrid* _Grid;
    PlanCell* _Cell;
    PlanCell* _PreviousCell;
};
}

#endif // HRICS_TWODGRID_HPP

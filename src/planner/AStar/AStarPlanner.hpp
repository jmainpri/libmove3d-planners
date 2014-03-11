#ifndef ASTARPLANNER_HPP
#define ASTARPLANNER_HPP

#include "API/Trajectory/trajectory.hpp"
#include "API/Grids/gridsAPI.hpp"
#include "API/Search/AStar/State.hpp"

#include "planner/planner.hpp"

#include <Eigen/StdVector>

namespace Move3D {

class PlanGrid : public Move3D::TwoDGrid
{
public:
    PlanGrid(Robot* R, double pace, std::vector<double> envSize);

    Move3D::TwoDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y );

    void draw();
    void setRobotToStoredConfig();
    void reset();
    std::pair<double,double> getMinMaxCost();

    Robot* getRobot() { return robot_; }

private:
    Robot* robot_;
};

class PlanCell : public Move3D::TwoDCell
{

public:
    PlanCell(int i, Eigen::Vector2i coord, Eigen::Vector2d corner, PlanGrid* grid);

    ~PlanCell() { }

    Eigen::Vector2i getCoord() { return coord_; }

    double getCost(); /* { std::cout << " Warning not implemented"  << std::endl; }*/
    void resetCost() { cost_is_computed_ = false; }
    void setCost( double cost ) { cost_is_computed_ = true; cost_ = cost; }

    bool getOpen() { return open_; }
    void setOpen() { open_ = true; }
    bool getClosed() { return closed_; }
    void setClosed() { closed_ = true; }
    void resetExplorationStatus() { open_ = false; closed_ = false; }

    bool isValid();
    void resetIsValid() { is_cell_tested_ = false; }

private:

    confPtr_t setRobotAtCenter();

    Eigen::Vector2i coord_;

    bool open_;
    bool closed_;

    bool cost_is_computed_;
    double cost_;

    bool is_cell_tested_;
    bool is_valid_;
};

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

    PlanCell* getCell() { return cell_; }

protected:
    double computeLength(Move3D::State *parent);       /* g */
    double computeHeuristic(Move3D::State *parent = NULL ,Move3D::State* goal = NULL);    /* h */

private:
    PlanGrid* grid_;
    PlanCell* cell_;
    PlanCell* previous_cell_;
};

class AStarPlanner : public Planner
{
public:
    AStarPlanner(Robot* robot);
    ~AStarPlanner();

    Move3D::Trajectory* computeRobotTrajectory( confPtr_t source, confPtr_t target );
    Move3D::Trajectory* getSimplePath( std::vector<double> goal, std::vector<std::vector<double> >& path );

    //! change implementation here
    unsigned int run() { return 0; }
    unsigned int init();

    void reset();
    void draw();
    double pathCost();
    void allow_smoothing(bool state);

    void set_pace( double pace ) { pace_ = pace; }

    PlanGrid* getGrid() { return grid_; }

private:

    bool computeAStarIn2DGrid( Eigen::Vector2d source, Eigen::Vector2d target );
    bool solveAStar( PlanState* start, PlanState* goal );

    std::vector<double> env_size_;
    double max_radius_;
    double pace_;
    PlanGrid* grid_;
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > path_;
    std::vector<Move3D::TwoDCell*> cell_path_;
};

}

#endif // ASTARPLANNER_HPP

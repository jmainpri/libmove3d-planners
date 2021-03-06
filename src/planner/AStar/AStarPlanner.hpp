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
    PlanGrid(Robot* R, double pace, std::vector<double> envSize, bool print_cost=true);

    Move3D::TwoDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y );

    void setRobotToStoredConfig();
    void reset();
    std::pair<double,double> getMinMaxCost();
    void draw();
    void setCostBounds(double min, double max) { use_given_bounds_ = true; min_cost_ = min; max_cost_ = max; }

    Robot* getRobot() { return robot_; }    

private:
    Robot* robot_;
    bool print_cost_;
    bool use_given_bounds_;
    double min_cost_;
    double max_cost_;
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

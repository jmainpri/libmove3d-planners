#ifndef HRICS_DETOURS_HPP
#define HRICS_DETOURS_HPP

#include "API/Device/robot.hpp"
#include "planner/AStar/AStarPlanner.hpp"

namespace HRICS
{

class Detours
{
public:
    Detours();

    void planAStar();

    void draw();

private:
    Robot* robot_;
    PlanGrid* grid_;
};

}

#endif // HRICS_DETOURS_HPP

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
    ~Detours();

    void planAStar();

    void draw();

private:
    Move3D::Robot* robot_;
    Move3D::PlanGrid* grid_;
};

}

#endif // HRICS_DETOURS_HPP

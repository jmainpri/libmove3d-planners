#ifndef HRICS_WS_OCCUPANCYCOSTSPACE_HPP
#define HRICS_WS_OCCUPANCYCOSTSPACE_HPP

#include "HRICS_WorkspaceOccupancy.hpp"

void HRICS_initOccupancyPredictionFramework();
double HRICS_getPredictionOccupancyCost(Configuration& q);

namespace HRICS
{
class HumanPredictionCostSpace {

public:
    HumanPredictionCostSpace( Robot* robot, WorkspaceOccupancyGrid* occup_grid);
    ~HumanPredictionCostSpace();

    void draw();

    double getCost(Configuration& q);
    double getCostFromActiveJoints(Configuration& q);

private:
    void draw_sampled_points();
    void sampleRobotPoints();
    void setActiveJoints();

    Robot* m_robot;
    WorkspaceOccupancyGrid* m_ws_occupancy;
    BodySurfaceSampler* m_surface_sampler;
    std::vector<int> m_active_joints;
};
}

extern HRICS::HumanPredictionCostSpace* global_humanPredictionCostSpace;

#endif // HRICS_WS_OCCUPANCYCOSTSPACE_HPP

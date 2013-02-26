#ifndef HRICS_WS_OCCUPANCYCOSTSPACE_HPP
#define HRICS_WS_OCCUPANCYCOSTSPACE_HPP

#include "HRICS_WorkspaceOccupancy.hpp"

namespace HRICS
{
class HumanPredictionCostSpace {

public:
    HumanPredictionCostSpace( const std::string& robotname, WorkspaceOccupancyGrid* occup_grid);
    ~HumanPredictionCostSpace();

    void draw();

    double cost(Configuration& q);

private:
    void draw_sampled_points();
    void sampleRobotPoints();

    Robot* m_robot;
    WorkspaceOccupancyGrid* m_ws_occupancy;
    BodySurfaceSampler* m_surface_sampler;
};
}

extern HRICS::HumanPredictionCostSpace* global_HumanPredictionCostSpace;

#endif // HRICS_WS_OCCUPANCYCOSTSPACE_HPP

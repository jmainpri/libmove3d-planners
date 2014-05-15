#ifndef HRICS_WS_OCCUPANCYCOSTSPACE_HPP
#define HRICS_WS_OCCUPANCYCOSTSPACE_HPP

#include "HRICS_workspace_occupancy.hpp"

double HRICS_getPredictionOccupancyCost(Move3D::Configuration& q);

namespace HRICS
{

class HumanPredictionCostSpace {

public:
    HumanPredictionCostSpace( Move3D::Robot* robot, WorkspaceOccupancyGrid* occup_grid);
    ~HumanPredictionCostSpace();

    void draw();

    double getCost( Move3D::Configuration& q ) const;
    double getCostFromActiveJoints( Move3D::Configuration& q ) const;
    double getCurrentOccupationCost( Move3D::Configuration& q ) const;

private:
    void draw_sampled_points();
    void sampleRobotPoints();
    void setActiveJoints();

    Move3D::Robot* m_robot;
    WorkspaceOccupancyGrid* m_ws_occupancy;
    Move3D::BodySurfaceSampler* m_surface_sampler;
    std::vector<int> m_active_joints;
};

}

extern HRICS::HumanPredictionCostSpace* global_humanPredictionCostSpace;

#endif // HRICS_WS_OCCUPANCYCOSTSPACE_HPP

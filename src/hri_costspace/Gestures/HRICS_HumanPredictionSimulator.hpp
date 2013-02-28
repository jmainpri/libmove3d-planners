#ifndef HRICS_HUMANPREDICTIONSIMULATOR_HPP
#define HRICS_HUMANPREDICTIONSIMULATOR_HPP

#include "HRICS_RecordMotion.hpp"
#include "HRICS_ClassifyMotion.hpp"
#include "HRICS_WorkspaceOccupancy.hpp"

void HRICS_initOccupancyPredictionFramework();

namespace HRICS
{
class HumanPredictionSimulator
{
public:
    HumanPredictionSimulator( ClassifyMotion* classifier );

    void loadHumanTrajectory( const motion_t& motion );
    void run();

private:
    bool updateMotion();
    void predictVoxelOccupancy();

    motion_t m_motion;
    Eigen::MatrixXd m_current_traj;
    int m_increment;
    ClassifyMotion* m_classifier;
    WorkspaceOccupancyGrid* m_occupacy_grid;
};
}

extern HRICS::HumanPredictionSimulator* global_humanPredictionSimulator;

#endif // HRICS_HUMANPREDICTIONSIMULATOR_HPP

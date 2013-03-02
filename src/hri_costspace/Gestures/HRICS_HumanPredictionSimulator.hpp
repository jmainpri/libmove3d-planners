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
    HumanPredictionSimulator( Robot* robot, Robot* human, ClassifyMotion* classifier, WorkspaceOccupancyGrid* occupacy_grid );

    void loadHumanTrajectory( const motion_t& motion );
    void run();

private:
    bool updateMotion();
    void predictVoxelOccupancy();
    void loadGoalConfig();
    void runStomp(confPtr_t q_goal);
    void executeStomp(const API::Trajectory& path, bool to_end=false);
    int getBestPathId();

    Robot* m_robot;
    Robot* m_human;
    motion_t m_motion;
    Eigen::MatrixXd m_current_traj;
    int m_human_increment;
    ClassifyMotion* m_classifier;
    WorkspaceOccupancyGrid* m_occupacy_grid;
    confPtr_t m_q_start;
    std::vector<confPtr_t> m_goal_config;
    std::vector<API::Trajectory> m_paths;
    int m_max_stomp_iter;
};
}

extern HRICS::HumanPredictionSimulator* global_humanPredictionSimulator;

#endif // HRICS_HUMANPREDICTIONSIMULATOR_HPP

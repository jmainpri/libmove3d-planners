#ifndef HRICS_HUMANPREDICTIONSIMULATOR_HPP
#define HRICS_HUMANPREDICTIONSIMULATOR_HPP

#include "HRICS_RecordMotion.hpp"
#include "HRICS_ClassifyMotion.hpp"
#include "HRICS_WorkspaceOccupancy.hpp"

#include "API/Trajectory/trajectory.hpp"

void HRICS_initOccupancyPredictionFramework();

namespace HRICS
{
class HumanPredictionSimulator
{
public:
    HumanPredictionSimulator( Robot* robot, Robot* human, RecordMotion* recorder, ClassifyMotion* classifier, WorkspaceOccupancyGrid* occupacy_grid );

    void loadHumanTrajectory( const motion_t& motion );
    int classifyMotion( const motion_t& motion );
    void runVoxelOccupancy();
    double run();

private:
    bool updateMotion();
    void predictVoxelOccupancy();
    void loadGoalConfig();
    API::Trajectory setTrajectoryToMainRobot(const API::Trajectory& traj) const;
    void runMultipleStomp( int iter );
    void runParallelStomp( int iter, int id_goal );
    void runStandardStomp(int iter, int id_goal );
    void execute(const API::Trajectory& path, bool to_end=false);
    int getBestPathId();
    void setMatrixCol(Eigen::MatrixXd& mat, int j, confPtr_t q);
    void setHumanConfig( confPtr_t q );
    void printCosts() const;

    Robot* m_robot;
    Robot* m_human;
    motion_t m_motion;
    Eigen::MatrixXd m_current_human_traj;
    int m_human_increment;
    RecordMotion* m_recorder;
    ClassifyMotion* m_classifier;
    WorkspaceOccupancyGrid* m_occupacy_grid;

    confPtr_t m_q_start;
    std::vector<confPtr_t> m_goal_config;
    std::vector<API::Trajectory> m_paths;

    API::Trajectory m_executed_path;
    std::vector< std::vector<double> > m_cost;
    std::vector< std::vector<double> > m_colors;

    bool m_use_previous_trajectory;
    bool m_is_scenario_init;

    int m_best_path_id;
    int m_robot_steps_per_exection;
    double m_robot_step;
};
}

extern HRICS::HumanPredictionSimulator* global_humanPredictionSimulator;

#endif // HRICS_HUMANPREDICTIONSIMULATOR_HPP

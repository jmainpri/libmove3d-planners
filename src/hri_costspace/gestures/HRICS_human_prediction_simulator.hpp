#ifndef HRICS_HUMANPREDICTIONSIMULATOR_HPP
#define HRICS_HUMANPREDICTIONSIMULATOR_HPP

#include "HRICS_record_motion.hpp"
#include "HRICS_classify_motion.hpp"
#include "HRICS_workspace_occupancy.hpp"

#include "API/Trajectory/trajectory.hpp"

void HRICS_initOccupancyPredictionFramework();

namespace HRICS
{
class HumanPredictionSimulator
{
public:
    HumanPredictionSimulator( Move3D::Robot* robot, Move3D::Robot* human, RecordMotion* recorder, ClassifyMotion* classifier, WorkspaceOccupancyGrid* occupacy_grid );

    void loadHumanTrajectory( const motion_t& motion );
    int classifyMotion( const motion_t& motion );
    void runVoxelOccupancy();
    double run();

private:
    bool updateMotion();
    void predictVoxelOccupancy();
    void loadGoalConfig();
    Move3D::Trajectory setTrajectoryToMainRobot(const Move3D::Trajectory& traj) const;
    void runMultipleStomp( int iter );
    void runParallelStomp( int iter, int id_goal );
    void runStandardStomp(int iter, int id_goal );
    void execute(const Move3D::Trajectory& path, bool to_end=false);
    int getBestPathId();
    void setMatrixCol(Eigen::MatrixXd& mat, int j, Move3D::confPtr_t q);
    void setHumanConfig( Move3D::confPtr_t q );
    void printCosts() const;

    Move3D::Robot* m_robot;
    Move3D::Robot* m_human;
    motion_t m_motion;
    Eigen::MatrixXd m_current_human_traj;
    int m_human_increment;
    RecordMotion* m_recorder;
    ClassifyMotion* m_classifier;
    WorkspaceOccupancyGrid* m_occupacy_grid;

    Move3D::confPtr_t m_q_start;
    std::vector<Move3D::confPtr_t> m_goal_config;
    std::vector<Move3D::Trajectory> m_paths;

    Move3D::Trajectory m_executed_path;
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

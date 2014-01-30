#ifndef HRICS_RUN_MULTIPLE_STOMP_HPP
#define HRICS_RUN_MULTIPLE_STOMP_HPP

#include "API/Device/robot.hpp"
#include "API/Trajectory/trajectory.hpp"

namespace HRICS
{

class MultipleStomp
{
public:
    MultipleStomp();
    ~MultipleStomp() { }

    void multipleRun( int nb_runs );
    void initializeNoisy();
    bool run();
    void saveTrajsToFile();
    void loadTrajsFromFile(int nb_trajs=10);

    const std::vector<API::Trajectory>& getBestTraj() { return best_traj_; }

private:
    Robot* robot_;
    std::vector<API::Trajectory> best_traj_;
    std::string folder_;
};

}

#endif // HRICS_RUN_MULTIPLE_STOMP_HPP
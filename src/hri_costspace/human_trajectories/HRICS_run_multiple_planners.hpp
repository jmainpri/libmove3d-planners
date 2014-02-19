#ifndef HRICS_RUN_MULTIPLE_STOMP_HPP
#define HRICS_RUN_MULTIPLE_STOMP_HPP

#include "API/Device/robot.hpp"
#include "API/Trajectory/trajectory.hpp"

namespace HRICS
{
enum planner_t {
    stomp=0,
    astar=1,
    rrt=2 };

class MultiplePlanners
{
public:
    MultiplePlanners(Robot* robot);
    ~MultiplePlanners() { }

    bool run();
    void multipleRun( std::string folder, int nb_runs );
    void saveTrajsToFile( std::string folder );
    void loadTrajsFromFile( std::string folder, int nb_max_files=-1 );

    void initializeNoisy();

    const std::vector<API::Trajectory>& getBestTrajs() { return best_traj_; }
    void setPlannerType( planner_t planner ) { planner_type_ = planner; }
    void clearTrajs() { best_traj_.clear(); }

    void draw();

private:

    bool runStomp();
    bool runAStar();
    bool runRRT();

    Robot* robot_;
    std::vector<API::Trajectory> best_traj_;
    planner_t planner_type_;
};

}

#endif // HRICS_RUN_MULTIPLE_STOMP_HPP

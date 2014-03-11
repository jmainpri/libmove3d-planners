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
    MultiplePlanners( Move3D::Robot* robot);
    ~MultiplePlanners() { }

    bool run();
    void multipleRun( std::string folder, int nb_runs );
    void saveTrajsToFile( std::string folder );
    void loadTrajsFromFile( std::string folder, int nb_max_files=-1 );

    void initializeNoisy();

    const std::vector<Move3D::Trajectory>& getBestTrajs() { return best_traj_; }
    const std::vector<Move3D::Trajectory>& getAllTrajs();
    void setPlannerType( planner_t planner ) { planner_type_ = planner; }
    void clearTrajs() { best_traj_.clear(); }
    void setStompInit( const Move3D::Trajectory& t ) { init_stomp_ = t; }

    void draw();

private:

    bool runStomp();
    bool runAStar();
    bool runRRT();

    Move3D::Robot* robot_;
    std::vector<Move3D::Trajectory> best_traj_;
    std::vector<Move3D::Trajectory> all_traj_;
    planner_t planner_type_;
    Move3D::Trajectory init_stomp_;
};

}

#endif // HRICS_RUN_MULTIPLE_STOMP_HPP

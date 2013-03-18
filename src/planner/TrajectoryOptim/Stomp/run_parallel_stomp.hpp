#ifndef PARALLEL_STOMP_HPP
#define PARALLEL_STOMP_HPP

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompTrajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompCost.hpp"

#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"

#include "collision_space/CollisionSpace.hpp"

#include <boost/thread/mutex.hpp>

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

class stompContext
{
public:
    stompContext( Robot* robot, const CollisionSpace* coll_space, const std::vector<int>& planner_joints, const std::vector<CollisionPoint>& collision_points );
    ~stompContext();

    bool initRun( API::Trajectory& T );
    void run();

    void setParallelRobots( const std::vector<Robot*>& robots );
    void setPathColor( const std::vector<double>& color ) { m_color = color; }

    MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::StompOptimizer> getStompOptimizer() const { return m_stomp; }

private:

    Robot*                                  m_robot;
    MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::StompOptimizer>  m_stomp;
    stomp_motion_planner::StompParameters*  m_stompparams;
    ChompPlanningGroup*                     m_chompplangroup;
    ChompTrajectory*                        m_chomptraj;
    int                                     m_nb_points;

    bool                                    m_use_iteration_limit;
    int                                     m_max_iterations;
    bool                                    m_use_costspace;
    int                                     m_runid;
    std::vector<double>                     m_color;

    std::vector<Robot*>                     m_parallel_robots;

    std::vector<CollisionPoint>             m_collision_points;
    std::vector<int>                        m_planner_joints;
    const CollisionSpace*                   m_coll_space;

};

class stompRun
{
public:
    stompRun(const CollisionSpace* coll_space, std::vector<int> planner_joints, const std::vector<CollisionPoint>& collision_points );
    ~stompRun();

    void setPool( const std::vector<Robot*>& robots );
    void run( int id, API::Trajectory &T );

    void start();
    void isRunning();

    void setPathColor( int id, const std::vector<double>& color );
    void setRobotPool( int id, const std::vector<Robot*>& robots );

    API::Trajectory getBestTrajectory( int id );

    void lockDraw() { m_mtx_draw.lock(); }
    void unlockDraw() { m_mtx_draw.unlock(); }

private:
    bool setParallelStompEnd(int id);

    boost::mutex                             m_mtx_draw;
    boost::mutex                             m_mtx_set_end;
    boost::mutex                             m_mtx_multi_end;
    std::vector<bool>                        m_is_thread_running;

    std::vector<Robot*>                      m_robots;
    std::vector<stompContext*>               m_stomps;

    std::vector<int>                         m_planner_joints;
    const CollisionSpace*                    m_coll_space;
    std::vector<CollisionPoint>              m_collision_points;
};

void srompRun_MultipleParallel();
void srompRun_OneParallel();

extern stompRun* global_stompRun;

#endif // PARALLEL_STOMP_HPP

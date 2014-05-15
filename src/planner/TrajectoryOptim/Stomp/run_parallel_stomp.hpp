#ifndef PARALLEL_STOMP_HPP
#define PARALLEL_STOMP_HPP

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompTrajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompCost.hpp"

#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"

#include "collision_space/collision_space.hpp"

#include <boost/thread/mutex.hpp>

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

namespace stomp_motion_planner
{

class stompContext
{
public:
    stompContext( Move3D::Robot* robot, const Move3D::CollisionSpace* coll_space, const std::vector<int>& planner_joints, const std::vector<Move3D::CollisionPoint>& collision_points );
    ~stompContext();

    bool initRun( Move3D::Trajectory& T );
    void run();

    void setParallelRobots( const std::vector<Move3D::Robot*>& robots );
    void setPathColor( const std::vector<double>& color ) { m_color = color; }

    MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::StompOptimizer> getStompOptimizer() const { return m_stomp; }

    Move3D::Robot* getRobot() { return m_robot; }

private:

    Move3D::Robot*                                  m_robot;
    MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::StompOptimizer>  m_stomp;
    stomp_motion_planner::StompParameters*  m_stompparams;
    Move3D::ChompPlanningGroup*             m_chompplangroup;
    Move3D::ChompTrajectory*                m_chomptraj;
    int                                     m_nb_points;

    bool                                    m_use_iteration_limit;
    int                                     m_max_iterations;
    bool                                    m_use_time_limit;
    double                                  m_max_time;
    bool                                    m_use_costspace;
    int                                     m_runid;
    std::vector<double>                     m_color;

    std::vector<Move3D::Robot*>             m_parallel_robots;

    std::vector<Move3D::CollisionPoint>     m_collision_points;
    std::vector<int>                        m_planner_joints;
    const Move3D::CollisionSpace*           m_coll_space;

};

class stompRun
{
public:
    stompRun(const Move3D::CollisionSpace* coll_space, std::vector<int> planner_joints, const std::vector<Move3D::CollisionPoint>& collision_points = std::vector<Move3D::CollisionPoint>() );
    ~stompRun();

    void setPool( const std::vector<Move3D::Robot*>& robots );
    bool run( int id, Move3D::Trajectory &T );

    void start();
    void isRunning();

    void setPathColor( int id, const std::vector<double>& color );
    void setRobotPool( int id, const std::vector<Move3D::Robot*>& robots );

    Move3D::Trajectory getBestTrajectory( int id );

    void lockDraw() { m_mtx_draw.lock(); }
    void unlockDraw() { m_mtx_draw.unlock(); }

    stompContext* getContext( int i ) { return m_stomps[i]; }

private:
    bool setParallelStompEnd(int id);

    boost::mutex                             m_mtx_draw;
    boost::mutex                             m_mtx_set_end;
    boost::mutex                             m_mtx_multi_end;
    std::vector<bool>                        m_is_thread_running;

    std::vector<Move3D::Robot*>              m_robots;
    std::vector<stompContext*>               m_stomps;

    std::vector<int>                         m_planner_joints;
    const Move3D::CollisionSpace*            m_coll_space;
    std::vector<Move3D::CollisionPoint>      m_collision_points;
};

void srompRun_MultipleParallel();
void srompRun_OneParallel();

extern stompRun* global_stompRun;

}

#endif // PARALLEL_STOMP_HPP

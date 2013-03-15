#ifndef PARALLEL_STOMP_HPP
#define PARALLEL_STOMP_HPP

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompTrajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompCost.hpp"

#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"

#include "collision_space/CollisionSpace.hpp"

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

class stompContext
{
public:
    stompContext(Robot* robot, const CollisionSpace* coll_space, const std::vector<int>& planner_joints, const std::vector<CollisionPoint>& collision_points );
    ~stompContext();

    bool initRun( API::Trajectory& T );
    void run();

    void setParallelRobots( const std::vector<Robot*>& robots );

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

    bool isRunning() const;

    void setIsRunning( int id );
    void setRobotPool( int id, const std::vector<Robot*>& robots );


private:

    std::vector<bool>                        m_is_thread_running;

    std::vector<Robot*>                      m_robots;
    std::vector<stompContext*>               m_stomps;

    std::vector<int>                         m_planner_joints;
    const CollisionSpace*                    m_coll_space;
    std::vector<CollisionPoint>              m_collision_points;
};

void srompRun_MultipleParallel();

void srompRun_OneParallel();

#endif // PARALLEL_STOMP_HPP

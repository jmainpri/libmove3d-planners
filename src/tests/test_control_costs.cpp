/*
 * Copyright (c) 2010-2015 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001).
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Sun 5 July 2015
 */


#include <iostream>

#include "API/project.hpp"
#include "feature_space/smoothness.hpp"
#include "planner/TrajectoryOptim/Lamp/lampTrajectory.hpp"
#include "planner/TrajectoryOptim/Stomp/covariant_trajectory_policy.hpp"

#include "planEnvironment.hpp"
#include "hri_costspace/gestures/HRICS_gest_parameters.hpp"
#include "hri_costspace/HRICS_parameters.hpp"

#include "utils/misc_functions.hpp"

#include <libmove3d/include/Planner-pkg.h>
#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Collision-pkg.h>

using std::cout;
using std::endl;

Move3D::Robot* robot = NULL;
std::vector<int> active_dofs;

Move3D::Trajectory create_random_trajectory()
{
    // Create random trajectory
    robot->setInitPos( *robot->shoot() );
    robot->setGoalPos( *robot->shoot() );

    int nb_way_points = 30;
    // Sample trajectories
    Move3D::LampSampler sampler( robot );
    std::vector<double> deriv_weights;
    deriv_weights.push_back(0);
    deriv_weights.push_back(1);
    deriv_weights.push_back(0);
    sampler.initialize( deriv_weights, nb_way_points );

    cout << "Create vector trajectory" << endl;
    Move3D::VectorTrajectory traj0( active_dofs.size(), nb_way_points, 1. );
    traj0.planning_group_ = sampler.planning_group_;
    traj0.trajectory_ = traj0.getSraightLineTrajectory();
    cout << " -- trajectory size : " << traj0.trajectory_.size() << endl;

    cout << "Sample trajectory" << endl;
    Eigen::VectorXd noise = sampler.sample( 0.03 );
    traj0.trajectory_ += noise;

    cout << "Get move3d trajectory" << endl;
    Move3D::Trajectory trajectory_0( robot );
    trajectory_0 = traj0.getMove3DTrajectory();
    trajectory_0.setUseTimeParameter( true );
    trajectory_0.setUseConstantTime( true );

    return trajectory_0;
}

double get_control_cost( const Move3D::Trajectory& trajectory,
                         const std::vector<int>& joints )
{
    // Intialize policy

    std::vector<double> derivative_costs;
    derivative_costs.push_back( 0 );
    derivative_costs.push_back( 1 );
    derivative_costs.push_back( 0 );

    double ridge_factor = 0;

    stomp_motion_planner::CovariantTrajectoryPolicy policy;
    policy.setPrintDebug( false );

    Move3D::ChompPlanningGroup*
    planning_group = new Move3D::ChompPlanningGroup( trajectory.getRobot(),
                                                     joints );

    int nb_dofs = planning_group->num_dofs_;
    int num_time_steps = trajectory.getNbOfViaPoints();

    policy.initialize( num_time_steps,
                       nb_dofs,
                       trajectory.getDuration(),
                       ridge_factor,
                       derivative_costs,
                       false,
                       planning_group);

    Eigen::VectorXd start = trajectory.getBegin()->getEigenVector(
                planning_group->getActiveDofs() );
    Eigen::VectorXd end = trajectory.getEnd()->getEigenVector(
                planning_group->getActiveDofs() );

    policy.setToMinControlCost( start, end );

    // Get parameter vectors

    std::vector<Eigen::MatrixXd> control_cost_matrices;

    std::vector<Eigen::VectorXd> parameters =
            std::vector<Eigen::VectorXd>( nb_dofs,
                                Eigen::VectorXd::Zero(num_time_steps) );
    std::vector<Eigen::VectorXd> noise=
            std::vector<Eigen::VectorXd>( nb_dofs,
                                Eigen::VectorXd::Zero(num_time_steps) );
    std::vector<Eigen::VectorXd> control_costs=
            std::vector<Eigen::VectorXd>( nb_dofs,
                                Eigen::VectorXd::Zero(num_time_steps) );
    double weight = 1.0;
    double dt = trajectory.getDeltaTime();

    Eigen::MatrixXd mat =
            trajectory.getEigenMatrix( planning_group->getActiveDofs() );


    if( nb_dofs != mat.rows() ) {
        cout << "ERROR nb of dofs ("
             << nb_dofs << " , " << mat.rows() << " )"
             << endl;
        exit(0);
    }
    if( num_time_steps != mat.cols() ) {
        cout << "ERROR time steps ("
             << num_time_steps << " , " << mat.cols() << " )"
             << endl;
        exit(0);
    }

    for ( int i=0; i<nb_dofs; ++i)
    {
        for ( int j=0; j<num_time_steps; ++j)
        {
            parameters[i][j] = mat(i,j);
        }
    }

    cout << "dt : " << dt << endl;
    // cout << "mat : " << mat << endl;

    policy.computeControlCosts( control_cost_matrices,
                                parameters,
                                noise,
                                weight,
                                control_costs,
                                dt );

    // Get total smoothness cost
    // current control costs are already multipied by
    // the cost weight
    double total_smoothness_cost = 0.;
    int num_dim = control_costs.size();

    for (int d=0; d<num_dim; ++d)
    {
//        cout << "control cost [" << d << "] : "
//             << control_costs[d].transpose()
//             << endl;

        total_smoothness_cost += control_costs[d].sum();
    }
    return total_smoothness_cost * dt;
}

Move3D::Trajectory create_sinusoidal_trajectory(
        Move3D::confPtr_t q_init_ptr,
        Move3D::confPtr_t q_goal_ptr, int nb_config )
{
    Move3D::Trajectory sinus_traj( robot );

    Eigen::VectorXd q_init = q_init_ptr->getEigenVector( active_dofs );
    Eigen::VectorXd q_goal = q_goal_ptr->getEigenVector( active_dofs );

    double alpha = .1;

    for( int i=0; i<nb_config; i++ )
    {
        double t = double(i) / double(nb_config);
        Eigen::VectorXd q = move3d_lerp( q_init, q_goal, t );

        for( int j=0; j<q.size(); j++ )
        {
            q[j] += alpha * std::sin( t * 2. * M_PI );
        }

        Move3D::confPtr_t q_new = robot->getNewConfig();
        q_new->setFromEigenVector( q, active_dofs );
        sinus_traj.push_back( q_new );
    }

    sinus_traj.setUseTimeParameter( true );
    sinus_traj.setUseConstantTime( true );

    return sinus_traj;
}

int main(int argc, char *argv[])
{
    if( !move3d_start_and_load_manipulator() )
    {
      return 0;
    }

    std::string  robot_name;
    robot_name = XYZ_ENV->robot[0]->name ;
    cout << "Robot name is : " << robot_name << endl;

    // Make c++ project interface
    Move3D::global_Project = new Move3D::Project(new Move3D::Scene(XYZ_ENV));
    Move3D::global_Project->getActiveScene()->setActiveRobot( robot_name );
    robot = Move3D::global_Project->getActiveScene()->getActiveRobot();

    // GET QT PARAMETERS (used to be in project)
    initPlannerParameters();
    initGestureParameters();
    initHricsParameters();

    cout << "get active joints for robot : "
         << robot->getName()
         << endl;

    std::vector<int> active_joints  = robot->getActiveJointsIds();
    active_dofs = robot->getActiveDoFsFromJoints( active_joints );

    Move3D::Trajectory trajectory_0;
    Move3D::Trajectory trajectory_1;
    Move3D::Trajectory trajectory_2;
    Move3D::Trajectory trajectory_3;
    Move3D::Trajectory trajectory_4;

    bool use_random_trajectories = false;

    int nb_points_0 = 30;
    int nb_points_1 = 100;
    int nb_points_2 = 500;
    int nb_points_3 = 800;
    int nb_points_4 = 1000;

    Move3D::confPtr_t q_init = robot->getInitPos();
    Move3D::confPtr_t q_goal = robot->getGoalPos();

    if( use_random_trajectories )
    {
        trajectory_0 = create_random_trajectory();

        trajectory_1 = trajectory_0;
        trajectory_2 = trajectory_0;
        trajectory_3 = trajectory_0;
        trajectory_4 = trajectory_0;

        trajectory_0.cutTrajInSmallLPSimple( nb_points_0, true );
        trajectory_1.cutTrajInSmallLPSimple( nb_points_1, true );
        trajectory_2.cutTrajInSmallLPSimple( nb_points_2, true );
        trajectory_3.cutTrajInSmallLPSimple( nb_points_3, true );
        trajectory_4.cutTrajInSmallLPSimple( nb_points_4, true );
    }
    else
    {
        robot->setInitPos( *robot->shoot() );
        robot->setGoalPos( *robot->shoot() );

        q_init = robot->getInitPos();
        q_goal = q_init->copy(); // robot->getGoalPos();

        trajectory_0 = create_sinusoidal_trajectory( q_init, q_goal, nb_points_0 );
        trajectory_1 = create_sinusoidal_trajectory( q_init, q_goal, nb_points_1 );
        trajectory_2 = create_sinusoidal_trajectory( q_init, q_goal, nb_points_2 );
        trajectory_3 = create_sinusoidal_trajectory( q_init, q_goal, nb_points_3 );
        trajectory_4 = create_sinusoidal_trajectory( q_init, q_goal, nb_points_4 );
    }

    double delta_0 = 1. / double(nb_points_0);
    double delta_1 = 1. / double(nb_points_1);
    double delta_2 = 1. / double(nb_points_2);
    double delta_3 = 1. / double(nb_points_3);
    double delta_4 = 1. / double(nb_points_4);

    trajectory_0.setDeltaTime( delta_0 );
    trajectory_1.setDeltaTime( delta_1 );
    trajectory_2.setDeltaTime( delta_2 );
    trajectory_3.setDeltaTime( delta_3 );
    trajectory_4.setDeltaTime( delta_4 );

    for( size_t i=0; i<active_dofs.size(); i++ )
        cout << "active dofs : "  << active_dofs[i] << endl;

    bool with_buffer(false);
    std::vector<Eigen::VectorXd> buffer; // diff_rule_length_-1 (6)

    if( with_buffer )
    {
        buffer.push_back( q_init->getEigenVector( active_dofs ) );
        buffer.push_back( q_init->getEigenVector( active_dofs ) );
        buffer.push_back( q_init->getEigenVector( active_dofs ) );
        buffer.push_back( q_init->getEigenVector( active_dofs ) );
        buffer.push_back( q_init->getEigenVector( active_dofs ) );
        buffer.push_back( q_init->getEigenVector( active_dofs ) );
    }

    // Test smoothness functions
    Move3D::VelocitySmoothness velocity;
    velocity.setActiveDoFs( active_dofs );
    if( with_buffer)
        velocity.setBuffer( buffer );

    cout << "vel 1 : " << velocity.getFeatureCount( trajectory_0 ) << endl;
    cout << "vel 2 : " << velocity.getFeatureCount( trajectory_1 ) << endl;
    cout << "vel 3 : " << velocity.getFeatureCount( trajectory_2 ) << endl;
    cout << "vel 4 : " << velocity.getFeatureCount( trajectory_3 ) << endl;
    cout << "vel 5 : " << velocity.getFeatureCount( trajectory_4 ) << endl;

    // Test smoothness functions
    Move3D::AccelerationSmoothness acceleration;
    acceleration.setActiveDoFs( active_dofs );
    if( with_buffer )
        acceleration.setBuffer( buffer );

    cout << "accel 1 : " << acceleration.getFeatureCount( trajectory_0 ) << endl;
    cout << "accel 2 : " << acceleration.getFeatureCount( trajectory_1 ) << endl;
    cout << "accel 3 : " << acceleration.getFeatureCount( trajectory_2 ) << endl;
    cout << "accel 4 : " << acceleration.getFeatureCount( trajectory_3 ) << endl;
    cout << "accel 5 : " << acceleration.getFeatureCount( trajectory_4 ) << endl;

    // Test smoothness functions
    Move3D::JerkSmoothness jerk;
    jerk.setActiveDoFs( active_dofs );
    if( with_buffer )
        jerk.setBuffer( buffer );

    cout << "jerk 1 : " << jerk.getFeatureCount( trajectory_0 ) << endl;
    cout << "jerk 2 : " << jerk.getFeatureCount( trajectory_1 ) << endl;
    cout << "jerk 3 : " << jerk.getFeatureCount( trajectory_2 ) << endl;
    cout << "jerk 4 : " << jerk.getFeatureCount( trajectory_3 ) << endl;
    cout << "jerk 5 : " << jerk.getFeatureCount( trajectory_4 ) << endl;





    std::string joint_name("J6"); // Last joint
    Move3D::Joint* joint = robot->getJoint( joint_name );
    for( size_t i=0;i<robot->getNumberOfJoints(); i++)
    {
        cout << robot->getJoint(i)->getName() << endl;
    }
    if( joint == NULL ) {
        cout << "no joint named : " << joint_name << endl;
        exit(0);
    }

    Move3D::TaskSmoothnessFeature task_smoothness( robot, joint );
    Eigen::VectorXd control_costs;

    cout << "task vel 1 : "
         << task_smoothness.getVelocity( trajectory_0, control_costs )
         << endl;
    cout << "task vel 2 : "
         << task_smoothness.getVelocity( trajectory_1, control_costs )
         << endl;
    cout << "task vel 3 : "
         << task_smoothness.getVelocity( trajectory_2, control_costs )
         << endl;
    cout << "task vel 4 : "
         << task_smoothness.getVelocity( trajectory_3, control_costs )
         << endl;
    cout << "task vel 5 : "
         << task_smoothness.getVelocity( trajectory_4, control_costs )
         << endl;


    cout << "task accel 1 : "
         << task_smoothness.getAcceleration( trajectory_0, control_costs )
         << endl;
    cout << "task accel 2 : "
         << task_smoothness.getAcceleration( trajectory_1, control_costs )
         << endl;
    cout << "task accel 3 : "
         << task_smoothness.getAcceleration( trajectory_2, control_costs )
         << endl;
    cout << "task accel 4 : "
         << task_smoothness.getAcceleration( trajectory_3, control_costs )
         << endl;
    cout << "task accel 5 : "
         << task_smoothness.getAcceleration( trajectory_4, control_costs )
         << endl;


    cout << "task jerk 1 : "
         << task_smoothness.getJerk( trajectory_0, control_costs )
         << endl;
    cout << "task jerk 2 : "
         << task_smoothness.getJerk( trajectory_1, control_costs )
         << endl;
    cout << "task jerk 3 : "
         << task_smoothness.getJerk( trajectory_2, control_costs )
         << endl;
    cout << "task jerk 4 : "
         << task_smoothness.getJerk( trajectory_3, control_costs )
         << endl;
    cout << "task jerk 5 : "
         << task_smoothness.getJerk( trajectory_4, control_costs )
         << endl;

    std::string folder = std::string(getenv("HOME_MOVE3D"))
            + "/../move3d-launch/launch_files/control_cost_profiles/";

    jerk.saveAbsValuesToFile( trajectory_0, folder );
    jerk.saveAbsValuesToFile( trajectory_1, folder );

    //    jerk.saveAbsValuesToFile( trajectory_2, folder, trajectory_2.getDeltaTime() );
    //    jerk.saveAbsValuesToFile( trajectory_4, folder, trajectory_4.getDeltaTime() );

    task_smoothness.saveAbsValuesToFileVelocity( trajectory_1, folder );
    task_smoothness.saveAbsValuesToFileAcceleration( trajectory_1, folder );
    // task_smoothness.saveAbsValuesToFileJerk( trajectory_1, folder );


    double control_cost_1 = get_control_cost( trajectory_0, active_joints );
    double control_cost_2 = get_control_cost( trajectory_1, active_joints );
    double control_cost_3 = get_control_cost( trajectory_2, active_joints );
    double control_cost_4 = get_control_cost( trajectory_3, active_joints );
//    double control_cost_5 = get_control_cost( trajectory_4, active_joints );

    cout << "control cost 1  : " << control_cost_1 << endl;
    cout << "control cost 2  : " << control_cost_2 << endl;
    cout << "control cost 3  : " << control_cost_3 << endl;
    cout << "control cost 4  : " << control_cost_4 << endl;
//    cout << "control cost 5  : " << control_cost_5 << endl;


    trajectory_0.saveToFile( folder + "test0.traj" );
    trajectory_1.saveToFile( folder + "test1.traj" );
    trajectory_2.saveToFile( folder + "test2.traj" );
    trajectory_3.saveToFile( folder + "test3.traj" );
    trajectory_4.saveToFile( folder + "test4.traj" );

    // Delete global project
    delete Move3D::global_Project;
    return 0;
}

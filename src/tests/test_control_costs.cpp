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
#include "API/libmove3d_api.hpp"
#include "feature_space/smoothness.hpp"
#include "planner/TrajectoryOptim/Lamp/lampTrajectory.hpp"
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

Move3D::Trajectory create_sinusoidal_trajectory( Move3D::confPtr_t q_init_ptr, Move3D::confPtr_t q_goal_ptr, int nb_config )
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
    std::string home, filename, robot_name;

    home =  getenv("HOME_MOVE3D");
    if ( "" == home )
    {
        std::cout << "Error : HOME_MOVE3D not define" << std::endl;
        return 0;
    }

    cout << "home : " << home << endl;

    // Load environment from file
    filename = "/../assets/Manipulator/manipulateur.p3d";
    p3d_col_set_mode(p3d_col_mode_none);
    p3d_read_desc((home+filename).c_str());
    robot_name = XYZ_ENV->robot[0]->name ;
    cout << "Robot name is : " << robot_name << endl;

    // Set collision checker
    p3d_col_set_mode(p3d_col_mode_pqp);
    p3d_col_start(p3d_col_mode_pqp);
    set_collision_by_object(TRUE);

    // Set c++ api function
    move3d_set_classic_libmove3d_api();

    // Make c++ project interface
    Move3D::global_Project = new Move3D::Project(new Move3D::Scene(XYZ_ENV));
    Move3D::global_Project->getActiveScene()->setActiveRobot( robot_name );
    robot = Move3D::global_Project->getActiveScene()->getActiveRobot();

    cout << "get active joints" << endl;

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

    for( int i=0; i<active_dofs.size(); i++ )
        cout << "active dofs : "  << active_dofs[i] << endl;

    bool with_buffer(true);
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

    std::string folder = std::string(getenv("HOME_MOVE3D")) + "/../move3d-launch/launch_files/control_cost_profiles/";
    acceleration.saveAbsValuesToFile( trajectory_0, folder, trajectory_0.getDeltaTime() );
    acceleration.saveAbsValuesToFile( trajectory_1, folder, trajectory_1.getDeltaTime( ));
//    velocity.saveAbsValuesToFile( trajectory_2, folder, trajectory_2.getDeltaTime( ));
//    velocity.saveAbsValuesToFile( trajectory_4, folder, trajectory_4.getDeltaTime() );

    trajectory_0.saveToFile( folder + "test0.traj" );
    trajectory_1.saveToFile( folder + "test1.traj" );
    trajectory_2.saveToFile( folder + "test2.traj" );
    trajectory_3.saveToFile( folder + "test3.traj" );
    trajectory_4.saveToFile( folder + "test4.traj" );

    // Delete global project
    delete Move3D::global_Project;
    return 0;
}

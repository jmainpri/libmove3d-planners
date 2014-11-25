/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
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
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#include "cost_computation.hpp"

#include "planEnvironment.hpp"
#include "cost_space.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "hri_costspace/HRICS_costspace.hpp"

using namespace stomp_motion_planner;
using namespace Move3D;

using std::cout;
using std::endl;

const double hack_tweek = 1;

// *******************************************************
// API FUNCTIONS
// *******************************************************
static std::vector< boost::function<int( Move3D::Robot* )> > Move3DGetNumberOfCollisionPoints;
static std::vector< boost::function<bool( Move3D::Robot*, int i, Eigen::MatrixXd&, std::vector< std::vector<Eigen::Vector3d> >& )> > Move3DGetConfigCollisionCost;

// *******************************************************
// SETTERS
// *******************************************************

void move3d_set_fct_get_nb_collision_points( boost::function<int(Move3D::Robot*)> fct, int id )
{
    if( id == int(Move3DGetNumberOfCollisionPoints.size()) )
    {
        Move3DGetNumberOfCollisionPoints.push_back( fct );
    }
    else if ( id < int(Move3DGetConfigCollisionCost.size()) ){
        Move3DGetNumberOfCollisionPoints[id] = fct;
    }
    else {
        cout << "ERROR SIZE in " << __PRETTY_FUNCTION__ << endl;
    }
}
void move3d_set_fct_get_config_collision_cost( boost::function<bool( Move3D::Robot* robot, int i, Eigen::MatrixXd&, std::vector< std::vector<Eigen::Vector3d> >& )> fct, int id )
{
    if( id == int(Move3DGetConfigCollisionCost.size()) ){
        Move3DGetConfigCollisionCost.push_back( fct );
    }
    else if( id < int(Move3DGetConfigCollisionCost.size()) ) {
        Move3DGetConfigCollisionCost[id] = fct;
    }
    else {
        cout << "ERROR SIZE in " << __PRETTY_FUNCTION__ << endl;
    }
}

// *******************************************************
// MAIN
// *******************************************************

costComputation::costComputation(Robot* robot,
                                 const CollisionSpace *collision_space,
                                 const ChompPlanningGroup* planning_group,
                                 std::vector< ChompCost > joint_costs,
                                 ChompTrajectory group_trajectory,
                                 double obstacle_weight,
                                 bool use_costspace,
                                 Move3D::confPtr_t q_source,
                                 Move3D::confPtr_t q_target,
                                 bool use_external_collision_space,
                                 int collision_space_id,
                                 const stomp_motion_planner::StompParameters* stomp_parameters)
{
    robot_model_ = robot;

    cout << "new cost computation with robot : " << robot_model_->getName() << " collision id : " << collision_space_id << endl;

    stomp_parameters_ = stomp_parameters;

    source_ = q_source;
    target_ = q_target;

    // Collision space
    move3d_collision_space_ = collision_space;
    use_external_collision_space_ = use_external_collision_space;
    collision_space_id_ = collision_space_id;

    planning_group_ = planning_group;
    joint_costs_ = joint_costs;

    group_trajectory_ = group_trajectory;

    obstacle_weight_ = obstacle_weight;
    use_costspace_ = use_costspace;
    hack_tweek_ = 1.0;

    iteration_ = 0;

    num_vars_free_ = group_trajectory_.getNumFreePoints();
    num_vars_all_ = group_trajectory_.getNumPoints();
    num_joints_ = group_trajectory_.getNumJoints();
    free_vars_start_ = group_trajectory_.getStartIndex();
    free_vars_end_ = group_trajectory_.getEndIndex();
    num_collision_points_ = planning_group_->collision_points_.size();

    joint_axis_eigen_.resize(num_vars_all_);
    joint_pos_eigen_.resize(num_vars_all_);
    collision_point_pos_eigen_.resize(num_vars_all_);
    collision_point_vel_eigen_.resize(num_vars_all_);
    collision_point_acc_eigen_.resize(num_vars_all_);

    segment_frames_.resize( num_vars_all_ );
    state_is_in_collision_.resize( num_vars_all_ );

    general_cost_potential_= Eigen::VectorXd::Zero(num_vars_all_);
    dt_ = Eigen::VectorXd::Zero(num_vars_all_);

    for(int i=0; i<num_vars_all_;i++)
    {
        segment_frames_[i].resize(planning_group_->num_dofs_);
        joint_pos_eigen_[i].resize(planning_group_->num_dofs_);
    }

    if (num_collision_points_ > 0)
    {
        collision_point_potential_ = Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
        collision_point_vel_mag_ = Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
        collision_point_potential_gradient_.resize(num_vars_all_, std::vector<Eigen::Vector3d>(num_collision_points_));

        for(int i=0; i<num_vars_all_;i++)
        {
            joint_axis_eigen_[i].resize(num_collision_points_);
            joint_pos_eigen_[i].resize(num_collision_points_);
            collision_point_pos_eigen_[i].resize(num_collision_points_);
            collision_point_vel_eigen_[i].resize(num_collision_points_);
            collision_point_acc_eigen_[i].resize(num_collision_points_);
        }
    }

    // Move the end configuration in the trajectory
    id_fixed_ = 2;
    allow_end_configuration_motion_ = PlanEnv->getBool(PlanParam::trajStompMoveEndConfig);
    if( allow_end_configuration_motion_ )
    {
        id_fixed_ = 1;
    }
    group_trajectory_.setFixedId( id_fixed_ );
}

costComputation::~costComputation()
{
    cout << "destroy cost computation" << endl;
}

bool costComputation::handleJointLimits(  ChompTrajectory& group_traj  )
{
    bool succes_joint_limits = true;
    bool joint_limits_violation = 0;

    for (int joint=0; joint<num_joints_; joint++)
    {
        if (!planning_group_->chomp_dofs_[joint].has_joint_limits_)
            continue;

        // Added by jim for pr2 free flyer
        if( planning_group_->robot_->getName() == "PR2_ROBOT" )
        {
            int index = planning_group_->chomp_dofs_[joint].move3d_dof_index_;

            if( index == 8 || index == 9 || index == 10 ) {
                group_trajectory_.getFreeJointTrajectoryBlock(joint) = Eigen::VectorXd::Zero(num_vars_free_);
                continue;
            }
        }

        double joint_max = planning_group_->chomp_dofs_[joint].joint_limit_max_;
        double joint_min = planning_group_->chomp_dofs_[joint].joint_limit_min_;

        // cout << "Joint max : " << joint_max << endl;
        // cout << "Joint min : " << joint_min << endl;
        int count = 0;
        bool violation = false;
        int count_violation = false;

        do
        {
            double max_abs_violation =  1e-6;
            double max_violation = 0.0;
            int max_violation_index = 0;
            violation = false;
            count_violation = false;
            //            bool violate_max = false;
            //            bool violate_min = false;
            double violate_max_with = 0;
            double violate_min_with = 0;

            for (int i=free_vars_start_; i<=free_vars_end_; i++)
            {
                double amount = 0.0;
                double absolute_amount = 0.0;
                if ( group_trajectory_(i, joint) > joint_max )
                {
                    amount = joint_max - group_trajectory_(i, joint);
                    absolute_amount = fabs(amount);
                    //                    violate_max = true;
                    if( absolute_amount > violate_max_with )
                    {
                        violate_max_with = absolute_amount;
                    }
                }
                else if ( group_trajectory_(i, joint) < joint_min )
                {
                    amount = joint_min - group_trajectory_(i, joint);
                    absolute_amount = fabs(amount);
                    //                    violate_min = true;
                    if( absolute_amount > violate_min_with )
                    {
                        violate_min_with = absolute_amount;
                    }
                }
                if (absolute_amount > max_abs_violation)
                {
                    max_abs_violation = absolute_amount;
                    max_violation = amount;
                    max_violation_index = i;
                    violation = true;

                    // cout << "Violation (" << absolute_amount ;
                    // cout << " , " << max_abs_violation << ")" << endl;

                    if ( i != free_vars_start_ && i != free_vars_end_ ) {
                        count_violation = true;
                    }
                }
            }

            if (violation)
            {
                // Only count joint limits violation for
                // interpolated configurations
                if( count_violation )
                {
                    joint_limits_violation++;
                }
                // cout << "Violation of Limits (joint) : " <<  joint << endl;
                int free_var_index = max_violation_index - free_vars_start_;
                double multiplier = max_violation / joint_costs_[joint].getQuadraticCostInverse()(free_var_index,free_var_index);

                group_trajectory_.getFreeJointTrajectoryBlock(joint) +=
                        (( multiplier * joint_costs_[joint].getQuadraticCostInverse().col(free_var_index)).segment(1,num_vars_free_-id_fixed_));
                //                double offset = ( joint_max + joint_min )  / 2 ;

                //                cout << "multiplier : " << multiplier << endl;
                //                cout << "joint limit max : " << joint_max << endl;
                //                cout << "joint limit min : " << joint_min << endl;
                //                cout << "offset : " << offset << endl;
                //                cout << group_trajectory_.getFreeJointTrajectoryBlock(joint).transpose() << endl;

                //                group_trajectory_.getFreeJointTrajectoryBlock(joint) = ( group_trajectory_.getFreeJointTrajectoryBlock(joint).array() - offset )*multiplier + ( offset );

                //                cout << group_trajectory_.getFreeJointTrajectoryBlock(joint).transpose() << endl;
            }
            if ( ++count > 10 )
            {
                succes_joint_limits = false;
                //cout << "group_trajectory_(i, joint) = " << endl << group_trajectory_.getFreeJointTrajectoryBlock(joint) << endl;
                break;
            }
        }
        while(violation);

        //        if( violation || !succes_joint_limits )
        //        {
        //            cout << "Violation of joint limits (joint) = " << joint << endl;
        //        }
    }

    //    cout << "succes_joint_limits : " << succes_joint_limits << endl;

    //    exit(1);

    return succes_joint_limits;
}

double costComputation::getCollisionSpaceCost( const Configuration& q )
{
    const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

    bool quiet = true;

    Configuration q_tmp = q;

    // Get the configuration dof values in the joint array
    Eigen::VectorXd joint_array( planning_group_->num_dofs_ );

    for(int j=0; j<planning_group_->num_dofs_;j++)
    {
        joint_array[j] = q_tmp[ joints[j].move3d_dof_index_ ];
    }

    getFrames( free_vars_start_, joint_array, q_tmp );

    double state_collision_cost=0.0;
    bool colliding = false;

    // Calculate the 3d position of every collision point
    for (int j=0; j<num_collision_points_; j++)
    {
        // OLD (free_vars_start_ is the first point, does not matter when called from outside)
        colliding |= getCollisionPointObstacleCost( free_vars_start_, j, collision_point_potential_( free_vars_start_, j ), collision_point_pos_eigen_[free_vars_start_][j] );
        state_collision_cost += collision_point_potential_( free_vars_start_, j ); // * collision_point_vel_mag_( free_vars_start_, j );

        if(!quiet)
            cout << "collision_point_potential_( " << free_vars_start_ << " , " <<  j << " ) : " << collision_point_potential_( free_vars_start_, j ) << endl;
    }

    if( colliding && !quiet ){
        cout << "config is in collision" << endl;
    }

    return state_collision_cost;
}

void costComputation::getFrames( int segment, const Eigen::VectorXd& joint_array, Configuration& q )
{
    const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

    // Set the configuration to the joint array value
    for(int j=0; j<planning_group_->num_dofs_;j++)
    {
        int dof = joints[j].move3d_dof_index_;

        if ( !std::isnan(joint_array[j]) )
        {
            q[dof]= joint_array[j];
        }
        else {
            cout << "q[" << dof << "] is nan" << endl;
            q[dof]= 0;
        }
    }

    robot_model_->setAndUpdate( q );
    // g3d_draw_allwin_active();

    // Get the collision point position
    for(int j=0; j<planning_group_->num_dofs_;j++)
    {
        segment_frames_[segment][j] = joints[j].move3d_joint_->getMatrixPos();

        //cout << "joints[" << j<< "].move3d_joint_ : " << joints[j].move3d_joint_->getName() << endl;
        //        eigenTransformToStdVector( t, segment_frames_[segment][j] );

        joint_pos_eigen_[segment][j] = segment_frames_[segment][j].translation();

//        cout << 'move3d_collision_space_ : ' << move3d_collision_space_ << endl;

        if( move3d_collision_space_ )
        {
            joint_axis_eigen_[segment][j](0) = segment_frames_[segment][j](0,2);
            joint_axis_eigen_[segment][j](1) = segment_frames_[segment][j](1,2);
            joint_axis_eigen_[segment][j](2) = segment_frames_[segment][j](2,2);
        }
    }

    /**
    q = *robot_model_->getCurrentPos();

    const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

    // Set the configuration to the joint array value
    for(int j=0; j<planning_group_->num_dofs_;j++)
    {
        int dof = joints[j].move3d_dof_index_;

        if ( !std::isnan(joint_array[j]) )
        {
            q[dof]= joint_array[j];
        }
        else {
            cout << "q[" << dof << "] is nan" << endl;
            q[dof]= 0;
        }
    }

    robot_model_->setAndUpdate( q );

    if( collision_space_ )
    {
        // Get the collision point position
        for(int j=0; j<planning_group_->num_dofs_;j++)
        {
            Eigen::Transform3d t = robot_model_->getJoint( joints[j].move3d_joint_->getId() )->getMatrixPos();

//            std::vector<double> vect;
//            eigenTransformToStdVector( t, vect );

            segment_frames_[segment][j]  = t;
            joint_pos_eigen_[segment][j] = t.translation();

            joint_axis_eigen_[segment][j](0) = t(0,2);
            joint_axis_eigen_[segment][j](1) = t(1,2);
            joint_axis_eigen_[segment][j](2) = t(2,2);
        }
    }
    **/
}

bool costComputation::getCollisionPointObstacleCost( int segment, int coll_point, double& collion_point_potential, Eigen::Vector3d& pos )
{
    bool colliding = false;



    if( move3d_collision_space_ /*&& (use_costspace_==false)*/ )
    {
        int i = segment;
        int j = coll_point;
        double distance;



        planning_group_->collision_points_[j].getTransformedPosition( segment_frames_[i], pos );



        // To fix in collision space
        // The joint 1 is allways colliding
        colliding = move3d_collision_space_->getCollisionPointPotentialGradient( planning_group_->collision_points_[j],
                                                                                 pos,
                                                                                 distance,
                                                                                 collion_point_potential,
                                                                                 collision_point_potential_gradient_[i][j] );

//        cout << "is colliding, robot : " << robot_model_->getName() << endl;
        // cout << "distance( " << i << ", " << j << " ) : " << distance << endl;
        //cout << "collision_point_potential_(" << i << ", " << j << " ) : " << collision_point_potential_(i,j) << endl;
        // if( collision_point_potential_(i,j) != 0.0 )
        // {
        //    cout << "collision_point_potential_(" << i << ", " << j << " ) : " << collision_point_potential_(i,j) << endl;
        // }
    }

    return colliding;
}

bool costComputation::getConfigObstacleCost( Move3D::Robot* robot, int i, Eigen::MatrixXd& collision_point_potential, std::vector< std::vector<Eigen::Vector3d> >& collision_point_pos )
{
//    cout << "get collision cost : " << robot->getName() << endl;
    bool in_collision = false;

    // calculate the position of every collision point
    for (int j=0; j<num_collision_points_; j++)
    {
        bool colliding = getCollisionPointObstacleCost( i, j, collision_point_potential(i,j), collision_point_pos[i][j] );

        if ( colliding )
        {
            // This is the function that discards joints too close to the base
            if( planning_group_->collision_points_[j].getSegmentNumber() > 1 )
            {
                in_collision = true;
            }
        }
    }

    return in_collision;
    /**
    bool colliding = false;

    if( collision_space_  ) // && (use_costspace_==false)
    {
        int i= segment; int j= coll_point; double distance;

        planning_group_->collision_points_[j].getTransformedPosition( segment_frames_[i], collision_point_pos_eigen_[i][j] );

        // To fix in collision space
        // The joint 1 is allways colliding
        colliding = collision_space_->getCollisionPointPotentialGradient( planning_group_->collision_points_[j],
                                                                          collision_point_pos_eigen_[i][j],
                                                                          distance,
                                                                          collision_point_potential_(i,j),
                                                                          collision_point_potential_gradient_[i][j]);
    }

    return colliding;
    **/
}

int costComputation::getNumberOfCollisionPoints(Move3D::Robot* R)
{
    return planning_group_->collision_points_.size();
}

bool costComputation::performForwardKinematics( const ChompTrajectory& group_traj, bool is_rollout )
{
    double invTime = 1.0 / group_traj.getDiscretization();
    double invTimeSq = invTime*invTime;

    // calculate the forward kinematics for the fixed states only in the first iteration:
    int start = free_vars_start_;
    int end = free_vars_end_;
//    if ( iteration_==0 )
//    {
//        start = 0;
//        end = num_vars_all_-1;
//    }

    is_collision_free_ = true;

    Eigen::VectorXd joint_array;

    Configuration q( *source_ );
    Configuration q_tmp( *robot_model_->getCurrentPos() );
    Configuration q_prev( robot_model_ );



//    if( PlanEnv->getBool(PlanParam::useLegibleCost) )
//    {
//        HRICS_activeLegi->setTrajectory( group_traj.getTrajectory() );
//    }


    // Move3D::Trajectory current_traj( robot_model_ );

//    int nb_features = 0;
//    StackedFeatures* fct = dynamic_cast<StackedFeatures*>( global_activeFeatureFunction );
//    if( ( fct != NULL ) && ( fct->getFeatureFunction("Distance") != NULL ) )
//        nb_features = fct->getFeatureFunction("Distance")->getNumberOfFeatures();
//    Eigen::VectorXd phi (Eigen::VectorXd::Zero(nb_features) );

//    cout << "Compute for robot : " << robot_model_->getName() << " collision space : " << collision_space_id_ << endl;
    int k = 0;
    int way_point_ratio = -1;

    // for each point in the trajectory
    for (int i=start; i<=end; ++i)
    {
        state_is_in_collision_[i] = false;

        group_traj.getTrajectoryPointP3d( group_traj.getFullTrajectoryIndex(i), joint_array );

        this->getFrames( i, joint_array, q ); // Perform FK

        // current_traj.push_back( confPtr_t(new Configuration(q)) );
        if( false && is_rollout && ( i%way_point_ratio != 0 ) && ( i != start ) && (i != end ) )
        {
            cout << "no fk for : " << i << endl;
            general_cost_potential_[i] = general_cost_potential_[i-1];
            state_is_in_collision_[i]  = state_is_in_collision_[i-1];

            if( move3d_collision_space_ || use_external_collision_space_ )
                for( int j=0; j<num_collision_points_;j++)
                {
                    collision_point_potential_(i,j) = collision_point_potential_(i-1,j);
                    collision_point_pos_eigen_[i][j] = collision_point_pos_eigen_[i-1][j];
                }
        }
        else
        {
            if( use_costspace_ )
            {
                general_cost_potential_[i] = global_costSpace->cost( q ); // no set and update
            }
            else if( PlanEnv->getBool(PlanParam::useLegibleCost) )
            {
                general_cost_potential_[i] = HRICS_activeLegi->legibilityCost(i);
            }

            if( move3d_collision_space_ || use_external_collision_space_ )
            {
                state_is_in_collision_[i] = Move3DGetConfigCollisionCost[ collision_space_id_ ]( robot_model_, i, collision_point_potential_, collision_point_pos_eigen_ );
            }
            else if( ( PlanEnv->getBool(PlanParam::useLegibleCost) || use_costspace_ ) && (move3d_collision_space_==NULL) )
            {
                state_is_in_collision_[i] = false;
            }

            if ( state_is_in_collision_[i] )
            {
                is_collision_free_ = false;
            }
        }

        dt_[i] = group_traj.getUseTime() || ( i == start ) ? group_traj.getDiscretization() : q.dist( q_prev );

//        cout << "dt_[i] : " << dt_[i] << endl;

//        if (( fct != NULL ) && ( fct->getFeatureFunction("Distance") != NULL ) )
//            phi += ( fct->getFeatureFunction( "Distance" )->getFeatures( q ) * dt_[i] );

        q_prev = q; // store configuration
        q = *source_; // Make sure dofs are set to source

        k++;
    }

//    cout << "is_collision_free_ : " << is_collision_free_  << endl;

//    cout << "phi : " << std::scientific << phi.transpose() << endl;
//    cout << "dt_[start] : " << std::scientific << dt_[start] << endl;
//    cout << "nb of points : " << k << endl;

//    cout << "dt : " << dt_.transpose() << endl;
//    cout << "dt size : " << dt_.size() << endl;

    // Set true for spetial IOC cost
    //    if( /*!HriEnv->getBool(HricsParam::ioc_use_stomp_spetial_cost)*/ true )
    //    {
    //        general_cost_potential_ = global_PlanarCostFct->getStompCost( current_traj );
    //    }

    //    if(is_collision_free_)
    //    {
    //        // for each point in the trajectory
    //        for (int i=free_vars_start_; i<free_vars_end_; i++)
    //        {
    //            LocalPath* LP = new LocalPath(getConfigurationOnGroupTraj(i),getConfigurationOnGroupTraj(i+1));
    //            if(!LP->isValid())
    //            {
    //                is_collision_free_=false;
    //                break;
    //            }
    //        }
    //    }

    // now, get the vel and acc for each collision point (using finite differencing)
    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        for (int j=0; j<num_collision_points_; j++)
        {
            collision_point_vel_eigen_[i][j] = Eigen::Vector3d::Zero();
            collision_point_acc_eigen_[i][j] = Eigen::Vector3d::Zero();

            for (int k=-DIFF_RULE_LENGTH/2; k<=DIFF_RULE_LENGTH/2; k++)
            {
                collision_point_vel_eigen_[i][j] += (invTime * DIFF_RULES[0][k+DIFF_RULE_LENGTH/2]) * collision_point_pos_eigen_[i+k][j];
                collision_point_acc_eigen_[i][j] += (invTimeSq * DIFF_RULES[1][k+DIFF_RULE_LENGTH/2]) * collision_point_pos_eigen_[i+k][j];
            }
            // get the norm of the velocity:
            collision_point_vel_mag_(i,j) = collision_point_vel_eigen_[i][j].norm();

            //      cout << "collision_point_pos_eigen_(" << i << " , "  << j << ") = " << endl << collision_point_pos_eigen_[i][j] << endl;
            //      cout << "collision_point_potential_(" << i << " , "  << j << ") = " << endl << collision_point_potential_[i][j] << endl;
            //      cout << "collision_point_potential_gradient_(" << i << " , "  << j << ") = " << endl << collision_point_potential_gradient_[i][j] << endl;
            //      cout << "collision_point_vel_mag_(" << i << " , "  << j << ") = " << endl << collision_point_vel_mag_[i][j] << endl;
            //      cout << "collision_point_vel_eigen_(" << i << " , "  << j << ") = " << endl << collision_point_vel_eigen_[i][j] << endl;
            //      cout << "collision_point_acc_eigen_(" << i << " , "  << j << ") = " << endl << collision_point_acc_eigen_[i][j] << endl;
        }
    }

    //  if (is_collision_free_)
    //  {
    //    cout << "-----------------------" << endl;
    //    cout << "Collision Free Traj" << endl;
    //    cout << "-----------------------" << endl;
    //  }
    //  else {
    //    cout << "------------------------------------" << endl;
    //    for (int i=start; i<=end; ++i)
    //    {
    //      if (state_is_in_collision_[i])
    //      {
    //        cout << "state[" << i << "] in collision" << endl;
    //      }
    //    }
    //    cout << "------------------------------------" << endl;
    //  }

    robot_model_->setAndUpdate( q_tmp );

    return is_collision_free_;

    /*
    double invTime = 1.0 / group_traj.getDiscretization();
    double invTimeSq = invTime*invTime;

    // calculate the forward kinematics for the fixed states only in the first iteration:
    int start = free_vars_start_;
    int end = free_vars_end_;
    if ( iteration_==0 )
    {
        start = 0;
        end = num_vars_all_-1;
    }

    is_collision_free_ = true;

    Eigen::VectorXd joint_array;

    Configuration q( robot_model_ );

    // for each point in the trajectory
    for (int i=start; i<=end; ++i)
    {
        group_traj.getTrajectoryPointP3d( group_traj.getFullTrajectoryIndex(i), joint_array );

        getFrames( i, joint_array, q );

        state_is_in_collision_[i] = false;

        if( use_costspace_ )
        {
            general_cost_potential_[i] = global_costSpace->cost(q);
            // cout << "compute cost : " << general_cost_potential_[i] << endl;
        }

        if( collision_space_ )
        {
            // calculate the position of every collision point:
            for (int j=0; j<num_collision_points_; j++)
            {
                bool colliding = getConfigObstacleCost( i, j, q );

                if ( colliding )
                {
                    // This is the function that discards joints too close to the base
                    if( planning_group_->collision_points_[j].getSegmentNumber() > 1 )
                    {
                        state_is_in_collision_[i] = true;
                    }
                }
            }
        }

        if( use_costspace_ && (collision_space_==NULL) )
        {
            state_is_in_collision_[i] = false;
        }

        if ( state_is_in_collision_[i] )
        {
            is_collision_free_ = false;
        }
    }

    if( collision_space_ )
    {
        // now, get the vel and acc for each collision point (using finite differencing)
        for (int i=free_vars_start_; i<=free_vars_end_; i++)
        {
            for (int j=0; j<num_collision_points_; j++)
            {
                collision_point_vel_eigen_[i][j] = Eigen::Vector3d::Zero();
                collision_point_acc_eigen_[i][j] = Eigen::Vector3d::Zero();

                for (int k=-DIFF_RULE_LENGTH/2; k<=DIFF_RULE_LENGTH/2; k++)
                {
                    collision_point_vel_eigen_[i][j] += (invTime * DIFF_RULES[0][k+DIFF_RULE_LENGTH/2]) *
                            collision_point_pos_eigen_[i+k][j];
                    collision_point_acc_eigen_[i][j] += (invTimeSq * DIFF_RULES[1][k+DIFF_RULE_LENGTH/2]) *
                            collision_point_pos_eigen_[i+k][j];
                }
                // get the norm of the velocity:
                collision_point_vel_mag_(i,j) = collision_point_vel_eigen_[i][j].norm();
            }
        }
    }

    return is_collision_free_;
    */
}


double costComputation::resampleParameters(std::vector<Eigen::VectorXd>& parameters)
{
    const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;
    Move3D::Smoothing traj(planning_group_->robot_);
    Eigen::MatrixXd parameters_tmp(num_joints_,num_vars_free_);

    for ( int i=0; i<num_joints_; ++i) {
        parameters_tmp.row(i) = parameters[i].transpose();
    }

    traj.clear();
    for (int j=0; j<num_vars_free_; ++j)
    {
        // Set the configuration from the stored source and target
        confPtr_t q;

        if( j==0 ) {
            q = source_;
        }
        else if( j == num_vars_free_-1 ) {
            q = target_;
        }
        else
        {
            //q = passive_dofs_[j];
            q = planning_group_->robot_->getCurrentPos();

            for ( int i=0; i<planning_group_->num_dofs_; ++i)
            {
                (*q)[joints[i].move3d_dof_index_] = parameters_tmp(i,j);
            }
        }

        traj.push_back(q);
    }

    PlanEnv->setBool(PlanParam::trajStompComputeColl, false );
    traj.runShortCut(15);
    PlanEnv->setBool(PlanParam::trajStompComputeColl, true );

    // calculate the forward kinematics for the fixed states only in the first iteration:
    double step = traj.getParamMax() / (num_vars_free_-1);
    double param = step;
    for (int j=0; j<num_vars_free_; ++j)
    {
        confPtr_t q = traj.configAtParam(param);

        for ( int i=0; i<planning_group_->num_dofs_; ++i )
        {
            parameters_tmp(i,j) = (*q)[joints[i].move3d_dof_index_];
        }
        param += step;
    }

    for ( int i=0; i<num_joints_; ++i) {
        parameters[i].transpose() = parameters_tmp.row(i);
    }

    return traj.cost();
}

bool costComputation::getCost(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, const int iteration_number, bool joint_limits, bool resample, bool is_rollout )
{
    //cout << "size : " << parameters.size() << " , " << parameters[0].size() <<  endl;

    // copy the parameters into group_trajectory_:

    for (int d=0; d<num_joints_; ++d) {
        group_trajectory_.getFreeJointTrajectoryBlock(d) = parameters[d].segment(1,parameters[d].size()-id_fixed_); // The use of segment prevents the drift
    }
    //cout << "group_trajectory_ = " << endl << group_trajectory_.getTrajectory() << endl;

    // respect joint limits:

    succeded_joint_limits_ = handleJointLimits( group_trajectory_ );

    // Fix joint limits
    //    if( !succeded_joint_limits_ )
    //    {
    //        for (int d=0; d<num_joints_; ++d) {
    //            group_trajectory_.getFreeJointTrajectoryBlock(d) = parameters[d];
    //        }
    //    }
    //cout << "Violation number : " << joint_limits_violation_ << endl;
    //succeded_joint_limits_ = true;

    // copy the group_trajectory_ parameters:
    if( joint_limits )
    {
        //cout << "return with joint limits" << endl;
        for (int d=0; d<num_joints_; ++d) {
            parameters[d].segment(1,parameters[d].size()-id_fixed_) = group_trajectory_.getFreeJointTrajectoryBlock(d);
        }
    }

    if( resample )
    {
        // Resample
        last_move3d_cost_ = resampleParameters( parameters );

        for (int d=0; d<num_joints_; ++d) {
            group_trajectory_.getFreeJointTrajectoryBlock(d) = parameters[d].segment(1,parameters[d].size()-id_fixed_); // The use of segment prevents the drift
        }
    }

    // copy to full traj TODO get full trajectory
    // updateFullTrajectory()

    // do forward kinematics:
    bool trajectory_collision_free = performForwardKinematics( group_trajectory_, is_rollout);
    bool last_trajectory_constraints_satisfied = true;

    // Special case for not handling joint limits
    if( !succeded_joint_limits_ )
    {
        is_collision_free_ = false;
        trajectory_collision_free = false; // collision free but not valid
    }

    double cost;

    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        double state_collision_cost = 0.0;
        double state_general_cost = 0.0;
        double cumulative = 0.0;

        if( move3d_collision_space_  || use_external_collision_space_ )
        {
            for (int j=0; j<num_collision_points_; j++)
            {
                cumulative += collision_point_potential_(i,j) * collision_point_vel_mag_(i,j); // Becarful ... with velocity breaks of taken off
                state_collision_cost += cumulative;
            }
        }
        if( use_costspace_ || PlanEnv->getBool(PlanParam::useLegibleCost) )
        {
            state_general_cost = ( pow( general_cost_potential_(i) , hack_tweek ) /* ( q_f - q_i ).norm() */) ;
        }

        cost = 0.0;
        cost += stomp_parameters_->getObstacleCostWeight() * state_collision_cost;
        cost += stomp_parameters_->getGeneralCostWeight() * state_general_cost;
        cost *= dt_[i];

        costs(i-free_vars_start_) = cost;

        //cout << "state_collision_cost : " << state_collision_cost << endl;
    }

    //cout << "StompOptimizer::execute::cost => " << costs.sum() << endl;
    double last_trajectory_cost = costs.sum();
    //last_trajectory_constraints_satisfied = (constraint_cost < 1e-6);
    return trajectory_collision_free;



    /**
//    cout << __PRETTY_FUNCTION__ << endl;
    iteration_ = iteration;

    // copy the parameters into group_trajectory_:
    for (int d=0; d<num_joints_; ++d) {
        group_trajectory_.getFreeJointTrajectoryBlock(d) = parameters[d].segment(1,num_vars_free_-id_fixed_);
    }
    //    group_trajectory_.print();

    // respect joint limits:
    succeded_joint_limits_ = handleJointLimits( group_trajectory_ );
    //succeded_joint_limits_ = true;

    // do forward kinematics:
    performForwardKinematics( group_trajectory_ );

    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        double state_collision_cost = 0.0;
        double state_general_cost = 0.0;
        double cumulative = 0.0;

        if( collision_space_ )
        {
            for (int j=0; j<num_collision_points_; j++)
            {
                cumulative += collision_point_potential_(i,j) * collision_point_vel_mag_(i,j);
                state_collision_cost += cumulative;
            }
        }
        if( use_costspace_ )
        {
            state_general_cost = ( pow( general_cost_potential_(i) , hack_tweek_ ) ) ; // ( q_f - q_i ).norm()
        }

        costs(i-free_vars_start_) = obstacle_weight_ * ( state_collision_cost + state_general_cost );
    }

    // cout << costs.transpose() << endl;
    // cout << "return with joint_limits" << endl;

    // copy the parameters into group_trajectory_:
    for (int d=0; d<num_joints_; ++d) {
        parameters[d].segment(1,num_vars_free_-id_fixed_) = group_trajectory_.getFreeJointTrajectoryBlock(d);
    }

    return true;

        **/
}

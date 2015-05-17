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
#include "lampComputeCost.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "planner/TrajectoryOptim/jointlimits.hpp"
#include "hri_costspace/HRICS_costspace.hpp"
#include "feature_space/features.hpp"
#include "feature_space/smoothness.hpp"
#include "API/Device/generalik.hpp"


using namespace stomp_motion_planner;
using namespace Move3D;

using std::cout;
using std::endl;

const double hack_tweek = 1;


// *******************************************************
// MAIN
// *******************************************************

LampCostComputation::LampCostComputation(Robot* robot,
                                 const CollisionSpace *collision_space,
                                 const ChompPlanningGroup* planning_group,
                                 Move3D::LampTrajectory group_trajectory,
                                 double obstacle_weight,
                                 bool use_costspace,
                                 Move3D::confPtr_t q_source,
                                 Move3D::confPtr_t q_target,
                                 bool use_external_collision_space,
                                 int collision_space_id,
                                 const stomp_motion_planner::StompParameters* stomp_parameters,
                                 stomp_motion_planner::Policy* policy,
                                 const Eigen::MatrixXd& dynamics )
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

    group_trajectory_ = group_trajectory;

    obstacle_weight_ = obstacle_weight;
    use_costspace_ = use_costspace;
    hack_tweek_ = 1.0;

    iteration_ = 0;

    num_vars_free_ = group_trajectory_.getNumFreePoints();
    num_vars_all_ = group_trajectory_.getNumFreePoints() + 2 * DIFF_RULE_LENGTH;
    num_joints_ = group_trajectory_.getNumJoints();

    free_vars_start_2_ = DIFF_RULE_LENGTH - 1;
    free_vars_end_2_ = (num_vars_all_ - 1) - (DIFF_RULE_LENGTH - 1);

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
        joint_axis_eigen_[i].resize(planning_group_->num_dofs_);
    }

    if (num_collision_points_ > 0)
    {
        cout << "num_collision_points_ : " << num_collision_points_ << endl;
        cout << "num_vars_all_ : " << num_vars_all_ << endl;

        collision_point_potential_ = Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
        collision_point_vel_mag_ = Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
        collision_point_potential_gradient_.resize(num_vars_all_, std::vector<Eigen::Vector3d>(num_collision_points_));

        for(int i=0; i<num_vars_all_;i++)
        {
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

    id_fixed_ = 0;
//    group_trajectory_.setFixedId( id_fixed_ );

    // Initialize control costs
    multiple_smoothness_ = true;

    if( multiple_smoothness_ )
    {
        Move3D::StackedFeatures* fct = dynamic_cast<Move3D::StackedFeatures*>( global_activeFeatureFunction );

        if( fct != NULL &&
                fct->getFeatureFunction("SmoothnessAll") != NULL &&
                fct->getFeatureFunction("SmoothnessAll")->is_active_ )
        {
            control_cost_weights_ = fct->getFeatureFunction("SmoothnessAll")->getWeights();
            //                cout << "control_cost_weights_ : " << control_cost_weights_.transpose() << endl;
        }
        else{
            multiple_smoothness_ = false;
        }
    }

    ///--------------------------------------------------------------------------

    joint_costs_.clear();
    double max_cost_scale = 0.0;

    for (int i=0; i<planning_group_->chomp_dofs_.size(); i++)
    {
        double joint_cost = 1.0;
        std::vector<double> derivative_costs(3);
        derivative_costs[0] = joint_cost*stomp_parameters_->getSmoothnessCostVelocity();
        derivative_costs[1] = joint_cost*stomp_parameters_->getSmoothnessCostAcceleration();
        derivative_costs[2] = joint_cost*stomp_parameters_->getSmoothnessCostJerk();

        // WARNING WATCH OUT FOR THE DIFFRULE
        joint_costs_.push_back(ChompCost(group_trajectory_.getNumPoints() + 2*(DIFF_RULE_LENGTH-1), i, derivative_costs, stomp_parameters_->getRidgeFactor()));

        double cost_scale = joint_costs_[i].getMaxQuadCostInvValue();
        if (max_cost_scale < cost_scale)
            max_cost_scale = cost_scale;
    }

    // scale the smoothness costs
    for (int i=0; i<joint_costs_.size(); i++)
    {
        joint_costs_[i].scale(max_cost_scale);
    }

    ///--------------------------------------------------------------------------
    /// ADD A GOAL TO THE POLICY IMPROVEMENT LOOP

    project_last_config_ = false;
//    project_last_config_ = PlanEnv->getBool(PlanParam::trajStompMoveEndConfig);

    if( project_last_config_ && ( planning_group_ != NULL ) )
    {
        ratio_projected_ = 1.0; // 1.0 = all along the trajectory

        Move3D::Robot* robot = planning_group_->robot_;
        eef_ = robot->getJoint( PlanEnv->getString(PlanParam::end_effector_joint) ); // plannar J4

        if( eef_ != NULL )
        {
            Move3D::confPtr_t q_goal = robot->getGoalPos();
            robot->setAndUpdate(*q_goal);
            x_task_goal_ = eef_->getVectorPos().head(3); // head(2) only (x,y)
        }
        else {
            project_last_config_ = false;
            x_task_goal_ = Eigen::VectorXd::Zero(0);
        }
    }

    ///--------------------------------------------------------------------------

    policy_ = policy;
    policy_->getControlCosts( control_costs_ );
    control_cost_weight_ = stomp_parameters_->getSmoothnessCostWeight();


    ///--------------------------------------------------------------------------
    ///
    // initialize the policy trajectory
    Eigen::VectorXd start   = group_trajectory_.getTrajectoryPoint(free_vars_start_).transpose();
    Eigen::VectorXd end     = group_trajectory_.getTrajectoryPoint(free_vars_end_).transpose();
    static_cast<CovariantTrajectoryPolicy*>(policy_)->setToMinControlCost( start, end );

    ///--------------------------------------------------------------------------
    /// JOINT LIMITS

    Eigen::VectorXd upper( num_vars_free_ * num_joints_ );
    Eigen::VectorXd lower( num_vars_free_ * num_joints_ );

    for (int joint=0; joint<num_joints_; joint++)
    {
        for (int i=free_vars_start_; i<=free_vars_end_; i++)
        {
            int id = i*num_joints_ + joint;
            upper( id ) = planning_group_->chomp_dofs_[joint].joint_limit_max_;
            lower( id ) = planning_group_->chomp_dofs_[joint].joint_limit_min_;
        }
    }

    joint_limits_computer_.dynamics_ = dynamics;
    joint_limits_computer_.upper_ = upper - 1e-5 * Eigen::VectorXd::Ones( upper.size() );
    joint_limits_computer_.lower_ = lower + 1e-5 * Eigen::VectorXd::Ones( lower.size() );

    if( !joint_limits_computer_.initialize() )
        cout << "ERROR could not initialize joint limits in " << __PRETTY_FUNCTION__ << endl;

    // PER JOINT
    joint_limits_computers_.resize( num_joints_ );

    for(int joint=0; joint<num_joints_; joint++)
    {
        double max_limit = planning_group_->chomp_dofs_[joint].joint_limit_max_ - 1e-5;
        double min_limit = planning_group_->chomp_dofs_[joint].joint_limit_min_ + 1e-5;

        joint_limits_computers_[joint].dynamics_ = control_costs_[joint]; //.block( DIFF_RULE_LENGTH, DIFF_RULE_LENGTH, num_vars_free_, num_vars_free_ );
        joint_limits_computers_[joint].upper_ = max_limit * Eigen::VectorXd::Ones(num_vars_free_);
        joint_limits_computers_[joint].lower_ = min_limit * Eigen::VectorXd::Ones(num_vars_free_) ;

        if( !joint_limits_computers_[joint].initialize() )
            cout << "ERROR could not initialize joint limits in " << __PRETTY_FUNCTION__ << endl;
    }

    time_cumul_ = 0.0;
    time_iter_ = 0;
}


void LampCostComputation::print_time() const
{
    cout << "total test time : " << std::scientific << time_cumul_ / double(time_iter_) << endl;
    cout << "total time : " << std::scientific << time_cumul_ << endl;
    cout << "total iter : " << time_iter_ << endl;
}

LampCostComputation::~LampCostComputation()
{
    cout << "destroy lamp cost computation" << endl;
}

void LampCostComputation::set_fct_get_nb_collision_points( boost::function<int(Move3D::Robot*)> fct, int id )
{
    if( id == int(move3d_get_number_of_collision_points_.size()) )
    {
        move3d_get_number_of_collision_points_.push_back( fct );
    }
    else if ( id < int(move3d_get_config_collision_cost_.size()) )
    {
        move3d_get_number_of_collision_points_[id] = fct;
    }
    else {
        cout << "ERROR SIZE in " << __PRETTY_FUNCTION__ << endl;
    }
}

void LampCostComputation::set_fct_get_config_collision_cost( boost::function<bool( Move3D::Robot* robot, int i, Eigen::MatrixXd&, std::vector< std::vector<Eigen::Vector3d> >& )> fct, int id )
{
    if( id == int(move3d_get_config_collision_cost_.size()) )
    {
        move3d_get_config_collision_cost_.push_back( fct );
    }
    else if( id < int(move3d_get_config_collision_cost_.size()) )
    {
        move3d_get_config_collision_cost_[id] = fct;
    }
    else {
        cout << "ERROR SIZE in " << __PRETTY_FUNCTION__ << endl;
    }
}

bool LampCostComputation::checkJointLimits(  LampTrajectory& group_traj  )
{
    bool succes_joint_limits = true;

    for (int joint=0; joint<num_joints_; joint++)
    {
        if (!planning_group_->chomp_dofs_[joint].has_joint_limits_)
            continue;

        double joint_max = planning_group_->chomp_dofs_[joint].joint_limit_max_;
        double joint_min = planning_group_->chomp_dofs_[joint].joint_limit_min_;

        double max_abs_violation =  1e-6;
        double violate_max_with = 0;
        double violate_min_with = 0;
        double absolute_amount;

        for (int i=free_vars_start_; i<=free_vars_end_; i++)
        {
            if ( group_traj( i, joint ) > joint_max )
            {
                // cout << "value is : " << group_traj( i, joint ) << " , max is : " << joint_max << endl;
                absolute_amount = std::fabs( joint_max - group_traj( i, joint ) );

                if( absolute_amount > violate_max_with )
                {
                    violate_max_with = absolute_amount;
                }
            }
            else if ( group_traj( i, joint ) < joint_min )
            {
                // cout << "value is : " << group_traj( i, joint ) << " , min is : " << joint_min << endl;
                absolute_amount = std::fabs( joint_min - group_traj( i, joint ) );

                if( absolute_amount > violate_min_with )
                {
                    violate_min_with = absolute_amount;
                }
            }

            if( absolute_amount > max_abs_violation )
            {
                max_abs_violation = absolute_amount;
                succes_joint_limits = false;
                break;
            }
        }

        if( !succes_joint_limits )
            return false;
    }

    return true;
}

bool LampCostComputation::handleJointLimitsQuadProg(  LampTrajectory& group_traj  )
{
//    if( checkJointLimits( group_traj ) )
//        return true;

//     double error = joint_limits_computer_.project( group_traj.trajectory_ );
    // cout << "error : " << error << endl;

    for(int joint=0; joint<num_joints_; joint++)
    {
        if(!planning_group_->chomp_dofs_[joint].has_joint_limits_)
            continue;

        Eigen::VectorXd joint_traj = group_traj.getDofTrajectoryBlock( joint );
        joint_limits_computers_[joint].project( joint_traj );
        group_traj.setDofTrajectoryBlock( joint, joint_traj );
    }

    if( checkJointLimits( group_traj ) )
        return true;

    // cout << "joint limits not ok! " << endl;

    return false;
}

bool LampCostComputation::handleJointLimits(  LampTrajectory& group_traj  )
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

                group_traj.setDofTrajectoryBlock( joint, Eigen::VectorXd::Zero(num_vars_free_) );
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
            // bool violate_max = false;
            // bool violate_min = false;
            double violate_max_with = 0;
            double violate_min_with = 0;

            for (int i=free_vars_start_; i<=free_vars_end_; i++)
            {
                double amount = 0.0;
                double absolute_amount = 0.0;
                if ( group_traj( i, joint ) > joint_max )
                {
                    amount = joint_max - group_traj(i, joint);
                    absolute_amount = fabs(amount);
                    // violate_max = true;
                    if( absolute_amount > violate_max_with )
                    {
                        violate_max_with = absolute_amount;
                    }
                }
                else if ( group_traj(i, joint) < joint_min )
                {
                    amount = joint_min - group_traj(i, joint);
                    absolute_amount = fabs(amount);
                    // violate_min = true;
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


                double multiplier = max_violation / joint_costs_[joint].getQuadraticCostInverse()( free_var_index, free_var_index );
                Eigen::VectorXd offset = multiplier * (joint_costs_[joint].getQuadraticCostInverse().col( free_var_index )).segment( id_fixed_, num_vars_free_-id_fixed_ );
                group_traj.addToDofTrajectoryBlock( joint , offset );
                // double offset = ( joint_max + joint_min )  / 2 ;

                // cout << "multiplier : " << multiplier << endl;
                // cout << "joint limit max : " << joint_max << endl;
                // cout << "joint limit min : " << joint_min << endl;
                // cout << "offset : " << offset << endl;
                // cout << group_trajectory_.getFreeJointTrajectoryBlock(joint).transpose() << endl;

                // group_trajectory_.getFreeJointTrajectoryBlock(joint) = ( group_trajectory_.getFreeJointTrajectoryBlock(joint).array() - offset )*multiplier + ( offset );

                // cout << group_trajectory_.getFreeJointTrajectoryBlock(joint).transpose() << endl;
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

    // cout << "succes_joint_limits : " << succes_joint_limits << endl;

    //    exit(1);

    return succes_joint_limits;
}

bool LampCostComputation::getControlCosts(const LampTrajectory& group_traj)
{
    std::vector<Eigen::VectorXd> parameters( num_joints_ );
    group_traj.getFreeParameters( parameters ); // TODO check the paramters size

    //            cout << "sum control costs" << endl;
    current_control_costs_ = std::vector<Eigen::VectorXd>( num_joints_, Eigen::VectorXd::Zero(num_vars_free_) );

    if( multiple_smoothness_ )
    {
        // cout << "policy_.get() : " << policy_.get() << endl;

        std::vector< std::vector<Eigen::VectorXd> > control_costs;

        static_cast<CovariantTrajectoryPolicy*>(policy_)->getAllCosts( parameters, control_costs, group_traj.getDiscretization() );

//        cout << "control_costs.size() " << control_costs.size() << endl;
//        cout << "current_control_costs_.size() " << current_control_costs_.size() << endl;
//        cout << "num joint : " << num_joints_ << endl;

        for (int c=0; c<4; ++c)
            for (int d=0; d<num_joints_; ++d) {
//                cout << "control_costs[c].size() : " << control_costs[c].size() << endl;
//                cout << "current_control_costs_[d] : " << current_control_costs_[d].size() << endl;
                current_control_costs_[d] += ( control_cost_weights_[c] * control_costs[c][d] );
            }

        // cout << "weight vector : " << control_cost_weights_.transpose() << endl;

        // cout << "GET CONTROL COST" << endl;

        for (int c=4; c<8; ++c) {
            general_cost_potential_.segment( free_vars_start_, num_vars_free_) += ( control_cost_weights_[c] * control_costs[c][0] );
        }
    }
    else
    {
            // cout << "NORMAL COMPUTATION" << endl;

        // TODO see why num_vars_free_ is 100 and not 100 - id_fixed
//        cout << "parameter size : " << parameters[0].size() << endl;
//        cout << "noise size : " << num_vars_free_ << endl;

        policy_->computeControlCosts(control_costs_,
                                     parameters,
                                     std::vector<Eigen::VectorXd>( num_joints_, Eigen::VectorXd::Zero(num_vars_free_) ),
                                     control_cost_weight_ ,
                                     current_control_costs_,
                                     group_traj.getDiscretization() );

//        for (int d=0; d<num_joints_; ++d)
//        {
//            general_cost_potential_.segment( free_vars_start_, num_vars_free_) += ( current_control_costs_[d] );

//            for (int i=0; i<current_control_costs_[d].size(); ++i) {
//                current_control_costs_[d][i] = 0.0;
//            }
//        }

    }

    return true;
}

double LampCostComputation::getCost() const
{
//    double cost = state_costs_.sum();
    double cost = 0.0;
    int num_dim = current_control_costs_.size();
    for (int d=0; d<num_dim; ++d)
        cost += current_control_costs_[d].sum();
    return cost;
}

double LampCostComputation::getCollisionSpaceCost( const Configuration& q )
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

void LampCostComputation::getMove3DConfiguration( const Eigen::VectorXd& joint_array, Configuration& q ) const
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
}

void LampCostComputation::getFrames( int segment, const Eigen::VectorXd& joint_array, Configuration& q )
{
    // Get configuration
    getMove3DConfiguration( joint_array, q );

    // Set robot to configuration
    robot_model_->setAndUpdate( q );

    // g3d_draw_allwin_active();

    const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

    // Get the collision point position
    for(int j=0; j<planning_group_->num_dofs_;j++)
    {
        segment_frames_[segment][j] = joints[j].move3d_joint_->getMatrixPos();
        joint_pos_eigen_[segment][j] = segment_frames_[segment][j].translation();

        if( move3d_collision_space_ )
        {
            joint_axis_eigen_[segment][j](0) = segment_frames_[segment][j](0,2);
            joint_axis_eigen_[segment][j](1) = segment_frames_[segment][j](1,2);
            joint_axis_eigen_[segment][j](2) = segment_frames_[segment][j](2,2);
        }
    }
}

bool LampCostComputation::getCollisionPointObstacleCost( int segment, int coll_point, double& collion_point_potential, Eigen::Vector3d& pos )
{
    bool colliding = false;

    if( move3d_collision_space_ )
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
    }

    return colliding;
}

bool LampCostComputation::getConfigObstacleCost( Move3D::Robot* robot, int i, Eigen::MatrixXd& collision_point_potential, std::vector< std::vector<Eigen::Vector3d> >& collision_point_pos )
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
}

int LampCostComputation::getNumberOfCollisionPoints(Move3D::Robot* R)
{
    return planning_group_->collision_points_.size();
}

bool LampCostComputation::performForwardKinematics( const LampTrajectory& group_traj, bool is_rollout )
{
    double invTime = 1.0 / group_traj.getDiscretization();
    double invTimeSq = invTime*invTime;

    // calculate the forward kinematics for the fixed states only in the first iteration:
    int start = free_vars_start_;
    int end = free_vars_end_;

    is_collision_free_ = true;

    Eigen::VectorXd joint_array( num_joints_ );

    Move3D::Configuration q( *source_ );
    Move3D::Configuration q_tmp( *robot_model_->getCurrentPos() );
    Move3D::Configuration q_prev( robot_model_ );
    // Move3D::Trajectory current_traj( robot_model_ );

    int k = 0;
    int way_point_ratio = -1;

    // for each point in the trajectory
    for (int i=start; i<=end; ++i)
    {
        state_is_in_collision_[i] = false;

        group_traj.getTrajectoryPointP3d( group_traj.getFullTrajectoryIndex(i), joint_array );

        this->getFrames( i, joint_array, q ); // Perform FK (set and update)
        // current_traj.push_back( confPtr_t(new Configuration(q)) );

        if( false && is_rollout && ( i%way_point_ratio != 0 ) && ( i != start ) && (i != end ) )
        {
            cout << "no fk for : " << i << endl;
            general_cost_potential_[i] = general_cost_potential_[i-1];
            state_is_in_collision_[i]  = state_is_in_collision_[i-1];

            if( move3d_collision_space_ || use_external_collision_space_ )
            {
                for( int j=0; j<num_collision_points_;j++)
                {
                    collision_point_potential_(i,j) = collision_point_potential_(i-1,j);
                    collision_point_pos_eigen_[i][j] = collision_point_pos_eigen_[i-1][j];
                }
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
                state_is_in_collision_[i] = move3d_get_config_collision_cost_[ collision_space_id_ ]( robot_model_, i, collision_point_potential_, collision_point_pos_eigen_ );
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

        q_prev = q; // store configuration
        q = *source_; // Make sure dofs are set to source

        k++;
    }

    // now, get the vel and acc for each collision point (using finite differencing)
    for (int i=free_vars_start_2_; i<=free_vars_end_2_; i++)
    {
        for (int j=0; j<num_collision_points_; j++)
        {
            collision_point_vel_eigen_[i][j] = Eigen::Vector3d::Zero();
            collision_point_acc_eigen_[i][j] = Eigen::Vector3d::Zero();

            //! TODO set buffer or something else
            for (int k=-DIFF_RULE_LENGTH/2; k<=DIFF_RULE_LENGTH/2; k++)
            {
                collision_point_vel_eigen_[i][j] += (invTime * DIFF_RULES[0][k+DIFF_RULE_LENGTH/2])     * collision_point_pos_eigen_[i+k][j];
                collision_point_acc_eigen_[i][j] += (invTimeSq * DIFF_RULES[1][k+DIFF_RULE_LENGTH/2])   * collision_point_pos_eigen_[i+k][j];
            }

            // get the norm of the velocity:
            collision_point_vel_mag_(i,j) = collision_point_vel_eigen_[i][j].norm();
        }
    }

    robot_model_->setAndUpdate( q_tmp );

    return is_collision_free_;
}

void LampCostComputation::projectToConstraints( LampTrajectory& group_traj ) const
{
    Eigen::VectorXd joint_array( num_joints_ );
    Move3D::confPtr_t q_end = robot_model_->getCurrentPos();

    group_traj.getTrajectoryPointP3d( group_traj.getFullTrajectoryIndex(free_vars_end_), joint_array );
    getMove3DConfiguration( joint_array, *q_end );

    Move3D::GeneralIK ik( robot_model_ );
    robot_model_->setAndUpdate( *q_end );

    ik.initialize( planning_group_->getActiveJoints(), eef_ );

    // cout << "x_task_goal_.size() : " << x_task_goal_.size() << endl;

    ik.solve( x_task_goal_ );
    Move3D::confPtr_t q_cur = robot_model_->getCurrentPos();
    const std::vector<int> active_dofs = ik.getActiveDofs();
    Eigen::VectorXd q_1 = q_end->getEigenVector( active_dofs );
    Eigen::VectorXd q_2 = q_cur->getEigenVector( active_dofs );
    Eigen::VectorXd dq = q_2 - q_1;

    // Get dq through J+
//    ik.magnitude_ = 1.0;
//    Eigen::VectorXd dq = ik.single_step_joint_limits( x_task_goal_ );

    int start = double(1. - ratio_projected_) * num_vars_free_;
    double alpha = 0.0;
    double delta = 1.0 / double(num_vars_free_-start);

    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        for( int joint=0; joint<num_joints_; joint++)
            group_traj(i, joint) += alpha * dq[joint];

        alpha += delta;
    }
}

bool LampCostComputation::getCost( Move3D::LampTrajectory& traj, Eigen::VectorXd& costs, const int iteration_number, bool joint_limits, bool resample, bool is_rollout )
{
    // copy the parameters into group_trajectory_
    group_trajectory_ = traj;

    // project to joint limits
    if( joint_limits )
    {
        double t_init, t_end;

        ChronoTimeOfDayTimes(&t_init);

        // succeded_joint_limits_ = handleJointLimits( group_trajectory_ );
        succeded_joint_limits_ = handleJointLimitsQuadProg( group_trajectory_ );

        ChronoTimeOfDayTimes(&t_end);

        time_cumul_ += (t_end - t_init );
        time_iter_++;

        // cout << std::scientific << "ts : " << t_end - t_init << " sec." << endl;
    }

    // project to goal set
    if( project_last_config_ && succeded_joint_limits_ )
    {
        projectToConstraints( group_trajectory_ );
    }

    // copy the group_trajectory_ parameters
    if( joint_limits || project_last_config_ || resample )
    {
        traj = group_trajectory_;
    }

    // do forward kinematics:
    bool trajectory_collision_free = performForwardKinematics( group_trajectory_, is_rollout );

    // Special case for not handling joint limits
    if( !succeded_joint_limits_ )
    {
        is_collision_free_ = false;
        trajectory_collision_free = false; // collision free but not valid
    }

    getControlCosts( group_trajectory_ );

    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        double state_collision_cost = 0.0;
        double state_general_cost = 0.0;
        double cumulative = 0.0;

        if( move3d_collision_space_ || use_external_collision_space_ )
        {
            for (int j=0; j<num_collision_points_; j++)
            {
                cumulative += collision_point_potential_(i,j) * collision_point_vel_mag_(i + DIFF_RULE_LENGTH - 1,j); // Becarful ... with velocity breaks of taken off
                state_collision_cost += cumulative;
            }
        }
        if( use_costspace_ || PlanEnv->getBool(PlanParam::useLegibleCost) )
        {
            state_general_cost = ( pow( general_cost_potential_(i) , hack_tweek ) /* ( q_f - q_i ).norm() */) ;
        }

        double cost = 0.0;
        cost += stomp_parameters_->getObstacleCostWeight() * state_collision_cost;
        cost += stomp_parameters_->getGeneralCostWeight() * state_general_cost;
        // cost *= dt_[i]; // optimize integral of cost

        costs(i-free_vars_start_) = cost;
    }

    return trajectory_collision_free;
}

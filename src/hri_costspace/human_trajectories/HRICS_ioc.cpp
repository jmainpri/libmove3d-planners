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
#include "HRICS_ioc.hpp"

#include "HRICS_parameters.hpp"
#include "HRICS_human_cost_space.hpp"
#include "HRICS_human_simulator.hpp"

#include "API/project.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "utils/misc_functions.hpp"
#include "utils/NumsAndStrings.hpp"

#include "feature_space/spheres.hpp"
#include "feature_space/squares.hpp"

#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planner/AStar/AStarPlanner.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Util-pkg.h>

#ifdef OWLQN_LIB
#include <owlqn/OWLQN.h>
#include <owlqn/leastSquares.h>
#include <owlqn/logreg.h>
#endif

#include <iomanip>
#include <sstream>
#include <fstream>

using namespace Move3D;
using namespace HRICS;
using std::cout;
using std::endl;
using std::cin;

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------

IocTrajectory::IocTrajectory( int nb_joints, int nb_var, double discretization )
{
    //    cout << "nb_joints : " << nb_joints << endl;
    //    cout << "nb_var : " << nb_var << endl;

    discretization_ = discretization;

    nominal_parameters_.clear();
    parameters_.clear();
    noise_.clear();
    control_costs_.clear();
    total_costs_.clear();

    for (int d=0; d<nb_joints; ++d)
    {
        nominal_parameters_.push_back( Eigen::VectorXd::Zero( nb_var ) );
        parameters_.push_back( Eigen::VectorXd::Zero( nb_var ) );
        noise_.push_back( Eigen::VectorXd::Zero( nb_var ) );
        control_costs_.push_back( Eigen::VectorXd::Zero(nb_var) );
        total_costs_.push_back( 0.2 * Eigen::VectorXd::Ones(nb_var) );
    }
    state_costs_ = Eigen::VectorXd::Zero(nb_var);
    out_of_bounds_ = false;
}

Move3D::Trajectory IocTrajectory::getMove3DTrajectory( const ChompPlanningGroup* p_g ) const
{
    Robot* rob = p_g->robot_;

    if( parameters_.empty() )
    {
        cout << "empty parameters" << endl;
        return Move3D::Trajectory( rob );
    }

    const std::vector<ChompJoint>& joints = p_g->chomp_joints_;

    // Create move3d trajectory
    Move3D::Trajectory T( rob );

    if( discretization_ != 0.0 ){
        T.setUseTimeParameter( true );
        T.setUseConstantTime( true );
        T.setDeltaTime( discretization_ );
    }

    for ( int j=0; j<parameters_[0].size(); ++j )
    {
        confPtr_t q = rob->getCurrentPos();

        for ( size_t i=0; i<parameters_.size(); ++i ) {
            (*q)[joints[i].move3d_dof_index_] = parameters_[i][j];
        }

        T.push_back( q->copy() );
    }

    return T;
}

//! Interpolates linearly two configurations
//! u = 0 -> a
//! u = 1 -> b
Eigen::VectorXd IocTrajectory::interpolate( const Eigen::VectorXd& a, const Eigen::VectorXd& b, double u ) const
{
    Eigen::VectorXd out;
    if( a.size() != b.size() )
    {
        cout << "Error in interpolate" << endl;
        return out;
    }

    out = a;
    for( int i=0;i<int(out.size());i++)
    {
        out[i] += u*(b[i]-a[i]);
    }
    return out;
}

void IocTrajectory::setSraightLineTrajectory()
{
    // Set size
    straight_line_ = parameters_;

    // Get number of points
    int nb_points = straight_line_[0].size();
    int dimension = straight_line_.size();

    if( dimension < 1 )
    {
        cout << "Error : the trajectory only has one dimension" << endl;
        return;
    }

    if( nb_points <= 2 )
    {
        cout << "Warning : the trajectory is less or equal than 2 waypoints" << endl;
        return;
    }

    Eigen::VectorXd a(dimension);
    Eigen::VectorXd b(dimension);
    Eigen::VectorXd c(dimension);

    for ( int j=0; j<dimension; j++ ) {
        a[j] = straight_line_[j][0];
        b[j] = straight_line_[j][nb_points-1];
    }

    double delta = 1 / double(nb_points-1);
    double s = 0.0;

    // Only fill the inside points of the trajectory
    for( int i=1; i<nb_points-1; i++ )
    {
        c = interpolate( a, b, s );
        s += delta;

        for( int j=0; j<dimension; j++ )
            straight_line_[j][i] = c[j];
    }
}

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------

IocSampler::IocSampler()
{

}

IocSampler::IocSampler( int num_var_free, int num_joints ) : num_vars_free_(num_var_free), num_joints_(num_joints)
{

}

void IocSampler::initialize()
{
    initPolicy();
    policy_.getControlCosts( control_costs_ );
    preAllocateMultivariateGaussianSampler();
    tmp_noise_ = Eigen::VectorXd::Zero(num_vars_free_);
}

void IocSampler::initPolicy()
{
    policy_.setPrintDebug( false );

    std::vector<double> derivative_costs(3);
    derivative_costs[0] = 0.0; // velocity
    derivative_costs[1] = 0.0; // acceleration
    derivative_costs[2] = 1.0; // smoothness

    // initializes the policy
    policy_.initialize( num_vars_free_, num_joints_, 1.0, 0.0, derivative_costs );
}

bool IocSampler::preAllocateMultivariateGaussianSampler()
{
    // invert the control costs, initialize noise generators:
    inv_control_costs_.clear();
    noise_generators_.clear();

    for (int j=0; j<num_joints_; ++j)
    {
        inv_control_costs_.push_back( control_costs_[j].inverse() );

        // Uncomment to print the precision Matrix
        // cout << endl << control_costs_[j] << endl;

        // TODO see of the noise generator needs to be
        // var free or var all
        MultivariateGaussian mvg( Eigen::VectorXd::Zero( num_vars_free_ ), inv_control_costs_[j] );
        noise_generators_.push_back( mvg );
    }

    return true;
}

Eigen::MatrixXd IocSampler::sample(double std_dev)
{
    Eigen::MatrixXd traj( num_joints_, num_vars_free_ );

    for( int i=0;i<num_joints_;i++)
    {
        noise_generators_[i].sample( tmp_noise_ );
        traj.row(i) = std_dev*tmp_noise_;
    }
    return traj;
}

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------

Ioc::Ioc( int num_vars, const ChompPlanningGroup* planning_group ) :
    planning_group_( planning_group ),
    num_vars_( num_vars ),
    num_joints_( planning_group->num_joints_ ),
    sampler_( num_vars, num_joints_ )
{
    demonstrations_.clear();
    noise_stddev_ = HriEnv->getDouble(HricsParam::ioc_sample_std_dev);
    sampler_.initialize();
}

bool Ioc::addDemonstration( const Eigen::MatrixXd& demo, double discretization )
{
    if( num_joints_ != demo.rows() || num_vars_ != demo.cols() )
    {
        cout << "Error in add demonstration ";
        cout << "(num_joints : " << num_joints_ << " , " << demo.rows() << " ) " ;
        cout << "(num_vars_ : " << num_vars_ << " , " << demo.cols() << " )" ;
        cout << endl;
        return false;
    }

    IocTrajectory t( num_joints_, num_vars_, discretization );

    for(int i=0;i<int(t.parameters_.size());i++)
    {
        t.parameters_[i] = demo.row( i );
    }

    demonstrations_.push_back( t );

    // Set straight line for sampling
    demonstrations_.back().setSraightLineTrajectory();

    // Resize sample vector to the number of demos
    samples_.resize( demonstrations_.size() );

    return true;
}

bool Ioc::addSample( int d, const Eigen::MatrixXd& sample )
{
    if( num_joints_ != sample.rows() || num_vars_ != sample.cols()  )
    {
        cout << "Error in add sample : trajectory dimension not appropriate" << endl;
        return false;
    }
    if( d >= int(samples_.size()) ){
        cout << "Error in add sample : nb of demonstrations too small" << endl;
        return false;
    }

    IocTrajectory t( num_joints_, num_vars_, demonstrations_[d].discretization_ );

    for(int i=0;i<int(t.parameters_.size());i++)
    {
        t.parameters_[i] = sample.row( i );
    }

    samples_[d].push_back( t );

    return true;
}

//! Set mean
bool Ioc::setNominalSampleValue( int d, int i, const Eigen::MatrixXd& nominal_parameters )
{
    if( num_joints_ != nominal_parameters.rows() || num_vars_ != nominal_parameters.cols()  )
    {
        cout << "Error in add nominal sample values : trajectory dimension not appropriate" << endl;
        return false;
    }
    if( d >= int(samples_.size()) ){
        cout << "Error in add nominal sample values : nb of demonstrations too small" << endl;
        return false;
    }
    for(int k=0;k<int(samples_[d][i].nominal_parameters_.size());k++)
    {
        samples_[d][i].nominal_parameters_[k] = nominal_parameters.row( k );
    }
    return true;
}

//! Set mean
bool Ioc::setTotalCostsSampleValue( int d, int i, const Eigen::VectorXd& total_costs )
{
    if( num_vars_ != total_costs.rows()  )
    {
        cout << "Error in add total_costs : trajectory dimension not appropriate";
        cout << " : (" << num_vars_ << ") : (" << total_costs.cols() << ", " << total_costs.rows() << ")" << endl;
        return false;
    }
    if( d >= int(samples_.size()) ){
        cout << "Error in add total_costs : nb of demonstrations too small" << endl;
        return false;
    }
    for(int k=0;k<int(samples_[d][i].total_costs_.size());k++)
    {
        samples_[d][i].total_costs_[k] = total_costs;
    }
    return true;
}

//! returns true if in joint limits
bool Ioc::jointLimits( IocTrajectory& traj ) const
{
    bool is_in_joint_limits = true;

//    cout << "num_vars_ : " << num_joints_ << endl;
//    cout << "num_vars_ : " << num_vars_ << endl;

    for( int j=0;j<num_joints_;j++)
    {
        double coeff = 1.0;

        double j_max = planning_group_->chomp_joints_[j].joint_limit_max_;
        double j_min = planning_group_->chomp_joints_[j].joint_limit_min_;

        int nb_attempt = 0;

        for( int i=0; i<num_vars_; i++ )
        {
            // TODO SEE PROBLEM WITH smoothness
            if( planning_group_->chomp_joints_[j].is_circular_ && j_max == M_PI && j_min == -M_PI ) {
                traj.parameters_[j][i] =  angle_limit_PI( traj.parameters_[j][i] );
            }

            while( ( traj.parameters_[j][i] < j_min || traj.parameters_[j][i] > j_max ) && ( nb_attempt < 10 ) )
            {
//                cout << "not in limits, name : " << planning_group_->chomp_joints_[j].joint_name_ << endl;
//                cout << j << " : upper : " << j_max << ", lower : " << j_min << ", value : " << traj.parameters_[j][i] << endl;

                coeff *= 0.90; // 90 percent (10 * 0.9 = 0.3)
                traj.noise_[j] *= coeff;
                traj.parameters_[j] = traj.nominal_parameters_[j] + traj.noise_[j];
                nb_attempt++;
            }

            if( nb_attempt >= 10 ) {
                is_in_joint_limits = false;
            }

            nb_attempt = 0;
        }

        if( !is_in_joint_limits ) // If not in joint limits make sure to cap
        {
//            cout << "capping joint : " << planning_group_->chomp_joints_[j].joint_name_ << endl;

            for( int i=0; i<num_vars_; i++ )
            {
                if( traj.parameters_[j][i] < j_min || traj.parameters_[j][i] > j_max )
                {
//                    cout << "not in limits, name : " << planning_group_->chomp_joints_[j].joint_name_ << endl;
//                    cout << j << " : upper : " << j_max << ", lower : " << j_min << ", value : " << traj.parameters_[j][i] << endl;

                    if( traj.parameters_[j][i] < j_min )
                        traj.parameters_[j][i] = j_min + 1e-6;
                    if( traj.parameters_[j][i] > j_max )
                        traj.parameters_[j][i] = j_max - 1e-6;
                }
            }
        }

        is_in_joint_limits = true;

        for( int i=0; i<num_vars_; i++ )
            if( traj.parameters_[j][i] < j_min || traj.parameters_[j][i] > j_max )
                is_in_joint_limits = false;

        if( !is_in_joint_limits )
            cout << "joint " << planning_group_->chomp_joints_[j].joint_name_ << " not in limit" << endl;

        // Set the start and end values constant
        traj.parameters_[j].start(1) = traj.nominal_parameters_[j].start(1);
        traj.parameters_[j].end(1) = traj.nominal_parameters_[j].end(1);
    }

//    cout << "is_in_joint_limits : " << is_in_joint_limits << endl;

    return is_in_joint_limits;
}

//void Ioc::pureRandomSampling( int nb_samples )
//{
//    int nb_demos = getNbOfDemonstrations();

//    samples_.resize( nb_demos );

//    for (int d=0; d<nb_demos; ++d)
//    {
//        Move3D::Trajectory demo = demonstrations_[d].getMove3DTrajectory( planning_group_ );

//        confPtr_t q_init = demo.getBegin();
//        confPtr_t q_goal = demo.getEnd();

//        samples_[d].resize( nb_samples );

//        for (int ns=0; ns<int(samples_[d].size()); ++ns )
//        {
//            samples_[d][ns] = IocTrajectory( num_joints_, num_vars_ );

//            std::vector<Configuration> configs;

//            for (int k=0; k<int(samples_[d].size()); ++k )
//            {
//                configs.push_back( planning_group_->robot_->shoot() );
//            }

//            for (int ns=0; ns<int(samples_[d].size()); ++ns )
//            {
//                configs.push_back( planning_group_->robot_->shoot() );
//            }

//            for (int ns=0; ns<int(samples_[d].size()); ++ns )
//            {

//            }
//        }
//    }
//}

bool Ioc::isTrajectoryValid( const IocTrajectory& traj )
{
    Move3D::Trajectory path = traj.getMove3DTrajectory( planning_group_ );
    Move3D::Robot* robot = path.getRobot();
    Move3D::Scene* sce = global_Project->getActiveScene();


    std::vector<Move3D::Robot*> others;
    for( int i=0; i<sce->getNumberOfRobots(); i++ )
    {
        std::string robot_name = sce->getRobot(i)->getName();

        if( robot_name != robot->getName() && robot_name.find("HUMAN") == std::string::npos) {
            others.push_back( sce->getRobot(i) );
//            cout << "Add robot " << robot_name << " to others" << endl;
        }
    }

    for( int i=0; i<path.getNbOfViaPoints(); i++ )
    {
        robot->setAndUpdate( *path[i] );
        if( robot->isInCollisionWithOthers( others ) ){
            return false;
        }
    }

    return true;
}

int Ioc::generateSamples( int nb_samples, bool check_in_collision, context_t context )
{
    int nb_demos = getNbOfDemonstrations();

    samples_.resize( nb_demos );

    int nb_of_invalid_samples = 0;

    for( int d=0; d<nb_demos; ++d )
    {
        cout << "Generating samples for demonstration : " << d << endl;

        samples_[d].resize( nb_samples );

        for( int i=0; i<int(context.size()); i++ )
        {
            Move3D::Robot* entity = context[i][d]->getRobot();
            entity->setAndUpdate( *context[i][d] );
//            g3d_draw_allwin_active();
//            cout << "wait for key" << endl;
//            std::cin.ignore();
        }

        for (int ns=0; ns<int(samples_[d].size()); ++ns )
        {
//            cout.flush();
//            cout << "generating for demo " << d << " sample : " << int(ns+1) << "\r";

            samples_[d][ns] = IocTrajectory( num_joints_, num_vars_, demonstrations_[d].discretization_ );

            bool is_valid = true;
            int nb_failed = 0;
            do
            {
                // Sample noisy trajectory
                Eigen::MatrixXd noisy_traj = sampler_.sample( noise_stddev_ );

                for( int j=0; j<num_joints_; ++j )
                {
                    // Change to generate samples around demonstration
                    if( HriEnv->getBool(HricsParam::ioc_sample_around_demo) )
                        samples_[d][ns].nominal_parameters_[j] = demonstrations_[d].parameters_[j];
                    else
                        samples_[d][ns].nominal_parameters_[j] = demonstrations_[d].straight_line_[j]; // TODO why commented

                    samples_[d][ns].noise_[j] = noisy_traj.row(j);
                    samples_[d][ns].parameters_[j] = samples_[d][ns].nominal_parameters_[j] + samples_[d][ns].noise_[j]; //.cwiseProduct(samples_[d][ns].total_costs_[j]);
                }

                /*is_valid =*/ jointLimits( samples_[d][ns] );

                if( check_in_collision ) {
                    is_valid = isTrajectoryValid( samples_[d][ns] );
//                    is_valid = samples_[d][ns].getMove3DTrajectory( planning_group_ ).isValid();
                }
            }
            // Commented for humans
            while( (!is_valid) && ( nb_failed++ < 10 ) );

            // The sample trajectory is not valid
            if( !is_valid ) nb_of_invalid_samples++;// cout << "WARNING: generating invalid sample, in collision : " << check_in_collision << endl;
        }
    }

    cout << endl;

    return nb_of_invalid_samples; // TODO return false when out of bounds
}

std::vector< std::vector<Move3D::Trajectory> > Ioc::getSamples()
{
    std::vector< std::vector<Move3D::Trajectory> > samples(demonstrations_.size());

    for ( int d=0; d<int(demonstrations_.size()); ++d)
    {
        for ( int k=0; k<int(samples_[d].size()); ++k)
        {
            samples[d].push_back( samples_[d][k].getMove3DTrajectory( planning_group_ ) );
        }
    }

    return samples;
}

std::vector< Move3D::Trajectory > Ioc::getDemonstrations()
{
    std::vector<Move3D::Trajectory> demos(demonstrations_.size());

    for ( int d=0; d<int(demonstrations_.size()); ++d)
    {
        demos[d] = demonstrations_[d].getMove3DTrajectory( planning_group_ );
    }
    return demos;
}

void Ioc::addTrajectoryToDraw( const IocTrajectory& t, int color )
{
    Move3D::Trajectory T = t.getMove3DTrajectory( planning_group_ );
    T.setColor( color );
    global_trajToDraw.push_back( T );
}

void Ioc::addAllToDraw()
{
    global_trajToDraw.clear();

    if( HriEnv->getBool(HricsParam::ioc_draw_demonstrations) )
    {
        for ( int d=0; d<int(demonstrations_.size()); ++d)
        {
            cout << "adding to draw demo [" << d << "]" << endl;
            addTrajectoryToDraw( demonstrations_[d], d%8 );
        }
    }

    if( HriEnv->getBool(HricsParam::ioc_draw_samples) )
    {
        for ( int d=0; d<int(demonstrations_.size()); ++d)
        {
            for ( int k=0; k<int(samples_[d].size()); ++k)
            {
                cout << "adding to draw sample [" << d << "][" << k << "]" <<endl;
                addTrajectoryToDraw( samples_[d][k], k%8 );
            }
        }
    }
}

//////////////////////////////////////////////////////
// IOC objective
//////////////////////////////////////////////////////
#ifdef OWLQN_LIB
struct IocObjective : public DifferentiableFunction
{
    IocObjective() { }

    //! Main virtual function
    double Eval(const DblVec& w, DblVec& dw);

    Eigen::VectorXd getEigenVector(const DblVec& w);
    Eigen::VectorXd numericalGradient(double loss, const Eigen::VectorXd& w);
    double value(const Eigen::VectorXd& w);

    //! demonstrations features
    std::vector<Eigen::VectorXd> phi_demo_;

    //! sample features
    std::vector< std::vector<Eigen::VectorXd> > phi_k_;
};

Eigen::VectorXd IocObjective::getEigenVector( const DblVec& w )
{
    Eigen::VectorXd w_(w.size());

    for( size_t i=0; i<w.size(); i++ )
    {
        w_[i]=w[i];
    }
    return w_;
}

double IocObjective::value(const Eigen::VectorXd& w)
{
    double loss = 0.0;

    for (size_t d=0; d<phi_demo_.size();d++)
    {
        double denominator = 1e-9;
        for (size_t k=0; k<phi_k_.size();k++)
            denominator += std::exp(-1.0*w.transpose()*phi_k_[d][k]);

        loss += std::log( std::exp(-1.0*w.transpose()*phi_demo_[d]) / denominator );
    }

    loss = -loss;

    return loss;
}

Eigen::VectorXd IocObjective::numericalGradient(double loss, const Eigen::VectorXd& w)
{
    double eps = 1e-6;

    Eigen::VectorXd g(w.size());

    for( int i=0; i<g.size(); i++)
    {
        Eigen::VectorXd w_tmp(w);
        w_tmp[i] = w[i] + eps;
        g[i] = ( value(w_tmp) - loss ) / eps;
    }

    return g;
}

double IocObjective::Eval( const DblVec& w, DblVec& dw )
{
    double loss = 1.0;

    if( w.size() != dw.size() )
    {
        cout << "error in size : " << __PRETTY_FUNCTION__ << endl;
        return loss;
    }

    Eigen::VectorXd w_(getEigenVector(w));
    Eigen::VectorXd dw_(Eigen::VectorXd::Zero(dw.size()));

    // cout << endl;
    //cout << "w : " << w_.transpose() << endl;

    // Loss function
    loss = value(w_);

    // Gradient
    Eigen::VectorXd dw_n = numericalGradient( loss, w_ );

    for (int d=0; d<int(phi_demo_.size()); d++)
    {
        double denominator = 0.0;
        for (size_t k=0; k<phi_k_.size();k++)
            denominator += std::exp(-1.0*w_.transpose()*phi_k_[d][k]);

        for (int i=0; i<int(w.size()); i++)
        {
            double numerator = 0.0;
            for (size_t k=0; k<phi_k_.size();k++)
                numerator += std::exp(-1.0*w_.transpose()*phi_k_[d][k]) * phi_k_[d][k][i];

            dw_[i] +=  phi_demo_[d][i] - ( numerator /  denominator );
        }
    }

    for (int i=0; i<dw_.size(); i++)
        dw[i] = dw_[i];

    if( std::isnan(loss) )
    {
        cout << endl;
        cout << "w : " << w_.transpose() << endl;
        cout << "loss : " << loss << endl;
        cout <<  "----------------" << endl;
        cout << "dw_s : " << dw_.transpose() << endl;
        cout << "dw_n : " << dw_n.transpose() << endl;
        cout << "error : " << (dw_n - dw_).norm() << endl;

        //        exit(1);
    }

    return loss;
}
#endif

Eigen::VectorXd Ioc::solve( const std::vector<Eigen::VectorXd>& phi_demo, const std::vector< std::vector<Eigen::VectorXd> >& phi_k )
{
    if( phi_demo.size() < 1 )
    {
        cout << "no demo passed in Ioc solver" << endl;
        return Eigen::VectorXd();
    }

#ifdef OWLQN_LIB
    size_t size = phi_demo[0].size();

    IocObjective obj;
    obj.phi_demo_ = phi_demo;
    obj.phi_k_ = phi_k;


    Eigen::VectorXd w0 = Eigen::VectorXd::Zero(size);
    Eigen::VectorXd w1 = Eigen::VectorXd::Zero(size);
    for( int i=0;i<int(size);i++)
        w1[i] = 1;

    cout << "zeros value : " << obj.value(w0) << endl;
    cout << "ones value : " << obj.value(w1) << endl;

    DblVec ans(size);
    /*
    // Initial value ones
    DblVec init(size,1);

    for( int i=0;i<int(init.size());i++)
    {
//        init[i] = p3d_random(1,2);
        init[i] = 1;
    }

    bool quiet=false;
    int m = 50;
    double regweight=1;
    double tol = 1e-6;

    OWLQN opt(quiet);
    opt.Minimize( obj, init, ans, regweight, tol, m );
    */

    return obj.getEigenVector(ans);
#else

    return Eigen::VectorXd();
#endif
}

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------

IocEvaluation::IocEvaluation(Robot* rob, int nb_demos, int nb_samples, int nb_way_points, MultiplePlanners& planners, StackedFeatures* features, std::vector<int> active_joints,
                             std::string folder,  std::string traj_folder, std::string tmp_data_folder )
    : robot_(rob), planners_(planners)
{
    // Dimention of the problem space
    nb_demos_ = nb_demos;
    nb_samples_ = nb_samples;
    nb_way_points_ = nb_way_points;
    nb_planning_test_ = 1;

    // Folders
    folder_ = folder; // move3d_demo_folder; // static
    traj_folder_ = traj_folder;
    tmp_data_folder_ = tmp_data_folder;

    load_sample_from_file_ = HriEnv->getBool(HricsParam::ioc_load_samples_from_file);
    remove_samples_in_collision_ = true;

    active_joints_ = active_joints;

    plangroup_ = new ChompPlanningGroup( robot_, active_joints_ );

    feature_fct_ = features;

    nb_weights_ = feature_fct_->getNumberOfFeatures();
    original_vect_ = feature_fct_->getWeights();

    feature_type_ = "";

    // Active dofs are set when the planning group is created
    smoothness_fct_ = new TrajectorySmoothness;
    smoothness_fct_->setActiveDoFs( plangroup_->getActiveDofs() );

    // Set the feature type
    if( dynamic_cast<Spheres*>( feature_fct_->getFeatureFunction(0)) != NULL )
        feature_type_ = "spheres";

    if( dynamic_cast<Squares*>( feature_fct_->getFeatureFunction(0)) != NULL )
        feature_type_ = "squares";

    if( dynamic_cast<HumanTrajCostSpace*>( feature_fct_ ) != NULL )
        feature_type_ = "human_trajs";

    // Set active dofs
    if( feature_type_ == "spheres" || feature_type_ == "squares ")
    {
        global_PlanarCostFct->setActiveDoFs( plangroup_->getActiveDofs() );
    }

    use_context_ = false;
    use_simulator_ = false;
}

// Types are : astar, rrt, stomp
Move3D::Trajectory IocEvaluation::planMotion( planner_t type )
{
    if( use_simulator_ )
    {
        cout << "RUN SIMULATOR FOR DEMO : " << demo_id_ << endl;
        global_ht_simulator->setDemonstrationId( demo_id_ );
        global_ht_simulator->run();
        Move3D::Trajectory traj = global_ht_simulator->getExecutedPath();
        return traj;
    }
    else
    {
        planners_.clearTrajs();
        planners_.setPlannerType( type );

        if( planners_.run() )
        {
            return planners_.getBestTrajs()[0];
        }
        else{
            Move3D::Trajectory traj(robot_);
            return traj;
        }
    }
}

void IocEvaluation::runPlannerMultipleFeature( int nb_runs )
{
    planners_.clearTrajs();
    planners_.setPlannerType( planner_t(HriEnv->getInt(HricsParam::ioc_planner_type)) );

    for( int i=0; i<nb_runs; i++ )
    {
        std::vector<int> active_feature;
        for( int j=0;j<feature_fct_->getNumberOfFeatures();j++)
        {
            cout << "--------------------------------------------" << endl;
            cout << "RUN : " << i << " , FEATURE : " << j << endl;
            active_feature.clear();
            active_feature.push_back(j);
            feature_fct_->setActiveFeatures( active_feature );
            // planners_.multipleRun(1);
            planners_.run();
            g3d_draw_allwin_active();
        }
    }

    planners_.saveTrajsToFile( traj_folder_ );
}

void IocEvaluation::activateAllFeatures()
{
    std::vector<int> active_feature;
    for( int i=0;i<feature_fct_->getNumberOfFeatures();i++)
        active_feature.push_back(i);

    feature_fct_->setActiveFeatures( active_feature );
}

WeightVect IocEvaluation::computeOptimalWeights()
{
    WeightVect w( WeightVect::Ones( feature_fct_->getNumberOfFeatures() ) );

    const Move3D::Trajectory& traj = planners_.getBestTrajs().back();

    // Set all feature active
    activateAllFeatures();

    // Feature count
    FeatureVect phi = feature_fct_->getFeatureCount( traj );
    // phi = phi.array() + EPS6;

    // Jacobian Norm Per DoF
    FeatureJacobian J = feature_fct_->getFeatureJacobian( traj );
    FeatureVect phi_gradient_norm( FeatureVect::Zero( feature_fct_->getNumberOfFeatures() ) );
    for( int i=0;i<w.size();i++) // For each feature
        phi_gradient_norm(i) += J.col(i).norm();

    // COMPUTATION OF THE NEXT
    stored_features_[0] += phi;
    stored_features_[1] += phi_gradient_norm;

    static_cast<Squares*>(global_PlanarCostFct)->phi_cumul_ = stored_features_[0];
    static_cast<Squares*>(global_PlanarCostFct)->phi_jac_cumul_ = stored_features_[1];

//    phi = stored_features_[0].normalized();
//    phi_gradient_norm = stored_features_[1].normalized();

    w = stored_features_[0];
    w /= w.maxCoeff();

//    for( int i=0;i<w.size();i++) // For each feature
//    {
//        w( = phi_cumul_ ;
//    }

//    for( int i=0;i<w.size();i++) // For each feature
//    {
//        w(i) = std::abs( phi(i) - phi_demos_[0](i) );
//    }

//    w.array() + EPS3;
//    w.normalize();

    return w;
}

Move3D::Trajectory IocEvaluation::selectBestSample( double detla_mean, const std::vector<Move3D::Trajectory>& trajs )
{
    PlanarFeature* planar_fct = dynamic_cast<PlanarFeature*>( feature_fct_->getFeatureFunction(0) );

    double delta_min=std::numeric_limits<double>::max();
    int k=0;
    for( size_t i=0; i<trajs.size(); i++ )
    {
        double delta  = planar_fct->getDeltaSum( planners_.getAllTrajs()[k] );
        if( delta < delta_min ){
            delta_min = delta;
            k = i;
        }
    }
    return planners_.getAllTrajs()[k];
}

void IocEvaluation::runPlannerWeightedFeature( int nb_runs )
{
    cout << __PRETTY_FUNCTION__ << endl;

    planners_.clearTrajs();
    planners_.setPlannerType( planner_t(HriEnv->getInt(HricsParam::ioc_planner_type)) );

    activateAllFeatures();

    HRICS::Ioc ioc( nb_way_points_, plangroup_ );

    // Get weight vector
    WeightVect w( WeightVect::Ones( feature_fct_->getNumberOfFeatures() ) );
    feature_fct_->setWeights( w );

    // Get demos features
    phi_demos_ = addDemonstrations( ioc );

    // Jac sum of demos
    std::vector< std::vector< Move3D::Trajectory > > trajs_tmp; trajs_tmp.push_back( demos_ );
    std::vector< std::vector<FeatureVect> > jac_sum_demos = getFeatureJacobianSum( trajs_tmp );
    phi_jac_demos_ = jac_sum_demos.back();

    // Store features
    stored_features_.resize( 2, FeatureVect::Zero( feature_fct_->getNumberOfFeatures() ) );

    PlanarFeature* planar_fct = dynamic_cast<PlanarFeature*>( feature_fct_->getFeatureFunction(0) );
    if( planar_fct != NULL ) {
//        planar_fct->balance_cumul_jac_ = 1.0; // un comment to only track the feature count
        planar_fct->phi_demos_ = phi_demos_[0]; // = stored_features_[0];
        planar_fct->phi_cumul_ = FeatureVect::Ones( feature_fct_->getNumberOfFeatures() );
        planar_fct->phi_jac_cumul_ = FeatureVect::Zero( feature_fct_->getNumberOfFeatures() );
        planar_fct->demos_ = demos_;
    }

    cout <<  "phi_demos_ : " <<  planar_fct->phi_demos_.transpose() << endl;

    // Nb of run per_feature
    const int per_feature_run = 10;

    planar_fct->iter_ = 0;
    planar_fct->increment_delta_ = 2.0;

    double sum_delta = 0.0;
    double mean_delta = 0.0;
    double sum_variance = 0.0;
    double mean_variance = 0.0;

    std::vector<Move3D::Trajectory> best_trajs;

    planners_.setStompInit( demos_[0] );

    // Nb of total run
    for( int i=0; i<nb_runs; i++ )
    {
        for( int j=0;j<feature_fct_->getNumberOfFeatures();j++)
        {
            cout << "--------------------------------------------" << endl;
            cout << "RUN : " << i << " , FEATURE : " << j << endl;
            cout << "BLANCE : " <<  planar_fct->balance_cumul_jac_ << endl;

//            cout << "distance to feature count : " << ( planar_fct->phi_demos_ - ( planar_fct->phi_cumul_/ planar_fct->phi_cumul_.maxCoeff() ) ).norm() << endl;
//            cout << "distance to feature count : " << ( planar_fct->phi_demos_ - ( planar_fct->phi_cumul_/ planar_fct->phi_cumul_.maxCoeff() ) ).transpose() << endl;

//            std::vector<int> active_features;
//            active_features.clear();
//            active_features.push_back( j );
//            feature_fct_->setActiveFeatures( active_features );

//            planar_fct->produceDerivativeFeatureCostMap( i*planar_fct->getNumberOfFeatures() + j );

            planners_.run();


//            best_trajs.push_back( selectBestSample( mean_delta, planners_.getAllTrajs() ) );
            best_trajs.push_back( planners_.getBestTrajs().back() );
            best_trajs.back().replaceP3dTraj();

            double variance = planar_fct->getVariance( best_trajs.back() );
            double delta = planar_fct->getDeltaSum( best_trajs.back() );

            planar_fct->iter_++;

            sum_variance += variance;
            mean_variance = sum_variance / double(planar_fct->iter_);

            sum_delta += delta;
            mean_delta = sum_delta / double(planar_fct->iter_);

            if( mean_delta < 5 )
                planar_fct->increment_delta_ /= 1.10;
            else {
                if( delta > 0 )
                    planar_fct->increment_delta_ *= 1.5;
            }

            cout << "--------------------------------------------" << endl;
            cout << " VARIANCE : " << variance << endl;
            cout << " VARIANCE MEAN : " << mean_variance << endl;
            cout << " DELTA : " << delta << endl;
            cout << " DELTA MEAN : " << mean_delta << endl;
            cout << " DELTA INCREMENT : " << planar_fct->increment_delta_ << endl;
            cout << " ITER : " << planar_fct->iter_ << endl;

//            global_trajToDraw.clear();

//            for( size_t k=0;k<planners_.getAllTrajs().size();k++)
//            {
//                cout << "var : " << planar_fct->getVariance( planners_.getAllTrajs()[k] );
//                cout << " , delta : "  << planar_fct->getDeltaSum( planners_.getAllTrajs()[k] ) << endl;
//                global_trajToDraw.push_back( planners_.getAllTrajs()[k] );
//            }

            g3d_draw_allwin_active(); // TODO??? Keep this before optimal weight computation

            w = computeOptimalWeights();
//            feature_fct_->setWeights( w );
//            feature_fct_->printWeights();

            if( planar_fct != NULL ) {
                planar_fct->cur_min_diff_ =  planar_fct->min_diff_;
                planar_fct->min_diff_ = std::numeric_limits<double>::max();
            }

            if( PlanEnv->getBool(PlanParam::stopPlanner) )
                break;
        }

        if( PlanEnv->getBool(PlanParam::stopPlanner) )
            break;

        if( planar_fct != NULL )
            planar_fct->balance_cumul_jac_ += (1.0/double(per_feature_run));

        // WARNING RESET EVERY 10
        if( (i+1)%(per_feature_run) == 0 ) {
            w = Eigen::VectorXd::Ones( feature_fct_->getNumberOfFeatures() );
            stored_features_[0] = Eigen::VectorXd::Zero( feature_fct_->getNumberOfFeatures() );

            sum_delta = 0.0;
            mean_delta = 0.0;
            sum_variance = 0.0;
            mean_variance = 0.0;

            if( planar_fct != NULL ) {
                planar_fct->balance_cumul_jac_ = 0.0;
                planar_fct->phi_cumul_ = FeatureVect::Zero( feature_fct_->getNumberOfFeatures() );
                planar_fct->phi_jac_cumul_ = FeatureVect::Zero( feature_fct_->getNumberOfFeatures() );
                planar_fct->iter_ = 0;
                planar_fct->increment_delta_ = 2.0;
            }
        }
    }

    saveTrajectories( best_trajs );

//    planners_.saveTrajsToFile( move3d_traj_folder );
}

void IocEvaluation::generateDemonstrations( int nb_demos )
{
    std::vector<Move3D::Trajectory> demos;

    nb_demos_ = nb_demos;

    cout << "nb_demos : " << nb_demos_ << endl;
    cout << "nb_planning_test_ : " << nb_planning_test_ << endl;

    feature_fct_->printInfo();

    std::vector<int> demos_id;

    for(int i=0; i<nb_demos_; i++ )
    {
        std::vector< std::pair<double,Move3D::Trajectory> > demos_tmp( nb_planning_test_ );

        cout << "___________________________________________________" << endl;
        cout << " demo nb : " << i << endl;

        if( i >= 1 ) // Generate multiple start and goals
        {
            Move3D::confPtr_t q_init = robot_->shoot();
            Move3D::confPtr_t q_goal = robot_->shoot();
            robot_->setInitPos(*q_init);
            robot_->setGoalPos(*q_goal);
        }

        for( int j=0; j<nb_planning_test_; j++ )
        {
            demos_tmp[j].second = planMotion( planner_type_ );
            demos_tmp[j].first = feature_fct_->costTraj( demos_tmp[j].second );
        }

        for( int j=0; j<nb_planning_test_; j++ )
            cout << "cost[" << j << "] : " << demos_tmp[j].first << endl;

        cout << "min cost : " << std::min_element(demos_tmp.begin(), demos_tmp.end() )->first << endl;
        cout << "nb of via points : " << std::min_element(demos_tmp.begin(), demos_tmp.end() )->second.getNbOfViaPoints() << endl;

        demos.push_back( std::min_element(demos_tmp.begin(), demos_tmp.end() )->second );
        demos_id.push_back(i);

        g3d_draw_allwin_active();
    }

    cout << "nb_demos_ : " << nb_demos_ << endl;

    saveDemoToFile( demos, demos_id );

    cout << "exit generate demo" << endl;
}

void IocEvaluation::saveDemoToFile(const std::vector<Move3D::Trajectory>& demos, const std::vector<int>& demos_ids, std::vector<Move3D::confPtr_t> context )
{
    int id = 0;

    // Create a buffer to keep track of demo ids being saved
    // each demo is incremented when saved
    std::vector<int> saved_id( *std::max_element( demos_ids.begin(), demos_ids.end() )+1, 0 );

    global_trajToDraw.clear();

    for(size_t i=0;i<demos.size();i++)
    {
//        if( demos[i].replaceP3dTraj() )
//        {
//            cout << "has replaced p3d traj" << endl;
//            cout << robot_->getP3dRobotStruct()->tcur << endl;
//            cout << robot_->getName() << endl;
//        }

        global_trajToDraw.push_back( demos[i] );

        // Ids
        std::stringstream ss_id;
        ss_id << "_"  << std::setw(3) << std::setfill( '0' ) << demos_ids[i] << "_"  << std::setw(3) << std::setfill( '0' ) << saved_id[ demos_ids[i] ]++ ;

        // Set file names
        std::stringstream ss;
        ss << folder_ << "trajectory_" << feature_type_ << ss_id.str() << ".traj";

        cout << "save demo " << i << " : " << ss.str() << endl;
        cout << "nb of via points  : " << demos[i].getNbOfViaPoints() << endl;
        // p3d_save_traj( ss.str().c_str(), robot_->getP3dRobotStruct()->tcur );
        demos[i].saveToFile( ss.str() );

        saveTrajToMatlab( demos[i], i );
        cout << "save traj to matlab format!!!!" << endl;
    }

    if( !context.empty() )
    {
        // Ids
        std::stringstream ss_id;
        ss_id << "_"  << std::setw(3) << std::setfill( '0' ) << int(0) << "_"  << std::setw(3) << std::setfill( '0' ) << int(0) ;

        std::stringstream ss;
        ss.str("");
        ss << folder_ << "context_" << feature_type_ << ss_id.str() << ".configs" ;

        cout << "save context " << ss.str() << endl;
        move3d_save_context_to_csv_file( context, ss.str() );
    }

    cout  << "DEMO SIZE : " << demos.size() << endl;
}

void IocEvaluation::saveSamplesToFile(const std::vector< std::vector<Move3D::Trajectory> >& samples ) const
{
    for(size_t d=0; d<samples.size(); d++)
    {
        for(size_t i=0; i<samples[d].size(); i++)
        {
            std::stringstream ss;
            ss << folder_ << "samples/" << "trajectory_sample_" << feature_type_ <<
                  "_"  << std::setw(3) << std::setfill( '0' ) << d <<
                  "_"  << std::setw(3) << std::setfill( '0' ) << i << ".traj";

            cout << "save sample : " << i << " : " << ss.str() << endl;

            samples[d][i].saveToFile( ss.str() );
        }
    }
}

std::vector< std::vector< Move3D::Trajectory> > IocEvaluation::loadSamplesFromFile( int nb_demos, int nb_samples ) const
{
    std::vector< std::vector< Move3D::Trajectory> > samples( nb_demos );

    for(size_t d=0; d<nb_demos; d++)
    {
        cout << "Load samples of demo : " << d << endl;

        for(size_t i=0; i<nb_samples; i++)
        {
            std::stringstream ss;
            ss << folder_ << "samples/" << "trajectory_sample_" << feature_type_ <<
                  "_"  << std::setw(3) << std::setfill( '0' ) << d <<
                  "_"  << std::setw(3) << std::setfill( '0' ) << i << ".traj";

//            cout << "load sample : " << i << " : " << ss.str() << endl;

            samples[d].push_back( Move3D::Trajectory( robot_ ) );
            samples[d][i].loadFromFile( ss.str() );
        }
    }

    return samples;
}

bool file_exists_test(const std::string& name) {
    std::ifstream f( name.c_str() );
    if (f.good()) {
        f.close();
        return true;
    } else {
        f.close();
        return false;
    }
}

bool IocEvaluation::loadDemonstrations()
{
    if( nb_way_points_ <= 0 ){
        cout << "Error in : " << __PRETTY_FUNCTION__ << "can not load demonstration be cause number of way point not set " <<  endl;
        return false;
    }

    demos_.clear();
    context_.clear();
    global_trajToDraw.clear();
    std::stringstream ss;

    std::string folder = use_simulator_ ? original_demo_folder_ : folder_;

    int demo_id = 0;
    int instance_id = 0;

    if( use_context_ )
    {
        std::stringstream ss_id;
        ss_id << "_"  << std::setw(3) << std::setfill( '0' ) << demo_id << "_"  << std::setw(3) << std::setfill( '0' ) << instance_id ;
        ss.str("");
        ss << folder << "context_" << feature_type_ << ss_id.str() << ".configs";
        context_.push_back( move3d_load_context_from_csv_file( ss.str() ) );

        nb_demos_ = context_[0].size();

//        Move3D::Robot* robot = context_[0][0]->getRobot();

//        for( size_t d=0; d<context_.size(); d++)
//        {
//            robot->setAndUpdate( *context_[0][d] );
//            g3d_draw_allwin_active();
//            cout << "Type enter to exit : " << endl;
//            std::cin.ignore();
//        }
    }

    demo_id = 0;
    instance_id = 0;

    int discarded_demo = -1;
    std::vector<int> demos_to_remove;

    for(int d=0;d<nb_demos_;d++)
    {
        bool file_exists = false;

        do {
            std::stringstream ss_id;
            ss_id << "_"  << std::setw(3) << std::setfill( '0' ) << demo_id << "_"  << std::setw(3) << std::setfill( '0' ) << instance_id ;
            ss.str("");
            ss << folder << "trajectory_" << feature_type_ << ss_id.str() << ".traj";

            file_exists = file_exists_test( ss.str() );

            if( !file_exists )
            {
                demo_id++;
                instance_id = 0;
            }
        }
        while( !file_exists );

        if( discarded_demo == demo_id )
            demos_to_remove.push_back( d );

        cout << "Loading demonstration (" << d << ") from : " << ss.str() << endl;

        Move3D::Trajectory T( robot_ );

        if( !T.loadFromFile( ss.str() ) )//!p3d_read_traj( ss.str().c_str() ) )
        {
            cout << "could not load trajectory" << endl;
            return false;
        }
        else {
            cout << "p3d trajectory loaded correctly" << endl;
        }

        // Move3D::Trajectory T = robot_->getCurrentTraj();
//        T.computeSubPortionIntergralCost( T.getCourbe() );

        if( T.getNbOfViaPoints() != nb_way_points_ ) // Only cut if needed
        {
            cout << "ring in " << nb_way_points_ << " way points" << endl;
            T.cutTrajInSmallLP(nb_way_points_-1);
//            T.computeSubPortionIntergralCost( T.getCourbe() );
        }

//        T.replaceP3dTraj();

        T.setColor( d%8 ); cout << "color : " << d%8 << endl;
        global_trajToDraw.push_back(T);

        demos_.push_back( T );

        instance_id++;
    }

    context_t context(1);
    for(int i=0; i<context_[0].size(); i++){
        if( std::find( demos_to_remove.begin(), demos_to_remove.end(), i ) == demos_to_remove.end() )
            context[0].push_back( context_[0][i] );
    }
    context_ = context;

    cout << "End loading demonstrations" << endl;

    return true;
}

void IocEvaluation::loadPlannerTrajectories( int nb_trajs, int offset, int random )
{
    cout << "planners_.getBestTrajs().size() : " << planners_.getBestTrajs().size() << endl;

    if( nb_trajs < 0 ){
        nb_trajs = planners_.getBestTrajs().size();
    }
    if( offset < 0 ){
        offset = 0;
    }
    if( (random == 0) &&  ((offset + nb_trajs) > int(planners_.getBestTrajs().size() )) ){
        samples_.clear();
        cout << "out of bounds in : " << __PRETTY_FUNCTION__ << endl;
        return;
    }

    int sequence_size = 16;

    if( random == 0 ) // Continuous sequence
    {
        std::vector<Move3D::Trajectory>::const_iterator first = planners_.getBestTrajs().begin() + offset ;
        std::vector<Move3D::Trajectory>::const_iterator last  = planners_.getBestTrajs().begin() + offset + nb_trajs ;
        samples_ = std::vector<Move3D::Trajectory>(first, last);
    }
    else if( random == 1 ) // Random pick in each sequence
    {
        // int nb_sequences =  int(planners_.getBestTrajs().size()) / sequence_size;
        int nb_big_sequence = int(planners_.getBestTrajs().size()) / nb_trajs;

        samples_.clear();
        // cout << "nb of sequences : " << nb_sequences << endl;
        for(int i=0;i<nb_trajs;i++)
        {
            int id = i + nb_trajs * p3d_random_integer( 0, nb_big_sequence-1 );
            // cout << "Add id : " << id << endl;
            samples_.push_back( planners_.getBestTrajs()[id] );
        }
    }

    global_trajToDraw.clear();
    for( size_t i=0; i<samples_.size(); i++ )
    {
        double color = (double(i%sequence_size)/sequence_size);
        // cout << "i : " << i << ", color : " << color << endl;
        samples_[i].cutTrajInSmallLP( nb_way_points_-1 );
        samples_[i].setUseContinuousColors(true);
        samples_[i].setColor( color );

        global_trajToDraw.push_back( samples_[i] );
    }

    g3d_draw_allwin_active();
}

void IocEvaluation::loadWeightVector()
{
    cout << "Load Weight Vector" << endl;

    learned_vect_ = Eigen::VectorXd::Zero( nb_weights_ );

    // Load vector from file
    std::stringstream ss;
    ss << tmp_data_folder_ << "spheres_weights_" << std::setw(3) << std::setfill( '0' ) << nb_samples_ << ".txt";

    cout << "LOADING LEARNED WEIGHTS : " << ss.str() << endl;
    std::ifstream file( ss.str().c_str() );
    std::string line, cell;

    int i=0;

    if( file.good() )
    {
        std::getline( file, line );
        std::stringstream lineStream( line );

        while( std::getline( lineStream, cell, ',' ) )
        {
            std::istringstream iss( cell );
            iss >> learned_vect_[i++];
        }
    }
    else {
        cout << "ERROR could not load weights" << endl;
    }
    file.close();

    cout << " LEARNED weight : " << learned_vect_.transpose() << endl;
}

void IocEvaluation::setContext(int d)
{
    for( int i=0; i<int(context_.size()); i++ )
    {
        Move3D::Robot* entity = context_[i][d]->getRobot();
        entity->setAndUpdate( *context_[i][d] );
    }
}

void IocEvaluation::setBuffer(int d)
{
    if( ( d > 0 ) && (demo_ids_[d] != d ) )
    {
        if( !demo_ids_.empty() && demos_[ demo_ids_[d] ].getUseTimeParameter() )
        {
            double time_along_original_demo = demos_[ demo_ids_[d] ].getTimeLength() - demos_[d].getTimeLength();

//            cout << "demos_[ demo_ids_[d] ].getTimeLength() : " << demos_[ demo_ids_[d] ].getTimeLength() << endl;
//            cout << "demos_[d].getTimeLength() : " << demos_[d].getTimeLength() << endl;
//            cout << "time_along_original_demo : " << time_along_original_demo << endl;

            double dt = demos_[d].getDeltaTime();
            std::vector<Eigen::VectorXd> buffer;
            int nb_config = 7;
            for(int i=0; i<nb_config; i++){
                double t = time_along_original_demo-double(nb_config-i)*dt;
                Move3D::confPtr_t q = demos_[ demo_ids_[d] ].configAtTime( t );
                buffer.push_back( q->getEigenVector( plangroup_->getActiveDofs() ) );
                // cout << "buffer " << d << " [" << i << "] " << t << " : " << buffer[i].transpose() << endl;
            }
            static_cast<SmoothnessFeature*>(feature_fct_->getFeatureFunction("SmoothnessAll"))->setBuffer( buffer );
        }
    }
}

std::vector<FeatureVect> IocEvaluation::addDemonstrations(HRICS::Ioc& ioc)
{
    // Get features of demos
    std::vector<FeatureVect> phi_demo(demos_.size());

    for( size_t d=0; d<demos_.size(); d++ )
    {
        if( demos_[d].getNbOfViaPoints() != nb_way_points_ ) // Only cut of needed
            demos_[d].cutTrajInSmallLP( nb_way_points_-1 );

        if( use_context_ )
            setContext(d);

        if( feature_fct_->getFeatureFunction("SmoothnessAll") )
            setBuffer(d);

        FeatureVect phi = feature_fct_->getFeatureCount( demos_[d] );
        cout << "Feature Demo : " << phi.transpose() << endl;
//        ioc.addDemonstration( demos_[i].getEigenMatrix(6,7) );
        ioc.addDemonstration( demos_[d].getEigenMatrix( plangroup_->getActiveDofs() ), demos_[d].getDeltaTime() );
        phi_demo[d] = phi;
    }

//    demos_[0].replaceP3dTraj();

    return phi_demo;
}

std::vector< std::vector<FeatureVect> > IocEvaluation::addSamples(HRICS::Ioc& ioc)
{
    // Get features of demos
    for( size_t i=0; i<samples_.size(); i++ )
    {
//        if( samples_[i].getNbOfViaPoints() != nb_way_points_ ) // Only cut of needed
            samples_[i].cutTrajInSmallLP( nb_way_points_-1 );
//        ioc.addSample( 0, samples_[i].getEigenMatrix(6,7) );
        ioc.addSample( 0, samples_[i].getEigenMatrix( plangroup_->getActiveDofs() ) );
//        ioc.setNominalSampleValue( 0, i, samples_[i].getEigenMatrix(6,7)  );
        ioc.setNominalSampleValue( 0, i, samples_[i].getEigenMatrix( plangroup_->getActiveDofs() ) );


        // FeatureProfile p = feature_fct_->getFeatureJacobianProfile( samples_[i] );
        // ioc.setTotalCostsSampleValue( 0, i, p );
        // p = p.array().pow( 4 );
        // cout << "p(" << i << ") : " << p.transpose() << endl;
    }

    // ioc.generateSamples( nb_samples_ );

    // Compute the sum of gradient
    // double gradient_sum = 0.0;

    std::vector< std::vector<Move3D::Trajectory> > samples = ioc.getSamples();
    std::vector< std::vector<FeatureVect> > phi_k( samples.size() );
    for( int d=0;d<int(samples.size());d++)
    {
        if( use_context_ ){

            for( int i=0;i<int(context_.size());i++)
            {
                Move3D::Robot* entity = context_[i][d]->getRobot();
                entity->setAndUpdate( *context_[i][d] );
            }
        }

        for( int i=0;i<int(samples[d].size());i++)
        {
            FeatureVect phi = feature_fct_->getFeatureCount( samples[d][i] );
            // cout << "Feature Sample : " << phi.transpose() << endl;
            // gradient_sum += feature_fct_->getJacobianSum( samples[d][i] );
            phi_k[d].push_back( phi );
        }
    }

    // cout << "gradient sum = " << gradient_sum << endl;

    return phi_k;
}

bool IocEvaluation::isSampleDominated( const FeatureVect& demo, const FeatureVect& sample ) const
{
    for( int i=0; i<sample.size(); i++ )
    {
        if( sample[i] < demo[i] )
        {
            // cout << "Sample is NOT dominated!!!!" << endl;
            return false;
        }
    }
    cout << "Sample is dominated!!!!" << endl;
    return true;
}

void IocEvaluation::removeDominatedSamplesAndResample( HRICS::Ioc& ioc, std::vector< std::vector<FeatureVect> >& phi_k )
{
    for( int d=0;d<int(phi_k.size());d++)
    {
        if( use_context_ ){ // TODO VERIFY THIS FUNCTION FOR CONTEXT

            for( int i=0;i<int(context_.size());i++)
            {
                Move3D::Robot* entity = context_[i][d]->getRobot();
                entity->setAndUpdate( *context_[i][d] );
            }
        }

        for( int i=0;i<int(phi_k[d].size());i++)
        {
            while( isSampleDominated( phi_demos_[d], phi_k[d][i]) )
            {
                ioc.generateSamples( 1, remove_samples_in_collision_ );
                std::vector< std::vector<Move3D::Trajectory> > samples = ioc.getSamples();

                for( int dd=0;dd<int(samples.size());dd++)
                {
                    for( int ii=0;ii<int(samples[dd].size());ii++)
                    {
                        phi_k[d][i] = feature_fct_->getFeatureCount( samples[dd][ii] );
                    }
                }
            }
        }
    }
}


bool IocEvaluation::isTrajectoryValid( Move3D::Trajectory& path )
{
    Move3D::Robot* robot = path.getRobot();
    Move3D::Scene* sce = global_Project->getActiveScene();


    std::vector<Move3D::Robot*> others;
    for( int i=0; i<sce->getNumberOfRobots(); i++ )
    {
        std::string robot_name = sce->getRobot(i)->getName();
        if( robot_name != robot->getName() &&
                robot_name.find("HUMAN") == std::string::npos)
            others.push_back( sce->getRobot(i) );
    }

    for( int i=0; i<path.getNbOfViaPoints(); i++ )
    {
        robot->setAndUpdate( *path[i] );
        if( robot->isInCollisionWithOthers( others ) ){
            return false;
        }
    }

    return true;
}

std::vector<std::vector<Move3D::Trajectory> > IocEvaluation::runSampling()
{
    cout << __PRETTY_FUNCTION__ << endl;

    global_trajToDraw.clear();

//    print_joint_mapping( robot_ );

//    for( int i=0; i<plangroup_->chomp_joints_.size();i++){
//        cout << " group joint name : "  << plangroup_->chomp_joints_[i].joint_name_ << endl;
//        cout << "              min : "  << plangroup_->chomp_joints_[i].joint_limit_min_ << endl;
//        cout << "              max : "  << plangroup_->chomp_joints_[i].joint_limit_max_ << endl;
//    }

    for( int i=0;i<demo_ids_.size();i++)
    {
        cout << "demo_ids_[" << i << "] : " << demo_ids_[i] << endl;
    }


    // For human
    feature_fct_->setAllFeaturesActive();

    // Compute the sum of gradient
    // double gradient_sum = 0.0;

    feature_fct_->printInfo();

    cout << "nb_demos : " << demos_.size() << endl;
    cout << "nb_samples : " << nb_samples_ << endl;

    std::vector< std::vector<Move3D::Trajectory> > samples;

    cout << "Create Ioc" << endl;
    HRICS::Ioc ioc( nb_way_points_, plangroup_ );

    // Get demos features
    cout << "Add demonstrations" << endl;
    phi_demos_ = addDemonstrations( ioc );

//    cout << "wait for key" << endl;
//    std::cin.ignore();

    bool generate = true;

    if( generate )
    {
        // Generate samples by random sampling
        cout << "Generate samples (" << nb_samples_ << ")" << endl;
        remove_samples_in_collision_ = true;
        int nb_invalid_samples = ioc.generateSamples( nb_samples_, remove_samples_in_collision_, context_ );

        cout << "percentage of invalid samples : " << (100 * double(nb_invalid_samples) / double(nb_samples_)) << " \%" << endl;
        samples = ioc.getSamples();

//        ioc.addAllToDraw();
//        saveSamplesToFile( samples );
    }
    else { // load from file
        samples = loadSamplesFromFile( demos_.size(), nb_samples_ );
    }

    double demo_cost = feature_fct_->getWeights().transpose()*phi_demos_[0];

    cout << "cost " << int(0) << " : " <<  demo_cost << endl;
    cout << "dist wrist " << phi_demos_[0][0] << endl; //24
    cout << "length : " << demos_[0].getParamMax() << endl;

    FeatureVect phi = feature_fct_->getFeatureCount( demos_[0] );
    cout << "weight : " << feature_fct_->getWeights().transpose() << endl;
    cout << "cost : " << feature_fct_->getWeights().transpose()*phi << " , ";
    cout << "Feature Sample : " << int(-1) << " , " << phi.transpose() << endl;

    Feature* fct = feature_fct_->getFeatureFunction("Distance");
    if( fct != NULL )
        cout << "distance cost : " << fct->costTraj( demos_[0] ) << endl;

    int nb_lower_cost = 0;
    int nb_lower_feature = 0;
    int nb_shorter = 0;
    int nb_in_collision = 0;

    // Get samples features
    std::vector< std::vector<FeatureVect> > phi_k( samples.size() );

    // For each demonstration
    for( int d=0;d<int(samples.size());d++)
    {
        if( use_context_ )
            setContext(d);

        if( feature_fct_->getFeatureFunction("SmoothnessAll") )
            setBuffer(d);

        for( int i=0; i<int(samples[d].size()); i++)
        {
            phi = feature_fct_->getFeatureCount( samples[d][i] );

//            if( d == 8 )
//            {
//                cout << "wait for key" << endl;
//                std::cin.ignore();
//            }

            phi_k[d].push_back( phi );

            double cost = feature_fct_->getWeights().transpose()*phi;

            // Compute stats ...
            if( cost < demo_cost )
                nb_lower_cost++;
            if( samples[d][i].getParamMax() < demos_[d].getParamMax() )
                nb_shorter++;
            if( /*!samples[d][i].isValid()*/ !isTrajectoryValid( samples[d][i] ) )
                nb_in_collision++;

            // cout << "cost : " << cost << " , ";
            //            cout.precision(4);
//            cout << "Feature Sample : " << i << " , " << phi.transpose() << endl;
//            cout << "dist wrist " << phi[0] << endl; //24
//            cout << "length : " << samples[d][i].getParamMax() << endl;

            for( int j=0; j<phi_demos_[d].size(); j++ ){
                if( phi[j] - phi_demos_[d][j] < 0 )
                     nb_lower_feature++;
            }

            // gradient_sum += feature_fct_->getJacobianSum( samples[d][i] );
            // cout << "Sample(" << d << "," <<  i << ") : " << phi_k[d][i].transpose() << endl;
//             cout << "Smoothness(" << d << "," <<  i << ") : " << phi_k[d][i][0] << endl;
        }
    }

    cout << "nb_lower_cost : " << nb_lower_cost << endl;
    cout << "nb_lower_feature : " << nb_lower_feature << endl;
    cout << "nb_shorter : " << nb_shorter << endl;
    cout << "nb_in_collision : " << nb_in_collision << endl;

//    removeDominatedSamplesAndResample( ioc, phi_k );

    // checkStartAndGoal( samples );

    saveToMatrixFile( phi_demos_, phi_k, "spheres_features" ); // TODO change name to motion...

    g3d_draw_allwin_active();

//    samples_ = samples[0];
//    saveTrajectories( samples_ );

//    std::vector< std::vector<FeatureVect> > jac_sum_samples = getFeatureJacobianSum( ioc.getSamples() );
//    saveToMatrixFile( phi_jac_demos_, jac_sum_samples, "spheres_jac_sum" );

    return samples;
}

void IocEvaluation::runFromFileSampling(int offset)
{
    HRICS::Ioc ioc( nb_way_points_, plangroup_ );

    // Get demos features
    phi_demos_ = addDemonstrations( ioc );

    // Jac sum of demos
    std::vector< std::vector< Move3D::Trajectory > > trajs_tmp; trajs_tmp.push_back( demos_ );
    std::vector< std::vector<FeatureVect> > jac_sum_demos = getFeatureJacobianSum( trajs_tmp );
    phi_jac_demos_ = jac_sum_demos.back();

    // Load samples from file
    // samples_ = planners_.getBestTrajs();

    // Random removal in the vector
    bool random_removal = false;
    if( random_removal )
    {
        while( int(samples_.size()) != nb_samples_ )
        {
            int pos = p3d_random_integer(0,samples_.size()-1);
            std::vector<Move3D::Trajectory>::iterator it = samples_.begin();
            std::advance(it, pos);
            samples_.erase(it);
        }
    }
    else {
        if( offset == -1 )
            loadPlannerTrajectories( nb_samples_, -1, 1 );
        else
            loadPlannerTrajectories( nb_samples_, offset, 0 );
    }

    global_trajToDraw.clear();
    std::vector< std::vector<FeatureVect> > phi_k(1);
    for( int i=0;i<int(samples_.size());i++)
    {
        global_trajToDraw.push_back( samples_[i] );
        FeatureVect phi = feature_fct_->getFeatureCount( samples_[i] );
        phi_k[0].push_back( phi );
    }

    // Get samples features
    // std::vector< std::vector<FeatureVect> > phi_k = addSamples( ioc );

    //    std::vector< std::vector<Move3D::Trajectory> > samples;
    //    samples.push_back( samples_ );
    //    checkStartAndGoal( samples );

    //    ioc.addAllToDraw();
    saveToMatrixFile( phi_demos_, phi_k, "spheres_features" );

    std::vector< std::vector< Move3D::Trajectory > > samples_trajs_tmp; samples_trajs_tmp.push_back( samples_ );

//    std::vector< std::vector<FeatureVect> > jac_sum_samples = getFeatureJacobianSum( samples_trajs_tmp );
//    saveToMatrixFile( phi_jac_demos_, jac_sum_samples, "spheres_jac_sum" );
}

//! Generate a distribution that maximizes entropy
void IocEvaluation::monteCarloSampling( double factor, int nb_tries )
{
    cout << __PRETTY_FUNCTION__ << endl;

    HRICS::Ioc ioc( nb_way_points_, plangroup_ );

    // Get demos features
    phi_demos_ = addDemonstrations( ioc );

    std::vector< std::vector<FeatureVect> > phi_k( phi_demos_.size() );

    bool sampling_done = false;

    global_trajToDraw.clear();

    setOriginalWeights();

    demos_[0].computeSubPortionIntergralCost( demos_[0].getCourbe() );


    while( sampling_done == false )
    {
        ioc.generateSamples( nb_samples_, remove_samples_in_collision_ );

        std::vector< std::vector<Move3D::Trajectory> > samples = ioc.getSamples();

        // cout << "d : " << samples.size() << endl;

        for( int d=0; (d<int(samples.size()))  && (sampling_done == false);d++)
        {
            for( int i=0; (i<int(samples[d].size())) && (sampling_done == false);i++)
            {
                if( *demos_[d].getBegin() != *samples[d][i].getBegin() )
                    cout << "Error in begin config" << endl;

                if( *demos_[d].getEnd() != *samples[d][i].getEnd() )
                    cout << "Error in end config" << endl;

                FeatureVect phi = feature_fct_->getFeatureCount( samples[d][i] );

                double cost = original_vect_.transpose() * ( phi - phi_demos_[d] );

                if( ( std::exp( - cost ) / factor ) > p3d_random(0,1) )
                {
                    phi_k[d].push_back( phi );
                    cout << "samples ( " << phi_k[d].size() << " ) : " << cost << endl;
                    samples[d][i].setColor( phi_k[d].size() / double(nb_samples_) );
                    global_trajToDraw.push_back( samples[d][i] );
                }

                if( cost < 0.0 ){
                    cout << "cost demo : " << original_vect_.transpose() * feature_fct_->getFeatureCount( demos_[d] ) << endl;
                    cout << "cost sample : " << original_vect_.transpose() * phi << endl;
//                    samples[d][i].setColor( samples[d].size() / double(nb_samples_) );
//                    global_trajToDraw.push_back( samples[d][i] );
                }

                if( int(phi_k[d].size()) == nb_samples_ ) {
                    sampling_done = true;
                    break;
                }

                if( PlanEnv->getBool(PlanParam::stopPlanner)){
                    sampling_done = true;
                    break;
                }
            }
        }

        g3d_draw_allwin_active();
    }

    saveToMatrixFile( phi_demos_, phi_k, "spheres_features" );
}

bool IocEvaluation::checkStartAndGoal( const std::vector< std::vector<Move3D::Trajectory> >& samples ) const
{
    for( int d=0;d<int(samples.size());d++)
    {
        for( int i=0;i<int(samples[d].size());i++)
        {
            cout << "sample(" << d << " , " << i << ") : " << endl;
            samples[d][i].getBegin()->print();
            samples[d][i].getEnd()->print();
        }
    }

    return true;
}


bool IocEvaluation::checkDegeneration( const std::vector< FeatureVect>& phi_demos, const std::vector< std::vector<FeatureVect> >& phi_k ) const
{
    if( phi_demos.size() != phi_k.size() ){
        cout << "Error" << endl;
        return false;
    }
    for( size_t d=0; d<phi_k.size(); d++ )
    {
        for( size_t k=0; k<phi_k[d].size(); k++ )
        {
            for( int i=0; i<phi_k[d][k].size(); i++ )
            {
                // TODO check that no domination exists
            }
        }
    }

    return true;
}

void IocEvaluation::runLearning()
{
    //    for( int i=0;i<1;i++)
    //    {
    //        Eigen::VectorXd w = ioc.solve( phi_demo, phi_k );
    //        cout << "w : " << w.transpose() << endl;
    //    }
}

Eigen::VectorXd IocEvaluation::getCostsOfDemonstrations() const
{
    Eigen::VectorXd costs( demos_.size() );

    for( int d=0;d<int(demos_.size());d++)
    {
        costs[d] = feature_fct_->costTraj( demos_[d] );
    }

    return costs;
}

void IocEvaluation::setLearnedWeights()
{
    cout << "Set Learned Weight Vector" << endl;
    feature_fct_->setWeights( learned_vect_ );
    feature_fct_->printWeights();
}

void IocEvaluation::setOriginalWeights()
{
    cout << "Set Original Weight Vector" << endl;
    feature_fct_->setWeights( original_vect_ );
    feature_fct_->printWeights();
}

Eigen::VectorXd IocEvaluation::compareDemosAndPlanned()
{
    loadDemonstrations();

    setOriginalWeights();
    Eigen::VectorXd costs_demo = getCostsOfDemonstrations();
    Eigen::VectorXd costs_learned( costs_demo.size() );
    learned_.resize( costs_demo.size() );

    cout << ( costs_demo ).transpose() << endl;
    cout << "NB OF DEMOS : " << costs_demo.size() << endl;

    int nb_tests = planner_type_ == astar ? 1 : nb_planning_test_;

    loadWeightVector(); // Load learned from file

    for( int i=0; i<costs_demo.size(); i++ )
    {
        cout << "--------------------" << endl;

         costs_learned[i] = 0.0;

         global_trajToDraw.clear();

         std::vector< std::pair<double,Move3D::Trajectory> > traj_tmp( nb_tests );

         setLearnedWeights(); // Plan under the learned weights

        for( int j=0; j<nb_tests; j++ )
        {
            demo_id_ = j;
            traj_tmp[j].second  = planMotion( planner_type_ );
            traj_tmp[j].first = feature_fct_->costTraj( traj_tmp[j].second );
        }

        for( int j=0; j<nb_tests; j++ )
            cout << "cost[" << j << "] : " << traj_tmp[j].first << endl;

        // Min over the number of planning sequence (for radomized planners)
        learned_[i] = std::min_element( traj_tmp.begin(), traj_tmp.end() )->second;
        global_trajToDraw.push_back( learned_[i] );
        g3d_draw_allwin_active();

        setOriginalWeights(); // Evaluate under the true weights
        learned_[i].resetCostComputed();
        costs_learned[i] = feature_fct_->costTraj( learned_[i] );
    }

    double weight_dist = ( learned_vect_ - original_vect_ ).norm();

    double mean_demo = costs_demo.mean();
    double sq_sum_demo = costs_demo.transpose()*costs_demo;
    double stdev_demo = std::sqrt( sq_sum_demo / double(costs_demo.size()) - (mean_demo * mean_demo) );

    double mean_learned = costs_learned.mean();
    double sq_sum_learned = costs_learned.transpose()*costs_learned;
    double stdev_learned = std::sqrt( (sq_sum_learned / double(costs_learned.size())) - (mean_learned * mean_learned) );

    cout << "--------------------------------------" << endl;

    cout << "weight dist : " << weight_dist << endl;
    cout << "cost of demos : " << costs_demo.transpose() << endl;
    cout << "cost of learned : " << costs_learned.transpose() << endl;
    cout << "mean_demo : " << mean_demo << " , stdev_demo : " << stdev_demo << endl;
    cout << "mean_learned : " << mean_learned << " , stdev_learned : " << stdev_learned << endl;
    cout << "mean diff : " << mean_learned - mean_demo << " , stdev diff : " << stdev_learned - stdev_demo  << endl;
    cout << "min diff : " << ( costs_learned - costs_demo ).minCoeff() << endl;
    cout << "max diff : " << ( costs_learned - costs_demo ).maxCoeff() << endl;

    Eigen::VectorXd result(6);
    result[0] = mean_demo;
    result[1] = mean_learned;
    result[2] = mean_learned - mean_demo;
    result[3] = weight_dist;
    result[4] = ( costs_learned - costs_demo ).minCoeff();
    result[5] = ( costs_learned - costs_demo ).maxCoeff();
    return result;
}

std::vector< std::vector<FeatureVect> > IocEvaluation::getFeatureJacobianSum( const std::vector< std::vector<Move3D::Trajectory> >& all_trajs )
{
    std::vector< std::vector<FeatureVect> > feature_jac_sum( all_trajs.size() );

    for(size_t k=0;k<all_trajs.size();k++) // For each demo
    {
        for(size_t l=0;l<all_trajs[k].size();l++) // For each sample
        {
            feature_jac_sum[k].push_back( FeatureVect::Zero( feature_fct_->getNumberOfFeatures() ) );

            FeatureJacobian p = feature_fct_->getFeatureJacobian( all_trajs[k][l] );

            for(int i=0;i<p.rows();i++) // For each via points
            {
                for(int j=0;j<p.cols();j++) // For each feature
                {
                    feature_jac_sum[k][l](j) += p(i,j);
                }
            }
        }
    }

    // cout << "return feature_jac_sum" << endl;
    return feature_jac_sum;
}

std::vector< std::vector<FeatureVect> > IocEvaluation::getFeatureCount( const std::vector< std::vector<Move3D::Trajectory> >& all_trajs )
{
    std::vector< std::vector<FeatureVect> > feature_count( all_trajs.size() );

    for(size_t k=0;k<all_trajs.size();k++) // For each demo
    {
        for(size_t l=0;l<all_trajs[k].size();l++) // For each sample
        {
            feature_count[k][l] = feature_fct_->getFeatureCount( all_trajs[k][l] );
        }
    }

    // cout << "return feature_jac_sum" << endl;
    return feature_count;
}

void IocEvaluation::saveDemoToMatlab()
{
    saveTrajToMatlab( demos_[0], 0 );
}

void IocEvaluation::saveContextToFile(const std::vector<Move3D::confPtr_t>& context) const
{

}

void IocEvaluation::saveNbDemoAndNbFeatures()
{
    Eigen::MatrixXd mat( Eigen::MatrixXd::Zero( 2, 1 ));
    mat(0, 0) = nb_demos_;
    mat(1, 0) = feature_fct_->getNumberOfFeatures();

    move3d_save_matrix_to_file( mat, std::string( "matlab/problem.txt" ) );
}

void IocEvaluation::saveTrajToMatlab(const Move3D::Trajectory& t, int id) const
{
    Move3D::Trajectory saved_traj(t);
    int nb_way_points=100;

    saved_traj.cutTrajInSmallLP(nb_way_points-1);

//    Eigen::MatrixXd mat = saved_traj.getEigenMatrix(6,7);
    Eigen::MatrixXd mat = saved_traj.getEigenMatrix( plangroup_->getActiveDofs() );

    // Save traj to file
    move3d_save_matrix_to_file( mat, std::string( "matlab/traj_" + num_to_string<int>(id) + ".txt" ) );
}

void IocEvaluation::saveToMatrixFile( const std::vector<FeatureVect>& demos, const std::vector< std::vector<FeatureVect> >& samples, std::string name )
{
    if( demos.empty() )
    {
        return;
    }

    int nb_samples = 0;
    for( int d=0;d<int(samples.size());d++)
        for( int i=0;i<int(samples[d].size());i++)
            nb_samples++;

    Eigen::MatrixXd mat(demos.size()+nb_samples,demos[0].size());

    // FIRST ADD DEMOS
    for( int d=0;d<int(demos.size());d++)
    {
        mat.row(d) = demos[d];
    }

    // THEN ADD EACH SET OF SAMPLES AS A ROW, STARTING WITH DEMO 1..N
    int k=0;
    for( int d=0;d<int(samples.size());d++)
    {
        for( int i=0;i<int(samples[d].size());i++)
        {
            mat.row( demos.size() + k++ ) = samples[d][i];
        }
    }

    std::stringstream ss;
    ss << tmp_data_folder_ << name << "_" << std::setw(3) << std::setfill( '0' ) << nb_samples_ << ".txt";

    //cout << "save samples to : " << feature_matrix_name_ << endl;
    move3d_save_matrix_to_file( mat, ss.str() );
}

void IocEvaluation::saveTrajectories(const std::vector<Move3D::Trajectory>& trajectories)
{
    std::stringstream ss;

    for( size_t i=0; i<trajectories.size(); i++ )
    {
        ss.str("");
        ss << "trajectory" << std::setw(3) << std::setfill( '0' ) << i << ".traj";

        trajectories[i].replaceP3dTraj();
        p3d_save_traj( (traj_folder_ + "/" + ss.str()).c_str(), robot_->getP3dRobotStruct()->tcur );

        cout << "save planner result " << i << " : " << ss.str() << endl;
    }
}

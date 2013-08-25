#include "HRICS_ioc.hpp"

#include "planEnvironment.hpp"
#include "API/Trajectory/trajectory.hpp"

using namespace HRICS;
using std::cout;
using std::endl;

IocTrajectory::IocTrajectory( int nb_var, int nb_joints )
{
    parameters_.resize( nb_joints );
    parameters_.resize( nb_joints );
    noise_.resize( nb_joints );
    noise_projected_.resize( nb_joints );
    parameters_noise_projected_.resize( nb_joints );
    state_costs_.resize( nb_var );
    control_costs_.resize( nb_joints );
    total_costs_.resize( nb_joints );
    cumulative_costs_.resize( nb_joints );
    probabilities_.resize( nb_joints );

    for (int d=0; d<nb_joints; ++d)
    {
        //cout << "num_parameters_[" << d << "] = " << num_parameters_[d] << endl;
        parameters_.push_back( Eigen::VectorXd::Zero( nb_var ) );
        noise_.push_back( Eigen::VectorXd::Zero( nb_var ) );
        noise_projected_.push_back( Eigen::VectorXd::Zero(nb_var) );

        control_costs_.push_back( Eigen::VectorXd::Zero(nb_var) );
        total_costs_.push_back( Eigen::VectorXd::Zero(nb_var) );
        cumulative_costs_.push_back( Eigen::VectorXd::Zero(nb_var) );
        probabilities_.push_back( Eigen::VectorXd::Zero(nb_var) );
    }
    state_costs_ = Eigen::VectorXd::Zero(nb_var);
    out_of_bounds_ = false;
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
    derivative_costs[0] = 0.0; //smoothness_cost_velocity_;
    derivative_costs[1] = 1.0; //smoothness_cost_acceleration_;
    derivative_costs[2] = 0.0; //smoothness_cost_jerk_;

    // initializes the policy
    policy_.initialize( num_vars_free_, num_joints_, 1.0, 0.0, derivative_costs );
}

//! Allocates the sampler
//! also initializes the control cost inverse structure
bool IocSampler::preAllocateMultivariateGaussianSampler()
{
    // invert the control costs, initialize noise generators:
    inv_control_costs_.clear();
    noise_generators_.clear();

    for (int j=0; j<num_joints_; ++j)
    {
        inv_control_costs_.push_back( control_costs_[j].inverse() );

        // TODO see of the noise generator needs to be
        // var free or var all
        MultivariateGaussian mvg( Eigen::VectorXd::Zero( num_vars_free_ ), inv_control_costs_[j] );
        noise_generators_.push_back( mvg );
    }

    return true;
}

Eigen::VectorXd IocSampler::sample( int joint )
{
    noise_generators_[ joint ].sample( tmp_noise_ );
    return tmp_noise_;
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
    num_demonstrations_ = 0;
    demonstrations_.clear();
    noise_stddev_.resize(num_joints_, PlanEnv->getDouble(PlanParam::trajOptimStdDev));
    sampler_.initialize();
}

void Ioc::addDemonstration( const Eigen::MatrixXd& demo )
{
    IocTrajectory t( num_vars_, num_joints_ );

    t.parameters_.resize( demo.rows() );

    for(int i=0;i<int(t.parameters_.size());i++)
    {
        t.parameters_[i] = demo.row( i );
    }

    demonstrations_.push_back( t );
    num_demonstrations_ = demonstrations_.size();

    for(int j=0;j<int(t.parameters_.size());j++)
    {
        cout << "demo (" << demonstrations_.size()-1 << ") : " << demonstrations_.back().parameters_[j].transpose() << endl;
    }
}

void Ioc::generateSamples( int nb_samples )
{
    samples_.resize( num_demonstrations_ );

    for (int d=0; d<num_demonstrations_; ++d)
    {
        samples_[d].resize( nb_samples );

        for (int ns=0; ns<int(samples_[d].size()); ++ns )
        {
            samples_[d][ns] = IocTrajectory( num_joints_, num_vars_ );

            for (int j=0; j<num_joints_; ++j)
            {
                samples_[d][ns].noise_[j] = noise_stddev_[j]*sampler_.sample( j );
                samples_[d][ns].parameters_[j] = demonstrations_[d].parameters_[j] + samples_[d][ns].noise_[j];
                cout << "sample (" << ns << ") : " << samples_[d][ns].parameters_[j].transpose() << endl;
            }
        }
    }

    addAllToDraw();
}

void Ioc::getSingleRollout(const std::vector<Eigen::VectorXd>& rollout, std::vector<confPtr_t>& traj)
{
    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;
    Robot* robot = planning_group_->robot_;

    traj.clear();

    for ( int j=0; j<num_vars_; ++j)
    {
        confPtr_t q = robot->getCurrentPos();

        for ( int i=0; i<num_joints_; ++i)
        {
            (*q)[joints[i].move3d_dof_index_] = rollout[i][j];
        }
        traj.push_back(q);
    }
}

void Ioc::addTrajectoryToDraw( const std::vector<Eigen::VectorXd>& rollout, int color )
{
    std::vector<confPtr_t> traj;
    getSingleRollout( rollout, traj );

    API::Trajectory T( planning_group_->robot_ );

    for ( int i=0; i<int(traj.size()); ++i )
        T.push_back( traj[i] );

    //T.print();
    T.setColor( color );
    trajToDraw.push_back( T );
}

void Ioc::addAllToDraw()
{
    trajToDraw.clear();

    cout << "Add rollouts to draw" << endl;

    for ( int d=0; d<int(demonstrations_.size()); ++d)
    {
        addTrajectoryToDraw( demonstrations_[d].parameters_, d );
    }

    for ( int d=0; d<int(demonstrations_.size()); ++d)
    {
        for ( int k=0; k<int(samples_[d].size()); ++k)
        {
            addTrajectoryToDraw( samples_[d][k].parameters_, k );
        }
    }
}

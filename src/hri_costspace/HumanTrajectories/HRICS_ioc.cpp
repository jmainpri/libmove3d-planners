#include "HRICS_ioc.hpp"
#include "HRICS_spheres.hpp"

#include "API/project.hpp"
#include "planEnvironment.hpp"
#include "API/Trajectory/trajectory.hpp"

using namespace HRICS;
using std::cout;
using std::endl;

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------

void HRICS_run_sphere_ioc()
{
    Robot* rob = global_Project->getActiveScene()->getActiveRobot();
    if (!rob) {
        cout << "robot not initialized in file "
             << __FILE__ << " ,  " << __func__ << endl;
        return;
    }

    confPtr_t q_init( rob->getInitPos() );
    confPtr_t q_goal( rob->getGoalPos() );
    if( *q_init == *q_goal )
    {
        cout << "init equal q_goal in file "
             << __FILE__ << " ,  " << __func__ << endl;
        return;
    }

    std::vector<int> planner_joints(1);
    planner_joints[0] = 1;
    ChompPlanningGroup* plangroup = new ChompPlanningGroup( rob, planner_joints );

    int nb_way_points = 20;

    std::vector<confPtr_t> confs(2);
    confs[0] = q_init;
    confs[1] = q_goal;
    API::Trajectory T( confs );
    T.cutTrajInSmallLP( nb_way_points-1 );
    T.replaceP3dTraj();
    Eigen::MatrixXd mat = T.getEigenMatrix(6,7);

    cout << "mat : " << mat << endl;

    HRICS::Ioc ioc( nb_way_points, plangroup );
    ioc.addDemonstration( mat );
    ioc.generateSamples( 10 );

    std::vector<API::Trajectory> samples = ioc.getSamples();
    for( int i=0;i<int(samples.size());i++)
    {
        FeatureVect vect = global_SphereCostFct->getFeatureCount( samples[i] );
        cout << "Feature(" << i << ") : " << vect.transpose() << endl;
    }
}

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------

IocTrajectory::IocTrajectory( int nb_joints, int nb_var  )
{
    cout << "nb_joints : " << nb_joints << endl;
    cout << "nb_var : " << nb_var << endl;

    parameters_.clear();
    noise_.clear();
    noise_projected_.clear();
    parameters_noise_projected_.clear();

    control_costs_.clear();
    total_costs_.clear();
    cumulative_costs_.clear();
    probabilities_.clear();

    for (int d=0; d<nb_joints; ++d)
    {
        parameters_.push_back( Eigen::VectorXd::Zero( nb_var ) );
        noise_projected_.push_back( Eigen::VectorXd::Zero( nb_var ) );
        noise_.push_back( Eigen::VectorXd::Zero( nb_var ) );
        parameters_noise_projected_.push_back( Eigen::VectorXd::Zero(nb_var) );

        control_costs_.push_back( Eigen::VectorXd::Zero(nb_var) );
        total_costs_.push_back( Eigen::VectorXd::Zero(nb_var) );
        cumulative_costs_.push_back( Eigen::VectorXd::Zero(nb_var) );
        probabilities_.push_back( Eigen::VectorXd::Zero(nb_var) );

        cout << "init parameters : " << parameters_.back().transpose() << endl;
    }
    state_costs_ = Eigen::VectorXd::Zero(nb_var);
    out_of_bounds_ = false;
}

API::Trajectory IocTrajectory::getMove3DTrajectory( const ChompPlanningGroup* p_g ) const
{
     Robot* rob = p_g->robot_;

    if( parameters_.empty() )
    {
        cout << "empty parameters" << endl;
        return API::Trajectory( rob );
    }

    const std::vector<ChompJoint>& joints = p_g->chomp_joints_;

    API::Trajectory T( rob );
    for ( int j=0; j<parameters_[0].size(); ++j )
    {
        confPtr_t q = rob->getCurrentPos();

        for ( int i=0; i<int(parameters_.size()); ++i )
            (*q)[joints[i].move3d_dof_index_] = parameters_[i][j];

         T.push_back( q );
    }

    return T;
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
    num_demonstrations_ = 0;
    demonstrations_.clear();
    noise_stddev_ = PlanEnv->getDouble(PlanParam::trajOptimStdDev);
    sampler_.initialize();
}

bool Ioc::addDemonstration( const Eigen::MatrixXd& demo )
{
    IocTrajectory t( num_joints_, num_vars_ );

    if( num_joints_ != demo.rows() )
    {
        cout << "Error in add demonstration" << endl;
        return false;
    }

    cout <<  num_joints_ << endl;
    cout << t.parameters_.size() << endl;

    for(int i=0;i<int(t.parameters_.size());i++)
    {
        cout << demo.row( i ) << endl;
        t.parameters_[i] = demo.row( i );
    }

    demonstrations_.push_back( t );
    num_demonstrations_ = demonstrations_.size();

    for(int j=0;j<int(t.parameters_.size());j++)
    {
        cout << "demo (" << demonstrations_.size()-1 << ") : " << demonstrations_.back().parameters_[j].transpose() << endl;
    }

    return true;
}

void Ioc::generateSamples( int nb_samples )
{
    samples_.resize( num_demonstrations_ );

    for (int d=0; d<num_demonstrations_; ++d)
    {
        samples_[d].resize( nb_samples );

        for (int ns=0; ns<int(samples_[d].size()); ++ns )
        {
            // Allocate trajectory
            samples_[d][ns] = IocTrajectory( num_joints_, num_vars_ );

            cout << "samples_[d][ns].parameters_[j] : " << samples_[d][ns].parameters_[0].transpose() << endl;

            // Sample noisy trajectory
            Eigen::MatrixXd noisy_traj = sampler_.sample(noise_stddev_);

            for (int j=0; j<num_joints_; ++j)
            {
                cout << demonstrations_[d].parameters_[j].size() << endl;
                cout << "demonstrations_[d].parameters_[j] : " << demonstrations_[d].parameters_[j].transpose() << endl;
                cout << "samples_[d][ns].noise_[j]  : " << samples_[d][ns].noise_[j].transpose() << endl;
                cout << "samples_[d][ns].parameters_[j] : " << samples_[d][ns].parameters_[j].transpose() << endl;
                cout << "noisy_traj.row(j) : " << noisy_traj.row(j) << endl;

                samples_[d][ns].noise_[j] = noisy_traj.row(j);
                samples_[d][ns].parameters_[j] = demonstrations_[d].parameters_[j] + samples_[d][ns].noise_[j];
                cout << "sample (" << ns << ") : " << samples_[d][ns].parameters_[j].transpose() << endl;
            }
        }
    }

    addAllToDraw();
}

std::vector<API::Trajectory> Ioc::getSamples()
{
    std::vector<API::Trajectory> samples;

    for ( int d=0; d<int(demonstrations_.size()); ++d)
    {
        for ( int k=0; k<int(samples_[d].size()); ++k)
        {
            samples.push_back( samples_[d][k].getMove3DTrajectory( planning_group_ ) );
        }
    }

    return samples;
}

void Ioc::addTrajectoryToDraw( const IocTrajectory& t, int color )
{
    API::Trajectory Move3DTraj = t.getMove3DTrajectory( planning_group_ );
    Move3DTraj.setColor( color );
    trajToDraw.push_back( Move3DTraj );
}

void Ioc::addAllToDraw()
{
    trajToDraw.clear();

    cout << "Add rollouts to draw" << endl;

    for ( int d=0; d<int(demonstrations_.size()); ++d)
    {
        addTrajectoryToDraw( demonstrations_[d], d );
    }

    for ( int d=0; d<int(demonstrations_.size()); ++d)
    {
        for ( int k=0; k<int(samples_[d].size()); ++k)
        {
            cout << "add sample : " << k << endl;
            addTrajectoryToDraw( samples_[d][k], k );
        }
    }
}

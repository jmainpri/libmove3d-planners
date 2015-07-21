#include "lampTrajectory.hpp"

#include "API/project.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "API/Graphic/drawCost.hpp"
#include "API/Graphic/drawModule.hpp"
#include "API/Device/generalik.hpp"

#include "utils/misc_functions.hpp"
#include "utils/NumsAndStrings.hpp"

#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planner/AStar/AStarPlanner.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"
#include "planner/cost_space.hpp"

#include "collision_space/collision_space_factory.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Util-pkg.h>

#include <iomanip>
#include <sstream>
#include <fstream>

using namespace Move3D;
using std::cout;
using std::endl;
using std::cin;


void lamp_sample_trajectories()
{
    Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();

    // Warning leaking
//    Move3D::AStarPlanner* planner = new Move3D::AStarPlanner(robot);
//    planner->set_pace( PlanEnv->getDouble(PlanParam::grid_pace) );
//    planner->init();

//    Move3D::confPtr_t q_init = robot->getInitPos();
//    Move3D::confPtr_t q_goal = robot->getGoalPos();
//    Move3D::Trajectory* traj = planner->computeRobotTrajectory( q_init, q_goal );

    /* TODO planar grid

    std::vector<double> env_size = global_Project->getActiveScene()->getBounds();
    env_size.resize(4);
    cout << "pace : " << pace_ << " meters" << endl;
    grid = new PlanGrid( robot,  pace, env_size );

    if( API_activeGrid != NULL )
        delete API_activeGrid;
    API_activeGrid = grid;

    */

//    Move3D::Trajectory traj( robot );
//    traj.loadFromFile( "/jim_local/Dropbox/move3d/move3d-launch/launch_files/3d_traj.txt" );
//    robot->setCurrentMove3DTraj( traj );

    Move3D::Trajectory traj( robot );
    traj.push_back( robot->getInitPos() );
    traj.push_back( robot->getGoalPos() );
    robot->setCurrentMove3DTraj( traj );

//    LampSampler sampler( robot );
//    sampler.initialize();
//    sampler.sampleTrajectories( 15, traj );
}

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------

VectorTrajectory::VectorTrajectory( int nb_dofs, int nb_var, double duration )
{
    state_costs_ = Eigen::VectorXd::Zero(nb_var);
    out_of_bounds_ = false;
    num_vars_free_ = nb_var;
    num_dofs_ = nb_dofs;

    // Set duration to external parameter if not equal 0.0
    use_time_ =  duration != 0.0 ? true : false ;
    duration_ = use_time_ ? duration : 0.0;
    discretization_ = duration_ / double( num_vars_free_ );

    trajectory_.resize( num_dofs_ * num_vars_free_ );
}

void VectorTrajectory::setFromMove3DTrajectory( const Move3D::Trajectory& T )
{
    const std::vector<Move3D::ChompDof>& joints = planning_group_->chomp_dofs_;

    trajectory_ = Eigen::VectorXd::Zero( num_vars_free_ * num_dofs_ );

    double delta = T.getParamMax() / double(num_vars_free_-1);
    double s = 0.0;

    for ( size_t i=0; i<num_vars_free_; ++i )
    {
        Move3D::confPtr_t q = T.configAtParam(s);
        s += delta;

        for ( size_t j=0; j<joints.size(); ++j )
            (*this)( i, j ) = (*q)[joints[j].move3d_dof_index_];
    }

    if( !use_time_ )
    {
        discretization_ = T.getParamMax() / double( num_vars_free_ );
    }
}

Move3D::confPtr_t VectorTrajectory::getMove3DConfiguration(int i) const
{
    const std::vector<Move3D::ChompDof>& joints = planning_group_->chomp_dofs_;

    Move3D::confPtr_t q = planning_group_->robot_->getInitPos();

    for ( size_t j=0; j<joints.size(); ++j )
        (*q)[ joints[j].move3d_dof_index_ ] = (*this)( i, j );

    return q;
}

Eigen::VectorXd VectorTrajectory::getTrajectoryPoint( int i ) const
{
    Eigen::VectorXd q(num_dofs_);

    for ( size_t j=0; j<num_dofs_; ++j )
        q(j) = (*this)( i, j );

//    TODO implement as segment...
//    q = trajectory_.segment(i,n);

    return q;
}

Move3D::Trajectory VectorTrajectory::getMove3DTrajectory() const
{
    Move3D::Robot* rob = planning_group_->robot_;

    if( trajectory_.size() == 0 )
    {
        cout << "empty parameters" << endl;
        return Move3D::Trajectory( rob );
    }

    const std::vector<Move3D::ChompDof>& joints = planning_group_->chomp_dofs_;

    // Create move3d trajectory
    Move3D::Trajectory T( rob );

    // cout << "discretization : " << discretization_ << endl;

    if( discretization_ != 0.0 ){
        T.setUseTimeParameter( true );
        T.setUseConstantTime( true );
        T.setDeltaTime( discretization_ );
    }

    for ( size_t i=0; i<num_vars_free_; ++i )
    {
        Move3D::confPtr_t q = rob->getInitPos();

        for ( size_t j=0; j<joints.size(); ++j )
            (*q)[joints[j].move3d_dof_index_] = (*this)( i, j );

        T.push_back( q->copy() );
    }

    return T;
}

//! Interpolates linearly two configurations
//! u = 0 -> a
//! u = 1 -> b
Eigen::VectorXd VectorTrajectory::interpolate( const Eigen::VectorXd& a, const Eigen::VectorXd& b, double u ) const
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

Eigen::VectorXd VectorTrajectory::getSraightLineTrajectory()
{
    // Set size
    Eigen::VectorXd straight_line = trajectory_;

    // Get number of points
//    int nb_points = num_vars_free_;
//    int dimension = num_dofs_;

    if( num_dofs_ < 1 )
    {
        cout << "Error : the trajectory only has one dimension" << endl;
        return Eigen::VectorXd::Zero(1);
    }

    if( num_vars_free_ <= 2 )
    {
        cout << "Warning : the trajectory is less or equal than 2 waypoints" << endl;
        return Eigen::VectorXd::Zero(1);;
    }

    Eigen::VectorXd a(num_dofs_);
    Eigen::VectorXd b(num_dofs_);
    Eigen::VectorXd c(num_dofs_);

//    for ( int j=0; j<num_dofs_; j++ ) {
//        a[j] = straight_line[0*int(num_dofs_) + int(j)];
//        b[j] = straight_line[(num_dofs_-1)*int(num_dofs_) + int(j)];
//    }

    std::vector<int> dofs = planning_group_->getActiveDofs();

//    for( int i=0; i<dofs.size() ; i++)
//    {
//        cout << "active dof " << i << " : " << dofs[i] << endl;
//    }

//    cout << "planning_group_ : " << planning_group_ << endl;
//    cout << " -- robot : " << planning_group_->robot_ << endl;

    a = planning_group_->robot_->getInitPos()->getEigenVector( dofs );
    b = planning_group_->robot_->getGoalPos()->getEigenVector( dofs );

//    cout << "a : " << a.transpose() << endl;
//    cout << "b : " << b.transpose() << endl;

    double delta = 1 / double(num_vars_free_-1);
    double s = 0.0;

    // Only fill the inside points of the trajectory
    for( int i=0; i<num_vars_free_; i++ )
    {
        c = interpolate( a, b, s );
        s += delta;

        for( int j=0; j<num_dofs_; j++ )
            straight_line[ i*int(num_dofs_) + int(j) ] = c[j];
    }

    // cout << "straight line : " << endl << straight_line.transpose() << endl;

    return straight_line;
}

void VectorTrajectory::setDofTrajectoryBlock(int dof, const Eigen::VectorXd traj)
{
    for( int i=0; i<num_vars_free_; i++)
    {
        (*this)(i, dof) = traj[i];
    }
}

void VectorTrajectory::addToDofTrajectoryBlock(int dof, const Eigen::VectorXd traj)
{
    for( int i=0; i<num_vars_free_; i++)
    {
        (*this)(i, dof) += traj[i];
    }
}

Eigen::VectorXd VectorTrajectory::getDofTrajectoryBlock( int dof ) const
{
    Eigen::VectorXd traj( num_vars_free_ );

    for( int i=0; i<num_vars_free_; i++)
    {
        traj[i] = (*this)(i, dof);
    }

    return traj;
}

bool VectorTrajectory::getParameters(std::vector<Eigen::VectorXd>& parameters) const
{
    if( int(parameters.size()) != num_dofs_ )
        return false;

    for(int dof=0; dof<num_dofs_; dof++)
        parameters[dof] = getDofTrajectoryBlock(dof);

    return true;
}

bool VectorTrajectory::getFreeParameters(std::vector<Eigen::VectorXd>& parameters) const
{
    if( int(parameters.size()) != num_dofs_ )
        return false;

    for(int dof=0; dof<num_dofs_; dof++)
        parameters[dof] = getDofTrajectoryBlock(dof); //.segment(start_index_, getNumFreePoints() /*-id_fixed_*/ ); // TODO see why

    return true;
}

void VectorTrajectory::getTrajectoryPointP3d(int traj_point, Eigen::VectorXd& jnt_array) const
{
    jnt_array.resize( num_dofs_ );

    for (int i=0; i<num_dofs_; i++)
    {
        jnt_array(i) = (*this)( traj_point, i );

        if ( std::isnan(jnt_array(i)) )
        {
            jnt_array(i) = 0;
        }
    }
}

int VectorTrajectory::getVectorIndex(int traj_point, int dof)
{
    return traj_point*num_dofs_ + dof;
}

double& VectorTrajectory::dof_cost(int traj_point, int dof)
{
    return dof_costs_[ traj_point*num_dofs_ + dof ];
}

double VectorTrajectory::dof_cost(int traj_point, int dof) const
{
    return dof_costs_[ traj_point*num_dofs_ + dof ];
}

double& VectorTrajectory::operator() (int traj_point, int dof)
{
    return trajectory_[ traj_point*num_dofs_ + dof ];
}

double VectorTrajectory::operator() (int traj_point, int dof) const
{
    return trajectory_[ traj_point*num_dofs_ + dof ];
}

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------


LampSampler::LampSampler( int num_var_free, int num_dofs ) : num_vars_free_(num_var_free), num_dofs_(num_dofs)
{

}

void LampSampler::initialize(const std::vector<double>& derivative_costs, int nb_points )
{
    cout << "initialize sampler" << endl;

    std::vector<int> joints;

    // Initializae scenario
    if( robot_model_->getName().find("PR2_ROBOT") != std::string::npos )
    {
        traj_optim_init_collision_spaces( traj_optim::Shelf, robot_model_ );
        joints = traj_optim_get_planner_joints();
    }
    else
    {
        joints = robot_model_->getActiveJointsIds();
        // joints.push_back(1);
    }

    // Initialize planning group
    planning_group_ = new ChompPlanningGroup( robot_model_, joints );
    planning_group_->collision_points_ = traj_optim_get_collision_points();

    num_vars_free_ = nb_points;
    num_dofs_ = planning_group_->chomp_dofs_.size();

    cout << "num_vars_free_ : " << num_vars_free_ << endl;
    cout << "num_dofs_ : " << num_dofs_ << endl;

    policy_.setPrintDebug( false );

    // initializes the policy
    policy_.initialize( num_vars_free_, num_dofs_, 1.0, 0.0, derivative_costs, planning_group_ );
    policy_.getControlCosts( control_costs_ );

    tmp_noise_ = Eigen::VectorXd::Zero( num_vars_free_*num_dofs_ );

    min_eigen_value_ = std::numeric_limits<double>::max();
    max_eigen_value_ = std::numeric_limits<double>::min();

    preAllocateMultivariateGaussianSampler();
}


void LampSampler::setOneDofBlockOfPrecisionMatrix( int dof, Eigen::MatrixXd matrix )
{
    for( int i=0; i<matrix.rows() ; i++)
        for( int j=0; j<matrix.cols() ; j++)
        {
            if( ( i*num_dofs_ + dof ) < precision_.rows() &&
                ( j*num_dofs_ + dof ) < precision_.cols() )
            {
                precision_( i*num_dofs_ + dof, j*num_dofs_ + dof ) += matrix( i , j );
            }
        }
}

void LampSampler::setTimeStepPrecisionMatrix( int time_step, Eigen::MatrixXd matrix )
{
    if(  ( num_dofs_ == matrix.rows() && num_dofs_ == matrix.cols() ) )
    {
        precision_.block( time_step*num_dofs_, time_step*num_dofs_, num_dofs_, num_dofs_ ) += matrix;
        // cout << "Add for time step : " << time_step << endl;
    }
//    else
//    {
//        cout << "hessian (" << matrix.rows() << " , " << matrix.cols() <<  ")" << endl;
//    }
}

Eigen::MatrixXd LampSampler::getOneDofBlockOfPrecisionMatrix(int dof)
{
    Eigen::MatrixXd matrix( num_vars_free_, num_vars_free_ );

    for( int i=0; i<matrix.rows() ; i++)
        for( int j=0; j<matrix.cols() ; j++)
        {
            matrix( i, j ) = precision_( i + dof, j + num_dofs_ * j );
        }

    return matrix;
}

Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> LampSampler::getConfigurationBlockOfPrecisionMatrix(int var)
{
    return precision_.block( var*num_dofs_, var*num_dofs_, num_dofs_, num_dofs_ );
}

bool LampSampler::preAllocateMultivariateGaussianSampler()
{
    // invert the control costs, initialize noise generators:
    // inv_control_costs_.clear();
//    noise_generators_.clear();

    // inv_control_costs_.push_back( control_costs_[j].inverse() );

    // move3d_save_matrix_to_file( inv_control_costs_[0], "../matlab/invcost_matrix.txt" );

    // Uncomment to print the precision Matrix
    // cout << endl << control_costs_[j] << endl;

    precision_ = Eigen::MatrixXd::Zero( num_vars_free_*num_dofs_, num_vars_free_*num_dofs_ );

    for( int j=0; j<num_dofs_; j++ )
    {
        setOneDofBlockOfPrecisionMatrix( j, control_costs_[j] );
    }

    control_ = precision_;

    // Get covariance matrix
    double sigma( std::pow( PlanEnv->getDouble(PlanParam::trajOptimStdDev), 2 ) );
    covariance_ = sigma * precision_.inverse();

    // TODO see of the noise generator needs to be var free or var all
    MultivariateGaussian mvg( Eigen::VectorXd::Zero( num_vars_free_* num_dofs_ ), covariance_ );
    noise_generators_.clear();
    noise_generators_.push_back( mvg );

//    covariance_ = precision_.inverse();

//    cout << "control cost : " << endl << control_costs_[0] << endl;
//    cout << "precision_ : " << endl << precision_<< endl;

//    move3d_save_matrix_to_file( precision_, "../matlab/cost_free.txt" );
//    move3d_save_matrix_to_file( covariance_, "../matlab/invcost_matrix.txt" );

    return true;
}

bool LampSampler::addHessianToPrecisionMatrix( const Move3D::VectorTrajectory& traj )
{
    // invert the control costs, initialize noise generators:
    // inv_control_costs_.clear();
//    noise_generators_.clear();

    for ( size_t i=0; i<num_vars_free_; ++i )
    {
        Move3D::confPtr_t q = traj.getMove3DConfiguration( i );

        // Eigen::MatrixXd H = PlanEnv->getDouble(PlanParam::hessian_factor) * std::pow( q->cost(), 4 )  * Eigen::MatrixXd::Identity( num_dofs_ , num_dofs_ );
//         PlanEnv->getDouble(PlanParam::hessian_factor) * global_costSpace->getHessian( *q, planning_group_->getActiveDofs() );

        Eigen::MatrixXd H = PlanEnv->getDouble(PlanParam::lamp_hessian_factor) * global_costSpace->getHessian( *q, planning_group_->getActiveDofs() );



        Eigen::EigenSolver<Eigen::MatrixXd> es(H);
//        Eigen::VectorXcd eigen_values = es.eigenvalues();

        Eigen::VectorXcd eig_val = es.eigenvalues();
//        Eigen::MatrixXcd D = eig_val.asDiagonal();
        Eigen::MatrixXcd V = es.eigenvectors();
;
        std::vector<bool> is_negative( eig_val.size(), false );

        for( int k=0; k<is_negative.size(); k++)
        {
            if( 0 > eig_val[k].real() )
                is_negative[k] = true;
//                eig_val[k].real() = 0;
        }

        for( int k=0; k<is_negative.size(); k++)
        {
            if( is_negative[k] )
            {
                Eigen::VectorXd Vk = V.col(k).real();
                H = H + Vk*Vk.transpose()*( -eig_val(k).real() );
            }
        }

//        H = ( V * D * V.inverse() ).real();

//        Eigen::VectorXd J =  global_costSpace->getJacobian( *q, planning_group_->getActiveDofs() );
//        cout << "J size : " << J.size() << endl;
//         Eigen::MatrixXd H2 = PlanEnv->getDouble(PlanParam::hessian_factor) * J * J.transpose() ;

//         cout << "H2 : "  << endl << H2 << endl;
//         H += H2;


        // Eigen::MatrixXd H = PlanEnv->getDouble(PlanParam::hessian_factor) * global_costSpace->getHessian( *q, planning_group_->getActiveDofs() );

        setTimeStepPrecisionMatrix( i, H );

//        Eigen::VectorXcd ev = H.eigenvalues();

//        for( int k=0; k<ev.size(); k++)
//        {
//            double real = ev[k].real();
//            if( min_eigen_value_ > real )
//                min_eigen_value_ = real;
//        }

//        if( min_eigen_value_ >  )


//        cout << "H is : " << endl << H << endl;
//        cout << "eig is : " << endl << H.eigenvalues().transpose().real() << endl;




//        cout << "A : " << A << endl;
//        cout << "min_eigen_value_ : " <<  min_eigen_value_ << endl;
    }

//    cout << "control cost : " << endl << control_costs_[0] << endl;
//    cout << "precision_ : " << endl << precision_<< endl;

//    precision_ -= min_eigen_value_ * Eigen::MatrixXd::Identity( num_vars_free_*num_dofs_ , num_vars_free_*num_dofs_ );

//    covariance_ = precision_.inverse();

//    move3d_save_matrix_to_file( precision_, "../matlab/cost_free.txt" );
//    move3d_save_matrix_to_file( covariance_, "../matlab/invcost_matrix.txt" );

    return true;
}

Eigen::VectorXd LampSampler::sample(double std_dev)
{
    Eigen::VectorXd traj( num_dofs_ * num_vars_free_ );
    noise_generators_[0].sample( tmp_noise_ );
    traj = std_dev*tmp_noise_;
    return traj;
}

std::vector<VectorTrajectory> LampSampler::sampleTrajectories( int nb_trajectories, const Move3D::VectorTrajectory& current_trajectory )
{
    global_trajToDraw.clear();

//    preAllocateMultivariateGaussianSampler();
//    addHessianToPrecisionMatrix( current_trajectory );

    std::vector<VectorTrajectory> trajectories( nb_trajectories );

    for( int i=0; i<trajectories.size(); i++)
    {
        Eigen::VectorXd noise = sample();

        trajectories[i] = current_trajectory;
        trajectories[i].trajectory_ += noise;

         cout << "norm of noise : " << noise.norm() << endl;

//        if( ENV.getBool(Env::drawTrajVector) && ENV.getBool(Env::drawTraj) )
//        {
//            global_trajToDraw.push_back( trajectories[i].getMove3DTrajectory() );
//            global_trajToDraw.back().setColor( i );
//        }
    }

    return trajectories;
}

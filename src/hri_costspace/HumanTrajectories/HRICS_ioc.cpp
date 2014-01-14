#include "HRICS_ioc.hpp"
#include "HRICS_spheres.hpp"
#include "HRICS_parameters.hpp"

#include "API/project.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planner/AStar/AStarPlanner.hpp"
#include "planEnvironment.hpp"
#include "plannerFunctions.hpp"

#include "utils/NumsAndStrings.hpp"

#include <libmove3d/include/Graphic-pkg.h>

#include <owlqn/OWLQN.h>
#include <owlqn/leastSquares.h>
#include <owlqn/logreg.h>

#include <iomanip>
#include <sstream>

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

    if( global_SphereCostFct == NULL )
    {
        HRICS_init_sphere_cost();
    }

    bool single_iteration = HriEnv->getBool(HricsParam::ioc_single_iteration);
    int nb_iterations = HriEnv->getInt(HricsParam::ioc_sample_iteration);

    int nb_demos = 1;
    int nb_sampling_phase = 20;
    int min_samples = 2;
    int max_samples = 100;

    bool StopRun = false;

    enum phase_t { generate, sample, compare } phase;

    cout << "ioc phase : " << HriEnv->getInt(HricsParam::ioc_phase) << endl;

    switch( HriEnv->getInt(HricsParam::ioc_phase) )
    {
    case 0:
        phase = generate;
        cout << "SET PHASE GENERATE" << endl;
        break;
    case 1:
        phase = sample;
        cout << "SET PHASE SAMPLE" << endl;
        break;
    case 2:
        phase = compare;
        cout << "SET PHASE COMPARE" << endl;
        break;
    default: cout << "Error : phase not defined!!!" << endl; return;
    }

    std::vector<Eigen::VectorXd> results;

    int iteration = 0;

    for(int i=0; i<nb_sampling_phase && !StopRun; i++)
    {
        // iteration = i; // 2, 5, 30, 50

        if( single_iteration )
            iteration = nb_iterations;
        else
            iteration = i;

        cout << "------------------------------" << endl;
        cout << " RUN : " << iteration << endl;
        cout << "------------------------------" << endl;

        // interpolation for the number of sampling phase
        int nb_samples = min_samples + double(iteration)*(max_samples-min_samples)/double(nb_sampling_phase-1);

        cout << "NB SAMPLES : " << nb_samples << endl;

        IocEvaluation eval( rob, nb_demos, nb_samples );

        switch( phase )
        {
        case generate:
            eval.generateDemonstrations();
            g3d_draw_allwin_active();
            break;

        case sample:
            eval.loadDemonstrations();
            // eval.runLearning();
            eval.runSampling();
            g3d_draw_allwin_active();
            break;

        case compare:
            results.push_back( eval.compareDemosAndPlanned() );
            g3d_draw_allwin_active();
            break;
        }

        if( single_iteration )
            break;

        if ( PlanEnv->getBool(PlanParam::stopPlanner) ) {
            StopRun = true;
        }
    }

    if( !results.empty() )
    {
        Eigen::MatrixXd mat( results.size(), results[0].size() );
        for( int i=0;i<mat.rows();i++)
        {
            mat.row(i) = results[i];
        }

        std::ofstream file_traj("matlab/data/result.txt");
        if (file_traj.is_open())
            file_traj << mat << '\n';
        file_traj.close();
    }

//    eval.loadDemonstrations();
//    eval.runLearning();
//    eval.loadWeightVector();
//    eval.generateDemonstrations();
//    eval.loadDemonstrations();
//    eval.saveDemoToMatlab();

//    eval.compareDemosAndPlanned();

//    global_SphereCostFct->produceCostMap();
}


// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------

IocEvaluation::IocEvaluation(Robot* rob, int nb_demos, int nb_samples) : robot_(rob)
{
    nb_demos_ = nb_demos;
    nb_samples_ = nb_samples;
    nb_way_points_ = 100;
    folder_ = "/home/jmainpri/workspace/move3d/assets/IOC/TRAJECTORIES/";

    std::vector<int> aj(1); aj[0] = 1;
    active_joints_ = aj;

    std::stringstream ss;
    ss << "matlab/data/spheres_features_" << std::setw(3) << std::setfill( '0' ) << nb_samples_ << ".txt";
    feature_matrix_name_ = ss.str();

    plangroup_ = new ChompPlanningGroup( robot_, active_joints_ );

    // Active dofs are set when the planning group is created
    smoothness_fct_ = new TrajectorySmoothness;
    smoothness_fct_->setActiveDofs( plangroup_->getActiveDofs() );

    if( global_SphereCostFct != NULL )
    {
        StackedFeatures* fct = new StackedFeatures;
        //fct->addFeatureFunction( smoothness_fct_ );
        fct->addFeatureFunction( global_SphereCostFct );
        fct->setWeights( global_SphereCostFct->getWeights() );

        feature_fct_ = fct;

        nb_weights_ = feature_fct_->getNumberOfFeatures();
        original_vect_ = feature_fct_->getWeights();

        cout << "original_vect : " << endl;
        feature_fct_->printWeights();

        // Save costmap to matlab with original weights
        global_SphereCostFct->produceCostMap();
        global_SphereCostFct->produceDerivativeFeatureCostMap();
    }
}

API::Trajectory IocEvaluation::planMotionRRT()
{
    try
    {
        p3d_run_rrt( robot_->getRobotStruct() );

        if( !ENV.getBool(Env::drawDisabled) ) {
            g3d_draw_allwin_active();
        }
    }
    catch (std::string str)
    {
        std::cerr << "Exeption in run rrt : " << endl;
        std::cerr << str << endl;
    }
    catch (...)
    {
        std::cerr << "Exeption in run qt_runDiffusion" << endl;
    }

    return p3d_get_last_trajectory();
}

API::Trajectory IocEvaluation::planAStar()
{
    // Warning leaking
    AStarPlanner planner( robot_ );
    planner.set_pace( PlanEnv->getDouble(PlanParam::grid_pace) );
    planner.init();

    confPtr_t q_init = robot_->getInitPos();
    confPtr_t q_goal = robot_->getGoalPos();

    API::Trajectory* traj = planner.computeRobotTrajectory( q_init, q_goal );

    if( traj != NULL )
    {
        cout << "AStar planning OK" << endl;
    }
    else {
        cout << "Error in run AStar planner" << endl;
    }

    return *traj;
}

API::Trajectory IocEvaluation::planMotionStomp()
{
    traj_optim_runStomp(0);
    return global_optimizer->getBestTraj();
}

void IocEvaluation::generateDemonstrations()
{
    std::vector<API::Trajectory> demos;

    for(int i=0;i<nb_demos_;i++)
    {
        //demos.push_back( planMotionStomp() );
        demos.push_back( planAStar() );
    }

    for(int i=0;i<int(demos.size());i++)
    {
        demos[i].replaceP3dTraj();

        // Set file names
        std::stringstream ss;
        ss << "trajectory" << std::setw(3) << std::setfill( '0' ) << i << ".traj";
        std::string filename = folder_ + ss.str();

        p3d_save_traj( filename.c_str(), robot_->getRobotStruct()->tcur );
        cout << "save demo " << i << " : " << ss.str() << endl;

        saveTrajToMatlab(demos[i],i);
        cout << "save traj to matlab format!!!!" << endl;
    }
}

void IocEvaluation::loadDemonstrations()
{
    demos_.clear();
    global_trajToDraw.clear();

    std::stringstream ss;

    for(int i=0;i<nb_demos_;i++)
    {
        ss.str(""); // clear stream
        ss << "trajectory" << std::setw(3) << std::setfill( '0' ) << i << ".traj";

        if ( !p3d_read_traj( ( folder_ + ss.str() ).c_str()) )
        {
            cout << "could not load trajectory" << endl;
            return;
        }

        API::Trajectory T( robot_, (p3d_traj*)p3d_get_desc_curid(P3D_TRAJ) );
        T.cutTrajInSmallLP(nb_way_points_-1);
        T.setColor( i%8 );
        cout << "color : " << i%8 << endl;
        demos_.push_back(T);
        global_trajToDraw.push_back(T);
    }
}

void IocEvaluation::loadWeightVector()
{
    cout << "Load Weight Vector" << endl;

    learned_vect_ = Eigen::VectorXd::Zero( nb_weights_ );

    // Load vector from file
    std::stringstream ss;
    ss << "matlab/data/spheres_weights_" << std::setw(3) << std::setfill( '0' ) << nb_samples_ << ".txt";

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
    file.close();

    // cout << " w : " << learned_vect_.transpose() << endl;
}

void IocEvaluation::runSampling()
{
    HRICS::Ioc ioc( nb_way_points_, plangroup_ );

    // Get features of demos
    std::vector<FeatureVect> phi_demo(demos_.size());
    for(int i=0;i<int(demos_.size());i++)
    {
        demos_[i].cutTrajInSmallLP( nb_way_points_-1 );
        FeatureVect phi = feature_fct_->getFeatureCount( demos_[i] );
        //cout << "Feature Demo : " << phi.transpose() << endl;
        ioc.addDemonstration( demos_[i].getEigenMatrix(6,7) );
        phi_demo[i] = phi;
    }

    // Get features of samples
    ioc.generateSamples( nb_samples_ );
    std::vector< std::vector<API::Trajectory> > samples = ioc.getSamples();
    std::vector< std::vector<FeatureVect> > phi_k( samples.size() );
    for( int d=0;d<int(samples.size());d++)
    {
        for( int i=0;i<int(samples[d].size());i++)
        {
            phi_k[d].push_back( feature_fct_->getFeatureCount( samples[d][i] ) );
            // cout << "Sample(" << d << "," <<  i << ") : " << phi_k[d][i].transpose() << endl;
            // cout << "Smoothness(" << d << "," <<  i << ") : " << phi_k[d][i][0] << endl;
        }
    }

    saveToMatrix( phi_demo, phi_k );

    // checkStartAndGoal( samples );

    // Only for plannar robot
    // global_SphereCostFct->produceCostMap();
    // trajToMatlab(T);
}

bool IocEvaluation::checkStartAndGoal( const std::vector< std::vector<API::Trajectory> >& samples ) const
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

    loadWeightVector();

    for( int i=0;i<costs_demo.size();i++)
    {
        setLearnedWeights();
//        learned_[i] = planMotionStomp();
//        learned_[i] = planMotionRRT();
        learned_[i] = planAStar();

        g3d_draw_allwin_active();

        setOriginalWeights();

        learned_[i].resetCostComputed();

        costs_learned[i] = feature_fct_->costTraj( learned_[i] );
    }

    double mean_demo = costs_demo.mean();
    double sq_sum_demo = costs_demo.transpose()*costs_demo;
    double stdev_demo = std::sqrt( sq_sum_demo / double(costs_demo.size()) - (mean_demo * mean_demo) );

    double mean_learned = costs_learned.mean();
    double sq_sum_learned = costs_learned.transpose()*costs_learned;
    double stdev_learned = std::sqrt( (sq_sum_learned / double(costs_learned.size())) - (mean_learned * mean_learned) );

    cout << ( costs_demo    ).transpose() << endl;
    cout << ( costs_learned ).transpose() << endl;

    cout << "mean_demo : " << mean_demo << " , stdev_demo : " << stdev_demo << endl;
    cout << "mean_learned : " << mean_learned << " , stdev_learned : " << stdev_learned << endl;

    cout << "mean diff : " << mean_learned - mean_demo << " , stdev diff : " << stdev_learned - stdev_demo  << endl;

    Eigen::VectorXd result(3);
    result[0] = mean_demo;
    result[1] = mean_learned;
    result[2] = mean_learned - mean_demo;
    return result;
}

void IocEvaluation::saveDemoToMatlab()
{
    saveTrajToMatlab( demos_[0], 0 );
}

void IocEvaluation::saveTrajToMatlab(const API::Trajectory& t, int id) const
{
    API::Trajectory saved_traj(t);
    int nb_way_points=100;
    saved_traj.cutTrajInSmallLP(nb_way_points-1);

    Eigen::MatrixXd mat = saved_traj.getEigenMatrix(6,7);

    // Save traj to file
    std::ofstream file_traj( std::string( "matlab/traj_" + num_to_string<int>(id) + ".txt" ).c_str() );
    if (file_traj.is_open())
        file_traj << mat << '\n';
    file_traj.close();
}

void IocEvaluation::saveToMatrix( const std::vector<FeatureVect>& demos, const std::vector< std::vector<FeatureVect> >& samples )
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

    for( int d=0;d<int(demos.size());d++)
    {
        mat.row(d) = demos[d];
    }

    int k=0;
    for( int d=0;d<int(samples.size());d++)
    {
        for( int i=0;i<int(samples[d].size());i++)
        {
            mat.row( demos.size() + k++ ) = samples[d][i];
        }
    }

    // Save traj to file
    std::ofstream file( feature_matrix_name_.c_str() );
    if (file.is_open())
        file << mat << '\n';
    file.close();

    cout << "save samples to : " << feature_matrix_name_ << endl;
}

bool IocEvaluation::loadFromMatrix( std::vector<FeatureVect>& demos, std::vector< std::vector<FeatureVect> >& samples )
{
    if( demos.empty() )
    {
        return false;
    }

    Eigen::MatrixXd mat(demos.size()+samples.size(),demos[0].size());

    for( int d=0;d<int(demos.size());d++)
    {
        demos[d] = mat.row(d);
    }

    int k=0;
    for( int d=0;d<int(samples.size());d++)
    {
        for( int i=0;i<int(samples[d].size());i++)
        {
            samples[d][i] = mat.row( demos.size() + k++ ) ;
        }
    }
    return true;
}

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------

IocTrajectory::IocTrajectory( int nb_joints, int nb_var  )
{
    //    cout << "nb_joints : " << nb_joints << endl;
    //    cout << "nb_var : " << nb_var << endl;

    nominal_parameters_.clear();
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
        nominal_parameters_.push_back( Eigen::VectorXd::Zero( nb_var ) );
        parameters_.push_back( Eigen::VectorXd::Zero( nb_var ) );
        noise_projected_.push_back( Eigen::VectorXd::Zero( nb_var ) );
        noise_.push_back( Eigen::VectorXd::Zero( nb_var ) );
        parameters_noise_projected_.push_back( Eigen::VectorXd::Zero(nb_var) );

        control_costs_.push_back( Eigen::VectorXd::Zero(nb_var) );
        total_costs_.push_back( Eigen::VectorXd::Zero(nb_var) );
        cumulative_costs_.push_back( Eigen::VectorXd::Zero(nb_var) );
        probabilities_.push_back( Eigen::VectorXd::Zero(nb_var) );

        //        cout << "init parameters : " << parameters_.back().transpose() << endl;
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
    noise_stddev_ = 10*PlanEnv->getDouble(PlanParam::trajOptimStdDev);
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

//    cout <<  num_joints_ << endl;
//    cout << t.parameters_.size() << endl;

    for(int i=0;i<int(t.parameters_.size());i++)
    {
//        cout << demo.row( i ) << endl;
        t.parameters_[i] = demo.row( i );
    }

    demonstrations_.push_back( t );
    num_demonstrations_ = demonstrations_.size();

    // TODO can be commented off
    // Set straight line for sampling
    demonstrations_.back().setSraightLineTrajectory();

//    for(int j=0;j<int(t.parameters_.size());j++)
//    {
//        cout << "demo (" << demonstrations_.size()-1 << ") : " << demonstrations_.back().parameters_[j].transpose() << endl;
//    }

    return true;
}

bool Ioc::jointLimits( IocTrajectory& traj ) const
{
    for( int j=0;j<num_joints_;j++)
    {
        double coeff = 1.0;

        double j_max = planning_group_->chomp_joints_[j].joint_limit_max_;
        double j_min = planning_group_->chomp_joints_[j].joint_limit_min_;

        for( int i=0;i<num_vars_;i++)
        {
            while( traj.parameters_[j][i] < j_min || traj.parameters_[j][i] > j_max )
            {
                coeff *= 0.90; // 90 percent
                traj.noise_[j] *= coeff;
                traj.parameters_[j] = traj.nominal_parameters_[j] +  traj.noise_[j];
            }

            // cout << "Joint limit coefficient : " << coeff << endl;
        }

        // Set the start and end values constant
        traj.parameters_[j].start(1) = traj.nominal_parameters_[j].start(1);
        traj.parameters_[j].end(1) = traj.nominal_parameters_[j].end(1);
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
            // cout << "samples_[d][ns].parameters_[j] : " << samples_[d][ns].parameters_[0].transpose() << endl;

            // Sample noisy trajectory
            Eigen::MatrixXd noisy_traj = sampler_.sample(noise_stddev_);

            for (int j=0; j<num_joints_; ++j)
            {
                // Change to generate samples around demonstration
                //samples_[d][ns].nominal_parameters_[j] = demonstrations_[d].parameters_[j];
                samples_[d][ns].nominal_parameters_[j] = demonstrations_[d].straight_line_[j];
                samples_[d][ns].noise_[j] = noisy_traj.row(j);
                samples_[d][ns].parameters_[j] = samples_[d][ns].nominal_parameters_[j] + samples_[d][ns].noise_[j];
                //cout << "sample (" << ns << ") : " << samples_[d][ns].parameters_[j].transpose() << endl;
                //cout << samples_[d][ns].noise_[j].transpose() << endl;
            }

            jointLimits( samples_[d][ns] );
        }
    }

    addAllToDraw();
}

std::vector< std::vector<API::Trajectory> > Ioc::getSamples()
{
    std::vector< std::vector<API::Trajectory> > samples(demonstrations_.size());

    for ( int d=0; d<int(demonstrations_.size()); ++d)
    {
        for ( int k=0; k<int(samples_[d].size()); ++k)
        {
            samples[d].push_back( samples_[d][k].getMove3DTrajectory( planning_group_ ) );
        }
    }

    return samples;
}

void Ioc::addTrajectoryToDraw( const IocTrajectory& t, int color )
{
    API::Trajectory T = t.getMove3DTrajectory( planning_group_ );
    T.setColor( color );
    global_trajToDraw.push_back( T );
}

void Ioc::addAllToDraw()
{
    global_trajToDraw.clear();
    //cout << "Add rollouts to draw" << endl;

    for ( int d=0; d<int(demonstrations_.size()); ++d)
    {
        addTrajectoryToDraw( demonstrations_[d], d%8 );
    }

    for ( int d=0; d<int(demonstrations_.size()); ++d)
    {
        for ( int k=0; k<int(samples_[d].size()); ++k)
        {
            //cout << "add sample : " << k << endl;
            addTrajectoryToDraw( samples_[d][k], k%8 );
        }
    }
}

//////////////////////////////////////////////////////
// IOC objective
//////////////////////////////////////////////////////
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
        cout << "error in size : " << __func__ << endl;
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

Eigen::VectorXd Ioc::solve( const std::vector<Eigen::VectorXd>& phi_demo, const std::vector< std::vector<Eigen::VectorXd> >& phi_k )
{
    if( phi_demo.size() < 1 )
    {
        cout << "no demo passed in Ioc solver" << endl;
        return Eigen::VectorXd();
    }

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
}

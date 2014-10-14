#include "HRICS_human_simulator.hpp"
#include "HRICS_human_cost_space.hpp"
#include "HRICS_play_motion.hpp"
#include "HRICS_parameters.hpp"
#include "HRICS_play_motion.hpp"
#include "HRICS_gest_parameters.hpp"
#include "HRICS_human_prediction_cost_space.hpp"

#include "API/project.hpp"
#include "API/Graphic/drawModule.hpp"

#include "planner/cost_space.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/run_parallel_stomp.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/planEnvironment.hpp"

#include "collision_space/collision_space.hpp"
#include "collision_space/collision_space_factory.hpp"

#include <boost/bind.hpp>
#include <iomanip>

#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Collision-pkg.h>

#include <iomanip>

using namespace HRICS;
using std::cout;
using std::endl;
using std::cin;

HRICS::HumanTrajSimulator* global_ht_simulator = NULL;
HRICS::HumanTrajCostSpace* global_ht_cost_space = NULL;

extern bool hrics_set_baseline;
extern bool hrics_one_iteration;

//----------------------------------------------------------------------
//----------------------------------------------------------------------

bool HRICS_init_human_trajectory_cost()
{
    cout << "---------------------------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;
    cout << "---------------------------------------------" << endl;

    if( global_ht_cost_space == NULL )
    {
        Move3D::Scene* sce = Move3D::global_Project->getActiveScene();
        Move3D::Robot* human1 = sce->getRobotByName( "HERAKLES_HUMAN1" );
        Move3D::Robot* human2 = sce->getRobotByName( "HERAKLES_HUMAN2" );
        if( human1 == NULL || human2 == NULL )
        {
            cout << "No humans HERAKLES in the the scene" << endl;
            return false;
        }

        bool load_kinect_motions = true;
        if( load_kinect_motions )
        {
            global_motionRecorders.push_back( new HRICS::RecordMotion( human1 ) );
            global_motionRecorders.push_back( new HRICS::RecordMotion( human2 ) );

            // std::string foldername = "/home/jmainpri/workspace/move3d/libmove3d/statFiles/collaboration/recorded_motion_01_09_13";
            // Set this bool to false is you want to print file names as they are loaded
            //        bool quiet = true;
            //        global_motionRecorders[0]->loadCSVFolder( foldername + "/human0", quiet );
            //        global_motionRecorders[1]->loadCSVFolder( foldername + "/human1", quiet );

            if( human1->getJoint( "rShoulderZ" ) != NULL )
            {
                global_motionRecorders[0]->useOpenRAVEFormat( true );
                global_motionRecorders[1]->useOpenRAVEFormat( true );

                // std::string foldername = "/home/jmainpri/Dropbox/move3d/move3d-launch/matlab/quan_motion";
                // motion_t traj1 = global_motionRecorders[0]->loadFromCSV( foldername + "/[1016#-#1112]#motion_saved_00000_00000.csv" );
                // motion_t traj2 = global_motionRecorders[1]->loadFromCSV( foldername + "/[1016#-#1112]#motion_saved_00001_00000.csv" );
                // global_motionRecorders[0]->storeMotion( traj1 );
                // global_motionRecorders[1]->storeMotion( traj2 );

                //        std::string foldername = "/home/jmainpri/Dropbox/move3d/move3d-launch/matlab/kinect_good_motions/good_lib/";
                std::string foldername = "/home/jmainpri/Dropbox/move3d/move3d-launch/matlab/kinect_good_motions/human_one_good/";

                bool quiet = true;
                global_motionRecorders[0]->loadCSVFolder( foldername + "human_one/", quiet, +0.5 );
                global_motionRecorders[1]->loadCSVFolder( foldername + "human_two/", quiet, -0.5 );
            }
            else
            {
                cout << "LOAD BIO MOTIONS" << endl;
                global_motionRecorders[0]->useBioFormat( true );
                global_motionRecorders[1]->useBioFormat( true );

                // ORIGINAL
//                std::string foldername = "/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/python_module/bioik/tmp";
//                motion_t traj1 = global_motionRecorders[0]->loadFromCSV( foldername + "/[1460-1620]_human2_.csv" );
//                motion_t traj2 = global_motionRecorders[1]->loadFromCSV( foldername + "/[1460-1620]_human1_.csv" );
//                global_motionRecorders[0]->storeMotion( traj1, "[1460-1620]_human2_.csv" );
//                global_motionRecorders[1]->storeMotion( traj2, "[1460-1620]_human1_.csv" );

                std::string foldername = "/home/jmainpri/catkin_ws_hrics/src/hrics-or-rafi/python_module/bioik/ten_motions_last/";
                bool quiet = true;
                global_motionRecorders[0]->loadCSVFolder( foldername + "human_two/", quiet, -1.5 );
                global_motionRecorders[1]->loadCSVFolder( foldername + "human_one/", quiet, +1.5 );

                cout << "Stored motion names : " << endl;
                for( int i=0; i<global_motionRecorders[0]->getStoredMotions().size();i++)
                {
                    cout << global_motionRecorders[0]->getStoredMotionName(i) << endl;
                    cout << global_motionRecorders[1]->getStoredMotionName(i) << endl;
                }
            }
        }

        cout << "create human traj cost space" << endl;

        // SET BASELINE HERE
        hrics_set_baseline = false;
        hrics_one_iteration = false;
        PlanEnv->setDouble( PlanParam::trajOptimSmoothWeight, hrics_set_baseline ? 100. : 1.0000 );

        // Workspace Occupancy costspace
        std::vector<double> size = Move3D::global_Project->getActiveScene()->getBounds();
        HRICS::WorkspaceOccupancyGrid* occupancyGrid = new WorkspaceOccupancyGrid( human1, 0.03, size );
        global_humanPredictionCostSpace = new HRICS::HumanPredictionCostSpace( human2, occupancyGrid );

        // Human trajectory costspace
        global_ht_cost_space = new HRICS::HumanTrajCostSpace( human2, human1 );

        // Set active joints and joint bounds
        global_ht_simulator = new HRICS::HumanTrajSimulator( global_ht_cost_space );
        // Set the sampling bounds for the human simulator
        global_ht_simulator->setPelvisBoundsByUser( HriEnv->getBool(HricsParam::ioc_user_set_pelvis_bounds) );
        global_ht_simulator->init();

        // Define cost functions
        cout << " add cost : " << "costHumanTrajectoryCost" << endl;
        Move3D::global_costSpace->addCost( "costHumanWorkspaceOccupancy", boost::bind( &HRICS::HumanPredictionCostSpace::getCurrentOccupationCost, global_humanPredictionCostSpace, _1) );
        Move3D::global_costSpace->addCost( "costHumanTrajectoryCost", boost::bind( &HumanTrajCostSpace::cost, global_ht_cost_space, _1) );
    }

    ENV.setBool( Env::isCostSpace, true );
    Move3D::global_costSpace->setCost( "costHumanTrajectoryCost" );

    if( !global_ht_cost_space->initCollisionSpace() )
        cout << "Error : could not init collision space" << endl;

    cout << " global_ht_cost_space : " << global_ht_cost_space << endl;
    global_activeFeatureFunction = global_ht_cost_space;

    return true;
}

void HRICS_run_human_planning()
{
    HRICS_init_human_trajectory_cost();

    HumanTrajSimulator sim( global_ht_cost_space );
    sim.init();
    sim.run();
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Human Trajectory Cost Space

HumanTrajCostSpace::HumanTrajCostSpace( Move3D::Robot* active, Move3D::Robot* passive ) :
    human_active_(active),
    human_passive_(passive),
    smoothness_feat_( active ),
    dist_feat_( active, passive ),
    visi_feat_(active, passive),
    musc_feat_( active ),
    collision_feat_( active )
{
    cout << "---------------------------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;

    nb_way_points_ = 20;

    use_bio_models_ = ( human_active_->getJoint( "rShoulderZ" ) == NULL );

//    w_ = getFeatures( *human_active_->getCurrentPos() );
//    smoothness_feat_.setActiveDoFs( acti);

    active_dofs_ = std::vector<int>(1,1);

    length_feat_.setActiveDoFs( active_dofs_ );
//    length_feat_.setWeights( 0.8 * Move3D::WeightVect::Ones( length_feat_.getNumberOfFeatures() ) );

    dist_feat_.setActiveDoFs( active_dofs_ );
    dist_feat_.setWeights( w_distance_16 );

    collision_feat_.setActiveDoFs( active_dofs_ );
//    collision_feat_.setWeights( Move3D::WeightVect::Ones( collision_feat_.getNumberOfFeatures() ) );

    visi_feat_.setActiveDoFs( active_dofs_ );
    visi_feat_.setWeights( Move3D::WeightVect::Ones( visi_feat_.getNumberOfFeatures() ) );

    if(!use_bio_models_)
    {
        musc_feat_.setActiveDoFs( active_dofs_ );
        musc_feat_.setWeights( Move3D::WeightVect::Ones( musc_feat_.getNumberOfFeatures() ) );
    }

    smoothness_feat_.setActiveDoFs( active_dofs_ );

    if( !hrics_set_baseline )
    {
        smoothness_feat_.setWeights( Move3D::WeightVect::Ones( smoothness_feat_.getNumberOfFeatures() ) );

        if(!addFeatureFunction( &smoothness_feat_ ) ){
            cout << "Error adding feature smoothness" << endl;
        }
    }
    if(!addFeatureFunction( &dist_feat_ ) ){
        cout << "Error adding feature distance feature" << endl;
    }

//    if(!addFeatureFunction( &length_feat_ ) ){
//        cout << "Error adding feature length" << endl;
//    }

//    if(!addFeatureFunction( &collision_feat_ ) ){
//        cout << "Error adding feature distance collision" << endl;
//    }

//    if(!addFeatureFunction( &visi_feat_ )){
//        cout << "Error adding feature visbility feature" << endl;
//    }
//    if(!addFeatureFunction( &musc_feat_ )){
//        cout << "Error adding feature musculoskeletal feature" << endl;
//    }

    w_ = getWeights();

    cout << "w_ = " << w_.transpose() << endl;

    setAllFeaturesActive();

    std::vector<std::string> active_features_names;
//    active_features_names.push_back("Smoothness");
    active_features_names.push_back("Collision");
    active_features_names.push_back("Length");
    active_features_names.push_back("Distance");
    active_features_names.push_back("Visibility");
    active_features_names.push_back("Musculoskeletal");

    setActiveFeatures( active_features_names );

    printInfo();

    cout << "---------------------------------------------" << endl;
}

Move3D::FeatureVect HumanTrajCostSpace::normalizing_by_sampling()
{
    cout << "---------------------------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;

    int nb_samples = 10000;

    std::vector<std::string> active_features_names;
//    active_features_names.push_back("Smoothness");
//    active_features_names.push_back("Length");
    active_features_names.push_back("Distance");
    active_features_names.push_back("Visibility");
    active_features_names.push_back("Musculoskeletal");

    setActiveFeatures( active_features_names );

    Move3D::FeatureVect phi( Move3D::FeatureVect::Zero(nb_features_) );
    Move3D::FeatureVect phi_sum( Move3D::FeatureVect::Zero(nb_features_) );
    Move3D::FeatureVect phi_max( Move3D::FeatureVect::Zero(nb_features_) );
    Move3D::FeatureVect phi_min( std::numeric_limits<double>::max() * Move3D::FeatureVect::Ones(nb_features_) );

    for (int i=0; i<nb_samples; i++)
    {
        Move3D::confPtr_t q = human_active_->shoot();
        human_active_->setAndUpdate(*q);
        phi = getFeatures(*q);
        phi_sum += phi;

        for( int j=0; j<phi.size(); j++ )
        {
            if( phi[j] < phi_min[j] )
                phi_min[j] = phi[j];

            if( phi[j] > phi_max[j] )
                phi_max[j] = phi[j];
        }
//        g3d_draw_allwin_active();
    }

    phi_sum /= double(nb_samples);

    cout << std::scientific;

//    for (int i=0; i<nb_features_; i++) {
//        cout << "phi[" << i << "] : " << phi_sum[i] << endl;
//    }

    cout << "MAX VALUES : " << endl;
    printFeatureVector( phi_max );

    cout << "MIN VALUES : " << endl;
    printFeatureVector( phi_min );

    cout << "MEAN VALUES : " << endl;
    printFeatureVector( phi_sum );

    exit(1);
    return phi_sum;
}

HumanTrajCostSpace::~HumanTrajCostSpace()
{

}

void HumanTrajCostSpace::setPassiveConfig( const Move3D::Configuration& q )
{
    human_passive_->setAndUpdate(q);
}

void HumanTrajCostSpace::setPassiveTrajectory( const motion_t& motion )
{
    Move3D::Trajectory t( human_passive_ );

    for(int i=0;i<int(motion.size());i++)
    {
        t.push_back( motion[i].second->copy() );
    }
    t.cutTrajInSmallLP( nb_way_points_-1 );
    passive_traj_ = t;
    passive_traj_.replaceP3dTraj();
}

//FeatureVect HumanTrajCostSpace::getFeatures(const Configuration& q)
//{
//    FeatureVect vect = dist_feat_.getFeatures(q);
//    return vect;
//}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Human Trajectory Simulator

HumanTrajSimulator::HumanTrajSimulator( HumanTrajCostSpace* cost_space )  :
    cost_space_( cost_space ),
    human_active_( cost_space->getActiveHuman() ),
    human_passive_( cost_space->getPassiveHuman() ),
    init_scenario_(false)
{
    cout << "Human active : " << human_active_->getName() << endl;
    cout << "Human passive : " << human_passive_->getName() << endl;

    is_pelvis_bound_user_defined_ = false;
    use_bio_models_ =  ( human_active_->getJoint( "rShoulderZ" ) == NULL );
}

bool HumanTrajSimulator::init()
{
    cout << __PRETTY_FUNCTION__ << endl;
//    if( !global_motionRecorders[0]->getStoredMotions().empty() )
//    {
//        cout << "Load init and goal from file" << endl;
//        const motion_t& motion_pas = global_motionRecorders[0]->getStoredMotions()[0];
//        const motion_t& motion_act = global_motionRecorders[1]->getStoredMotions()[0];

//        q_init_ = motion_act[0].second;
//        q_goal_ = motion_act.back().second;

//        // Adds the trajectory from the passive robot
//        // to the cost space
//        cost_space_->setPassiveTrajectory( motion_pas );
//    }
//    else
//    {
        q_init_ = human_active_->getInitPos();
        q_goal_ = human_active_->getGoalPos();
//    }

    // Set humans colors
    setHumanColor( human_active_, 0 ); // 3 // 0 Black
    setHumanColor( human_passive_, 2 ); // 2 Yellow // 3 Red


    // Sets the active robot as active for planning
    Move3D::global_Project->getActiveScene()->setActiveRobot( human_active_->getName() );

    // Change dofs limit
    // human_active_->se

    // Set init and goal config
    human_active_->setInitPos( *q_init_ );
    human_active_->setGoalPos( *q_goal_ );

    // Set to init pose
    human_active_->setAndUpdate( *q_init_ );

    // Set bounds and dofs
    setActiveJoints();

    // set the pointers to the motion recorder
    motion_recorders_ = global_motionRecorders;

    // Set minimal demonstration size
    minimal_demo_size_ = 10;
    trajectories_cut_ = false;

    use_one_traj_ = false;

    if( !motion_recorders_.empty() )
    {
        // Store demonstrations, compute pelvis bounds
        // add cut motions
        setReplanningDemonstrations();

//        if( !use_one_traj_ )
//            addCutMotions();

        // setInitAndGoalConfig(); // For simulation

    }

    // Set the planning bounds
    setPelvisBounds();

    // Set the current frame to 0
    current_frame_ = 0;
    id_of_demonstration_ = 0;

    draw_trace_ = true;

    if( global_DrawModule )
    {
        global_DrawModule->addDrawFunction( "HumanSimulator", boost::bind( &HumanTrajSimulator::draw, this) );
        global_DrawModule->enableDrawFunction( "HumanSimulator" );
    }

    cout << "pelvis_max_ : " << pelvis_max_.transpose() << endl;
    cout << "pelvis_min_ : " << pelvis_min_.transpose() << endl;

    return true;
}

void HumanTrajSimulator::draw()
{
    if( draw_trace_ && GestEnv->getBool(GestParam::draw_recorded_motion))
    {
        int nb_frames = 15;
        int id_of_demo_to_draw = HriEnv->getInt(HricsParam::ioc_spheres_to_draw);

        cout << "id_of_demonstration_ : " << id_of_demo_to_draw << endl;

        motion_recorders_[0]->drawMotion( human_1_motions_[id_of_demo_to_draw], nb_frames );
        motion_recorders_[1]->drawMotion( human_2_motions_[id_of_demo_to_draw], nb_frames );
    }
}

void HumanTrajSimulator::updateDofBounds( bool& initialized, Move3D::confPtr_t q_tmp )
{
//                    if(s == 0)
//                    {
    for( size_t i=0; i<6; i++)
    {
        if( initialized )
        {
            if( pelvis_max_[i] < (*q_tmp)[6+i] )
                pelvis_max_[i] = (*q_tmp)[6+i];
            if( pelvis_min_[i] > (*q_tmp)[6+i] )
                pelvis_min_[i] = (*q_tmp)[6+i];
        }
        else
        {
            pelvis_max_[i] = (*q_tmp)[6+i];
            pelvis_min_[i] = (*q_tmp)[6+i];
        }
    }

    if( use_bio_models_ )
    {
        int shoulder_trans_idx = human_active_->getJoint( "rShoulderTransX" )->getIndexOfFirstDof();

        for( size_t i=0; i<3; i++ )
        {
            if( initialized )
            {
                if( shoulder_trans_max_[i] < (*q_tmp)[shoulder_trans_idx+i] )
                    shoulder_trans_max_[i] = (*q_tmp)[shoulder_trans_idx+i];
                if( shoulder_trans_min_[i] > (*q_tmp)[shoulder_trans_idx+i] )
                    shoulder_trans_min_[i] = (*q_tmp)[shoulder_trans_idx+i];
            }
            else
            {
                shoulder_trans_max_[i] = (*q_tmp)[shoulder_trans_idx+i];
                shoulder_trans_min_[i] = (*q_tmp)[shoulder_trans_idx+i];
            }
        }
    }

    int arm_dof_index;

    if( initialized )
    {
        arm_dof_index = human_active_->getJoint( "rArmTrans" )->getIndexOfFirstDof();
        if( arm_max_ < (*q_tmp)[arm_dof_index] )
            arm_max_ = (*q_tmp)[arm_dof_index];
        if( arm_min_ > (*q_tmp)[arm_dof_index] )
            arm_min_ = (*q_tmp)[arm_dof_index];

        if( use_bio_models_ )
        {
            arm_dof_index = human_active_->getJoint( "lPoint" )->getIndexOfFirstDof();
            if( forearm_max_ < (*q_tmp)[arm_dof_index] )
                forearm_max_ = (*q_tmp)[arm_dof_index];
            if( forearm_min_ > (*q_tmp)[arm_dof_index] )
                forearm_min_ = (*q_tmp)[arm_dof_index];
        }
    }
    else
    {
        arm_dof_index = human_active_->getJoint( "rArmTrans" )->getIndexOfFirstDof();
        arm_max_ = (*q_tmp)[arm_dof_index];
        arm_min_ = (*q_tmp)[arm_dof_index];

        if( use_bio_models_ )
        {
            arm_dof_index = human_active_->getJoint( "lPoint" )->getIndexOfFirstDof();
            forearm_max_ = (*q_tmp)[arm_dof_index];
            forearm_min_ = (*q_tmp)[arm_dof_index];
        }
    }

    initialized = true;
//                    }
}

void HumanTrajSimulator::setDemonstrations( const std::vector<motion_t>& demos )
{
    human_2_motions_.clear();

    for( size_t d=0; d<demos.size(); d++ )
    {
        human_2_motions_.push_back( demos[d] );

        cout << "human_2_motions_[" << d << "].size() : " << human_2_motions_[d].size() << endl;

//        for( size_t s=0; s<demos[d].size(); s++ )
//        {
//            Move3D::confPtr_t q = human_2_motions_.back()[s].second;
//            Move3D::confPtr_t q_tmp = q->getRobot()->getInitPos();
//            q_tmp->setFromEigenVector( q->getEigenVector(active_dofs_), active_dofs_ );
//            q_tmp->adaptCircularJointsLimits();
//            human_2_motions_.back()[s].second = q_tmp;
////            updateDofBounds( initialized, q_tmp );
//        }
    }
}

void HumanTrajSimulator::setReplanningDemonstrations()
{
    cout << "---------------------------------------------" << endl;
    cout << "Set replanning demonstrations" << endl;

    std::vector<std::string> good_motions_names;
//    good_motions_names.push_back( "[0551-0602]motion_saved_00000_00000.csv" );
//    good_motions_names.push_back( "[1186-1245]motion_saved_00000_00001.csv" );
//    good_motions_names.push_back( "[1552-1581]motion_saved_00000_00000.csv" );
//    good_motions_names.push_back( "[1873-1929]motion_saved_00000_00001.csv" );
//    good_motions_names.push_back( "[3191-3234]motion_saved_00000_00000.csv" );
//    good_motions_names.push_back( "[3913-3950]motion_saved_00000_00000.csv" );

    // QUAN GOOD
//    good_motions_names.push_back( "[4125-4169]motion_saved_00000_00001.csv" );
//    good_motions_names.push_back( "[4422-4476]motion_saved_00000_00000.csv" );
//    good_motions_names.push_back( "[4591-4640]motion_saved_00000_00000.csv" );
//    good_motions_names.push_back( "[4753-4802]motion_saved_00000_00000.csv" );

    // MOCAP GOOD
//    good_motions_names.push_back( "[1460-1620]_human1_.csv" );
//    good_motions_names.push_back( "[1460-1620]_human2_.csv" );

    // TEN MOTIONS

    // BAD...
    //    good_motions_names.push_back("[1188-1256]_human2_.csv");
    //    good_motions_names.push_back("[1188-1256]_human1_.csv");
    //    good_motions_names.push_back("[2172-2249]_human2_.csv");
    //    good_motions_names.push_back("[2172-2249]_human1_.csv");
    //    good_motions_names.push_back("[2018-2099]_human2_.csv");
    //    good_motions_names.push_back("[2018-2099]_human1_.csv");
    //    good_motions_names.push_back("[1064-1140]_human2_.csv");
    //    good_motions_names.push_back("[1064-1140]_human1_.csv");
    //    good_motions_names.push_back("[0408-0491]_human2_.csv");
    //    good_motions_names.push_back("[0408-0491]_human1_.csv");

    // GOOD...
//    good_motions_names.push_back("[0446-0578]_human2_.csv");
//    good_motions_names.push_back("[0446-0578]_human1_.csv");

//    good_motions_names.push_back("[0525-0657]_human2_.csv");
//    good_motions_names.push_back("[0525-0657]_human1_.csv");

//    good_motions_names.push_back("[0444-0585]_human2_.csv");
//    good_motions_names.push_back("[0444-0585]_human1_.csv");

//    good_motions_names.push_back("[0489-0589]_human2_.csv");
//    good_motions_names.push_back("[0489-0589]_human1_.csv");

//    good_motions_names.push_back("[0780-0871]_human2_.csv");
//    good_motions_names.push_back("[0780-0871]_human1_.csv");

//    good_motions_names.push_back("[1537-1608]_human2_.csv");
//    good_motions_names.push_back("[1537-1608]_human1_.csv");

//    good_motions_names.push_back("[2711-2823]_human2_.csv");
//    good_motions_names.push_back("[2711-2823]_human1_.csv");

    // REPLANNING MOTION last

    good_motions_names.push_back("[7395-7595]_human2_.csv");
    good_motions_names.push_back("[7395-7595]_human1_.csv");

    // REPLANNING MOTION first
//    good_motions_names.push_back("[0629-0768]_human2_.csv");
//    good_motions_names.push_back("[0629-0768]_human1_.csv");


    if( !use_one_traj_ )
    {
//        good_motions_names.push_back("[0446-0578]_human2_.csv");
//        good_motions_names.push_back("[0446-0578]_human1_.csv");

//        good_motions_names.push_back("[0525-0657]_human2_.csv");
//        good_motions_names.push_back("[0525-0657]_human1_.csv");

//        good_motions_names.push_back("[0444-0585]_human2_.csv");
//        good_motions_names.push_back("[0444-0585]_human1_.csv");

//        good_motions_names.push_back("[0489-0589]_human2_.csv");
//        good_motions_names.push_back("[0489-0589]_human1_.csv");

//        good_motions_names.push_back("[0780-0871]_human2_.csv");
//        good_motions_names.push_back("[0780-0871]_human1_.csv");

//        good_motions_names.push_back("[1537-1608]_human2_.csv");
//        good_motions_names.push_back("[1537-1608]_human1_.csv");

//        good_motions_names.push_back("[2711-2823]_human2_.csv");
//        good_motions_names.push_back("[2711-2823]_human1_.csv");



//        good_motions_names.push_back("[1342-1451]_human2_.csv");
//        good_motions_names.push_back("[1342-1451]_human1_.csv");
//        good_motions_names.push_back("[2197-2343]_human2_.csv");
//        good_motions_names.push_back("[2197-2343]_human1_.csv");
    }

    human_1_demos_.clear();
    human_2_demos_.clear();

    pelvis_max_ = Eigen::VectorXd::Zero(6);
    pelvis_min_ = Eigen::VectorXd::Zero(6);

    shoulder_trans_max_ = Eigen::VectorXd::Zero(3);
    shoulder_trans_min_ = Eigen::VectorXd::Zero(3);

    motions_demo_ids_.clear();

    bool initialized = false;

    for( size_t k=0; k<good_motions_names.size(); k++ )
        for( size_t j=0; j<motion_recorders_[0]->getStoredMotions().size(); j++ )
            if( motion_recorders_[0]->getStoredMotionName(j) == good_motions_names[k] )
            {
                cout << "Add motion : " << good_motions_names[k] << endl;
                human_1_motions_.push_back( motion_recorders_[0]->getStoredMotions()[j] );
                human_2_motions_.push_back( motion_recorders_[1]->getStoredMotions()[j] );

                motions_1_names_.push_back( motion_recorders_[0]->getStoredMotionName(j) );
                motions_2_names_.push_back( motion_recorders_[1]->getStoredMotionName(j) );

                human_1_demos_.push_back( human_1_motions_.back() );
                human_2_demos_.push_back( human_2_motions_.back() );

                //
                int demo_id = motions_demo_ids_.size();
                motions_demo_ids_.push_back( demo_id );

                bool only_set_active_dofs = true;
                if( only_set_active_dofs )
                {
                    for( size_t s=0; s<human_2_motions_.back().size(); s++) // Only set active dofs on the configuration
                    {
                        Move3D::confPtr_t q = human_2_motions_.back()[s].second;
                        Move3D::confPtr_t q_tmp = q->getRobot()->getInitPos();
                        q_tmp->setFromEigenVector( q->getEigenVector(active_dofs_), active_dofs_ );
                        q_tmp->adaptCircularJointsLimits();
                        human_2_motions_.back()[s].second = q_tmp;
                        updateDofBounds( initialized, q_tmp );
                    }

                    if( use_bio_models_ ) // This might work for none bio models (kinect data) needs testing
                        for( size_t s=0; s<human_1_motions_.back().size(); s++) // Only set active dofs on the configuration
                        {
                            Move3D::confPtr_t q = human_1_motions_.back()[s].second;
                            Move3D::confPtr_t q_tmp = q->getRobot()->getInitPos();
                            q_tmp->setFromEigenVector( q->getEigenVector(active_dofs_), active_dofs_ );
                            q_tmp->adaptCircularJointsLimits();
                            human_1_motions_.back()[s].second = q_tmp;
                        }
                }
            }

    cout << "---------------------------------------------" << endl;
}

void HumanTrajSimulator::setInitAndGoalConfig()
{
    if( (!human_2_motions_.empty()) && (!human_2_motions_[0].empty()) )
    {
        human_active_->setInitPos( *human_2_motions_[0][0].second );
        human_active_->setGoalPos( *human_2_motions_[0].back().second );
    }
}

std::vector< std::vector<motion_t> > HumanTrajSimulator::getMotions()
{
    std::vector< std::vector<motion_t> > motions;
    motions.push_back( human_1_motions_ );
    motions.push_back( human_2_motions_ );
    return motions;
}

void HumanTrajSimulator::addCutMotions()
{
    cout << "size 1 before adding cut motions : " << human_1_motions_.size() << endl;
    cout << "size 2 before adding cut motions : " << human_2_motions_.size() << endl;

    std::vector<motion_t> human_1_motion_tmp;
    std::vector<motion_t> human_2_motion_tmp;

    motions_demo_ids_.clear();

    for( size_t i=0; i<human_1_motions_.size(); i++ ) // Add original motions
    {
        human_1_motion_tmp.push_back( human_1_motions_[i] );
        human_2_motion_tmp.push_back( human_2_motions_[i] );
        motions_demo_ids_.push_back( i );
    }

    cut_step_ = 10; // 10 * 0.01 = 0.1 sec
    minimal_demo_size_ = 80; // 70 * 0.01 = 0.7 sec

    for( size_t i=0; i<human_2_motions_.size(); i++ )
    {
        if( human_2_motions_[i].empty() )
            continue;
        if( human_2_motions_[i].size() != human_1_motions_[i].size() )
            continue;

        int max_nb_of_removed_frames = int(human_2_motions_[i].size()) - minimal_demo_size_;

        motion_t::const_iterator init_1 = human_1_motions_[i].begin();
        motion_t::const_iterator goal_1 = human_1_motions_[i].end();

        motion_t::const_iterator init_2 = human_2_motions_[i].begin();
        motion_t::const_iterator goal_2 = human_2_motions_[i].end();

        cout << "motion 2 size : " << human_2_motions_[i].size() << endl;
        cout << "min demo size : " << minimal_demo_size_ << endl;
        cout << "max_nb_of_removed_frames : " << max_nb_of_removed_frames << endl;

        // Add smaller cut trajectories
        for( int j=0; j<max_nb_of_removed_frames; j++ )
        {
            init_1++;
            init_2++;

            if( j % cut_step_ != 0 ){
                continue;
            }

            motion_t motion_1( init_1, goal_1 );
            motion_t motion_2( init_2, goal_2 );

            motion_1[0].first = 0.0;
            motion_2[0].first = 0.0;

            human_1_motion_tmp.push_back( motion_1 );
            human_2_motion_tmp.push_back( motion_2 );

            motions_demo_ids_.push_back( i );
        }

        trajectories_cut_ = true;
    }

    if( trajectories_cut_ )
    {
        human_1_motions_ = human_1_motion_tmp;
        human_2_motions_ = human_2_motion_tmp;
    }

    cout << "size 1 after adding cut motions : " << human_1_motions_.size() << endl;
    cout << "size 2 after adding cut motions : " << human_2_motions_.size() << endl;

    for( int i=0; i<human_1_motions_.size(); i++ ){
        cout << "time length [" << motions_demo_ids_[i] << "][" << i << "] : " << motion_duration( human_1_motions_[i] ) << endl;
    }
}

std::vector<Move3D::Trajectory> HumanTrajSimulator::getDemoTrajectories() const
{
    std::vector<Move3D::Trajectory> trajs;

    if( human_2_motions_.empty() )
        return trajs;

    for( size_t i=0; i<human_2_motions_.size(); i++ )
    {
        if( human_2_motions_[i].empty() )
            continue;

        Move3D::Robot* robot = human_2_motions_[i][0].second->getRobot();
        trajs.push_back( motion_to_traj( human_2_motions_[i], robot ) );
    }

    return trajs;
}

std::vector<Move3D::confPtr_t> HumanTrajSimulator::getContext() const
{
    std::vector<Move3D::confPtr_t> context;

    for( size_t i=0; i<human_1_motions_.size(); i++ )
    {
        if( human_1_motions_[i].empty() )
            continue;

        context.push_back( human_1_motions_[i][0].second->copy() );
    }

    return context;
}

void HumanTrajSimulator::setPelvisBounds()
{
    // Get first joint and change bounds
    Move3D::Joint* pelvis_joint = human_active_->getJoint( "Pelvis" );

    double dof[6][2];

    if( is_pelvis_bound_user_defined_ )
    {
        // take only (x, y, z) components of the base
        double bound_trans = 0.05;
        double bound_rotat = 0.1;

        for(int i = 0; i < 3; i++) { // Translation bounds
            dof[i][0] = pelvis_joint->getJointDof(i) - bound_trans;
            dof[i][1] = pelvis_joint->getJointDof(i) + bound_trans;
            cout << "PELVIS DOF: " << pelvis_joint->getJointDof(i) << endl;
        }
        for(int i = 3; i < 6; i++) { // Rotation bounds
            dof[i][0] = pelvis_joint->getJointDof(i) - bound_rotat;
            dof[i][1] = pelvis_joint->getJointDof(i) + bound_rotat;
        }

        arm_min_ = -.10;
        arm_max_ =  .10;
    }
    else
    {
        double bound_trans = 0.02;
        double bound_rotat = 0.05;

        if( pelvis_min_.size() == 6 && pelvis_max_.size() == 6 )
        {
            for(int i = 0; i < 3; i++) { // Translation bounds
                dof[i][0] = pelvis_min_(i) - bound_trans;
                dof[i][1] = pelvis_max_(i) + bound_trans;
            }
            for(int i = 3; i < 6; i++) { // Rotation bounds
                dof[i][0] = pelvis_min_(i) - bound_rotat;
                dof[i][1] = pelvis_max_(i) + bound_rotat;
            }
        }
    }

    for(int i=0; i<6; i++)
        p3d_jnt_set_dof_rand_bounds( pelvis_joint->getP3dJointStruct(), i, dof[i][0], dof[i][1] );

    double bound_translations = 0.02;

    Move3D::Joint* arm_joint = human_active_->getJoint( "rArmTrans" );
    p3d_jnt_set_dof_rand_bounds( arm_joint->getP3dJointStruct(), 0, arm_min_ - bound_translations, arm_max_ + bound_translations );

    if( use_bio_models_ )
    {
        Move3D::Joint* forearm_joint = human_active_->getJoint( "lPoint" );
        p3d_jnt_set_dof_rand_bounds( forearm_joint->getP3dJointStruct(), 0, forearm_min_ - bound_translations, forearm_max_ + bound_translations );

        if( shoulder_trans_min_.size() == 3 && shoulder_trans_max_.size() == 3 )
        {
            for(int i = 0; i < 3; i++) { // Translation bounds
                dof[i][0] = shoulder_trans_min_(i) - bound_translations;
                dof[i][1] = shoulder_trans_max_(i) + bound_translations;
            }
        }

        cout << "shoulder_trans_min_.transpose() : " << shoulder_trans_min_.transpose() << endl;
        cout << "shoulder_trans_max_.transpose() : " << shoulder_trans_max_.transpose() << endl;

        p3d_jnt_set_dof_rand_bounds( human_active_->getJoint( "rShoulderTransX" )->getP3dJointStruct(), 0, dof[0][0], dof[0][1] );
        p3d_jnt_set_dof_rand_bounds( human_active_->getJoint( "rShoulderTransY" )->getP3dJointStruct(), 0, dof[1][0], dof[1][1] );
        p3d_jnt_set_dof_rand_bounds( human_active_->getJoint( "rShoulderTransZ" )->getP3dJointStruct(), 0, dof[2][0], dof[2][1] );
    }

    // Set active joints
    for( int i=0; i<human_active_->getNumberOfJoints(); i++)
        p3d_jnt_set_is_user( human_active_->getJoint(i)->getP3dJointStruct(), FALSE );
    for( size_t i=0; i<active_joints_.size(); i++)
        p3d_jnt_set_is_user( active_joints_[i]->getP3dJointStruct(), TRUE );

//    Joint(0), Dof : 6, Pelvis
//    Is dof user : (min = 0.94, max = 1.04)
//    Joint(1), Dof : 7, Pelvis
//    Is dof user : (min = 0.61, max = 0.71)
//    Joint(2), Dof : 8, Pelvis
//    Is dof user : (min = 0.92, max = 1.02)
//    Joint(3), Dof : 9, Pelvis
//    Is dof user : (min = -0.10, max = 0.10)
//    Joint(4), Dof : 10, Pelvis
//    Is dof user : (min = -0.10, max = 0.10)
//    Joint(5), Dof : 11, Pelvis

//    Is dof user : (min = -0.77, max = -0.57)
//    Joint(0), Dof : 12, TorsoX
//    Is dof user : (min = -0.79, max = 0.79)
//    Joint(0), Dof : 13, TorsoY
//    Is dof user : (min = -0.35, max = 0.70)
//    Joint(0), Dof : 14, TorsoZ

//    Is dof user : (min = -0.79, max = 0.79)
//    Joint(0), Dof : 18, rShoulderX
//    Is dof user : (min = -3.14, max = 3.14)
//    Joint(0), Dof : 19, rShoulderZ
//    Is dof user : (min = -3.14, max = 3.14)
//    Joint(0), Dof : 20, rShoulderY

//    Is dof user : (min = -3.14, max = 3.14)
//    Joint(0), Dof : 22, rElbowZ
//    Is dof user : (min = -3.14, max = 3.14)
}
std::vector<int> HumanTrajSimulator::getActiveDofs() const
{
    return active_dofs_;
}

void HumanTrajSimulator::setActiveJoints()
{
    if( use_bio_models_ )
    {
        active_joints_.push_back( human_active_->getJoint( "Pelvis" ) );
        active_joints_.push_back( human_active_->getJoint( "TorsoX" ) );
        active_joints_.push_back( human_active_->getJoint( "TorsoZ" ) );
        active_joints_.push_back( human_active_->getJoint( "TorsoY" ) );
        active_joints_.push_back( human_active_->getJoint( "rShoulderTransX" ) );
        active_joints_.push_back( human_active_->getJoint( "rShoulderTransY" ) );
        active_joints_.push_back( human_active_->getJoint( "rShoulderTransZ" ) );
        active_joints_.push_back( human_active_->getJoint( "rShoulderY1" ) );
        active_joints_.push_back( human_active_->getJoint( "rShoulderX" ) );
        active_joints_.push_back( human_active_->getJoint( "rShoulderY2" ) );
        active_joints_.push_back( human_active_->getJoint( "rArmTrans" ) );
        active_joints_.push_back( human_active_->getJoint( "rElbowZ" ) );
        active_joints_.push_back( human_active_->getJoint( "rElbowX" ) );
        active_joints_.push_back( human_active_->getJoint( "rElbowY" ) );
        active_joints_.push_back( human_active_->getJoint( "lPoint" ) );
        active_joints_.push_back( human_active_->getJoint( "rWristZ" ) );
        active_joints_.push_back( human_active_->getJoint( "rWristX" ) );
        active_joints_.push_back( human_active_->getJoint( "rWristY" ) );
    }
    else {
        active_joints_.push_back( human_active_->getJoint( "Pelvis" ) ); // Pelvis
        active_joints_.push_back( human_active_->getJoint( "TorsoX" ) ); // TorsoX
        active_joints_.push_back( human_active_->getJoint( "TorsoY" ) ); // TorsoY
        active_joints_.push_back( human_active_->getJoint( "TorsoZ" ) ); // TorsoZ
        active_joints_.push_back( human_active_->getJoint( "rShoulderX" ) ); // rShoulderX
        active_joints_.push_back( human_active_->getJoint( "rShoulderZ" ) ); // rShoulderZ
        active_joints_.push_back( human_active_->getJoint( "rShoulderY" ) ); // rShoulderY
        active_joints_.push_back( human_active_->getJoint( "rArmTrans" ) ); // rArmTrans
        active_joints_.push_back( human_active_->getJoint( "rElbowZ" ) ); // rElbowZ
    }


//    active_joints_.push_back(14); // joint name : rWristX
//    active_joints_.push_back(15); // joint name : rWristY
//    active_joints_.push_back(16); // joint name : rWristZ

    // SET COST SPACE ACTIVE DOFS

    active_dofs_.clear();

    for( size_t i=0; i<active_joints_.size(); i++ ) // get all active dofs
    {
        std::vector<unsigned int> jnt_active_dofs = active_joints_[i]->getDofIndices();
        active_dofs_.insert( active_dofs_.end(), jnt_active_dofs.begin(), jnt_active_dofs.end() );
        cout << " active joints dof (" << active_joints_[i]->getName() << ") [" << i << "] : " << active_joints_[i]->getIndexOfFirstDof() << endl;
    }

    for( size_t i=0; i<active_dofs_.size(); i++ ) // print all active dofs
        cout << " active dofs [" << i << "] : " << active_dofs_[i] << endl;

    for( size_t i=0; i<cost_space_->getNumberOfFeatureFunctions(); i++ ) // set all features active dofs
    {
        cost_space_->getFeatureFunction(i)->setActiveDoFs( active_dofs_ );
    }

    global_humanPredictionCostSpace->setActiveJoints( active_joints_ );
}

void HumanTrajSimulator::setHumanColor(Move3D::Robot* human, int color)
{
    if( !human->getUseLibmove3dStruct() )
        return;

    for (int i=1; i<= human->getP3dRobotStruct()->njoints; i++)
    {
        p3d_obj* obj = human->getP3dRobotStruct()->joints[i]->o;

        if(obj == NULL)
            continue;

        for(int j = 0; j < obj->np; j++)
        {
            p3d_poly* poly = obj->pol[j];

            if(poly->color_vect == NULL)
                continue;

            if (         poly->color_vect[0] == 0.80
                      && poly->color_vect[1] == 0.00
                      && poly->color_vect[2] == 0.01) {

                double color_vect[4];

                g3d_get_color_vect(color, color_vect);

                poly->color = color;

                poly->color_vect[0] = color_vect[0];
                poly->color_vect[1] = color_vect[1];
                poly->color_vect[2] = color_vect[2];
                poly->color_vect[3] = color_vect[3];
            }
        }
    }
}

Move3D::Trajectory HumanTrajSimulator::getExecutedPath() const
{
    return HRICS::motion_to_traj( executed_trajectory_, human_active_ );
}

Move3D::Trajectory HumanTrajSimulator::getCurrentPath() const
{
    Move3D::Trajectory traj( path_ );
    traj.cutTrajInSmallLP( human_passive_motion_.size()- 1 );
    return traj;
}

motion_t HumanTrajSimulator::getExecutedTrajectory() const
{
    return executed_trajectory_;
}

//! HACKISH TODO have time trajectories
double HumanTrajSimulator::getCost( const motion_t& traj ) const
{
//    cout << "Get cost of traj" << endl;
//    cost_space_->printInfo();

    // get smoothness feature
    Move3D::Trajectory t = motion_to_traj( traj, human_active_ );
    Move3D::FeatureVect phi = cost_space_->getFeatureCount( t );

    // reset config dependent features
    for( int i=0; i<phi.size(); i++)
    {
//        cout << " feature at " << i << " : " << cost_space_->getFeatureFunctionAtIndex(i)->getName() << endl;
        if( cost_space_->getFeatureFunctionAtIndex(i)->is_config_dependent_ )
            phi[i] = 0;
    }

    // get configuration dependent features
    Move3D::confPtr_t q_1, q_2;
    int nb_via_points = traj.size();
    double dist = traj[0].second->dist( *traj[1].second );
    double time=0;

    for (int i=1; i<nb_via_points+1; i++)
    {
        int j =0;
        double time_traj = 0.0;
        while( j < human_1_motions_[id_of_demonstration_].size() ) // set passive human at time along motion
        {
            time_traj += human_1_motions_[id_of_demonstration_][j].first;

            if( time_traj >= time )
            {
                Move3D::confPtr_t q = human_1_motions_[id_of_demonstration_][j].second;
                human_passive_->setAndUpdate( *q );
                break;
            }

            j++;
        }

        q_1 = traj[i-1].second;

        human_active_->setAndUpdate( *q_1 );

        phi += ( cost_space_->getFeatures( *q_1 ) * dist );

        if( i < nb_via_points )
        {
            q_2 = traj[i].second;
            dist = q_1->dist( *q_2 );
        }

        time += traj[i].first;
    }


//    cost_space_->print( phi );

    return cost_space_->getWeights().transpose() * phi;
}

bool HumanTrajSimulator::loadActiveHumanGoalConfig()
{
    if( id_of_demonstration_ >= human_2_motions_.size()  )
        return false;

    // INITIALIZE SIMULATION
    current_human_traj_.resize( 0, 0 );
    executed_trajectory_.clear();
    best_path_id_ = -1;
    current_frame_ = 0;
    human_passive_increment_ = 1;
    current_time_ = 0.0;
    time_step_ = 0.1; // Simulation step
    global_discretization_ = 0.01; // time betweem configurations (choose a number that divides the simulation time step)
    time_along_current_path_ = 0.0;
    end_simulation_ = false;

    draw_execute_motion_ = false;

    // GET STORED CONFIGURATIONS
//    std::vector<Move3D::confPtr_t> configs = human_active_->getStoredConfigs();
//    if( configs.empty() ){
//        return false;
//    }
    //    cout << "size of stored config : " << configs.size() << endl;

    cout << "id_of_demonstration_ : " << id_of_demonstration_ << endl;

    // Set human passive motion
    // Demos have all dofs for the humans
    // the left arm is not set down
//    human_passive_motion_ = human_1_demos_[ id_of_demonstration_ ];
//    human_active_motion_  = human_2_demos_[ id_of_demonstration_ ];

    human_passive_motion_ = human_1_motions_[ id_of_demonstration_ ];
    human_active_motion_  = human_2_motions_[ id_of_demonstration_ ];

    // LOAD ACTIVE HUMAN MOTION SCENARIO
    q_init_ = human_active_motion_[0].second;
    q_goal_ = human_active_motion_.back().second;

    // q_goal_ = configs[ id_of_demonstration_ ];

    human_active_increments_per_exection_ = 10;
    human_active_step_ = q_init_->dist( *q_goal_ ) / 200;

    // Use passive motion because active changes when set at the end of simulation
    motion_duration_ = motion_duration( human_passive_motion_ );
    current_motion_duration_ = motion_duration_;

    // Iniitialize with 0.0
    executed_trajectory_.push_back( std::make_pair( 0.0, q_init_ ) );

    return true;
}

Move3D::Trajectory HumanTrajSimulator::setTrajectoryToMainRobot(const Move3D::Trajectory& traj) const
{
    Move3D::Trajectory traj_tmp(human_active_);

    for(int i=0;i<traj.getNbOfViaPoints();i++)
    {
        Move3D::confPtr_t q(new Move3D::Configuration(human_active_));
        q->setConfiguration( traj[i]->getConfigStructCopy() );
        traj_tmp.push_back( q );
    }

    return traj_tmp;
}

void HumanTrajSimulator::runStandardStomp( int iter )
{
    int nb_way_points = 100;
    PlanEnv->setInt( PlanParam::nb_pointsOnTraj, nb_way_points );
//    traj_optim_set_discretize( true );
//    traj_optim_set_discretization( 0.015 );

    human_active_->setAndUpdate( *q_init_ );

    human_active_->setInitPos( *q_init_->copy() );
    human_active_->setGoalPos( *q_goal_->copy() );

    if( iter>0 )
    {
        Move3D::Trajectory optimi_traj( human_active_ );
        double dt = current_motion_duration_ / double(nb_way_points-1);
        for(int i=0; i<nb_way_points-1; i++) {
            optimi_traj.push_back( path_.configAtTime( time_along_current_path_ + double(i) * dt ) );
        }
        optimi_traj.push_back( path_.getEnd() );

        traj_optim_set_use_extern_trajectory( true );
        traj_optim_set_extern_trajectory( optimi_traj );
    }
    else
    {
        traj_optim_set_use_extern_trajectory( false );

//        if( use_one_traj_)
//        {
//            // TEST WITH CURRENT DEMONSTRATION
//            traj_optim_set_use_extern_trajectory( true );
//            Move3D::Trajectory traj( motion_to_traj( human_active_motion_, human_active_ ) );
//            traj_optim_set_extern_trajectory( traj );
//        }
    }

    traj_optim_set_use_iteration_limit(true);
    traj_optim_set_iteration_limit( PlanEnv->getInt(PlanParam::stompMaxIteration) );
    PlanEnv->setDouble( PlanParam::trajDuration, current_motion_duration_ > 0.0 ? current_motion_duration_ : 0.1  );

    // Set buffer
    if( path_.size() > 0 && time_along_current_path_ > 0.0)
    {
        double dt = current_motion_duration_ / double(nb_way_points);
        std::vector<Eigen::VectorXd> buffer;
        int nb_config = 7;
        for(int i=0; i<nb_config; i++){
            Move3D::confPtr_t q = path_.configAtTime( time_along_current_path_-double(nb_config-i)*dt );
            buffer.push_back( q->getEigenVector( active_dofs_ ) );
        }
        traj_optim_set_buffer( buffer );

        if( !q_init_->equal( *path_.configAtTime( time_along_current_path_ ) ) ){
            cout << "ERROR IN INIT CONFIG" << endl;
            exit(0);
        }
    }

    traj_optim_resetInit();
    traj_optim_reset_collision_space();
    traj_optim_add_human_to_collision_space(true);

    traj_optim_runStomp(0);

//    exit(0);

    path_ = global_optimizer->getBestTraj();
    cout << "path.getUseTimeParameter() : " << path_.getUseTimeParameter() << endl;
}

bool HumanTrajSimulator::updatePassiveMotion()
{
    cout << "MOTION DURATION : " << motion_duration_ << endl;
    cout << "CURRENT TIME : " << current_time_ << endl;
    cout << "TIME LEFT : " << current_motion_duration_ << endl;
    cout << "TIME ALONG CURRENT TRAJ : " << time_along_current_path_ << endl;

    if( motion_duration_ - current_time_ < 1e-2 ) // inferior to a hundredth of a second
        return false;

    // Find closest configuration at current time
    // along motion trajectory
    double time_along_traj = 0.0;

    current_frame_ = -1;

    for( size_t i=0; i<human_passive_motion_.size(); i++ )
    {
        time_along_traj +=  human_passive_motion_[i].first;

        if ( time_along_traj > current_time_ )
        {
            Move3D::confPtr_t q_cur = human_passive_motion_[i].second;
            human_passive_->setAndUpdate(*q_cur);
            current_frame_ = i;
            break;
        }
    }

    if( current_frame_ == -1 )
        return false;

    return true;
}

void HumanTrajSimulator::execute(const Move3D::Trajectory& path, bool to_end)
{
    path.replaceP3dTraj();

    if( path.getUseTimeParameter() )
        current_discretization_ = path.getDeltaTime();
    else
        current_discretization_ = current_motion_duration_ / double( path.getNbOfPaths() );

    Move3D::confPtr_t q;

    double time_factor = 10; // Slow down execution by factor

    int nb_configs = time_step_ / global_discretization_; // global_discretization_ must be a multiple of time step
    double t = 0;

    for( int i=0; i<nb_configs; i++ )
    {
        // Find configurations of active human along the trajectory
        t += global_discretization_;
        q = path.configAtTime( t );
        human_active_->setAndUpdate( *q );

        // Add the configuration to the trajectory
        executed_trajectory_.push_back( std::make_pair( global_discretization_, q ) );

        // if the time exceeds the trajectory length
        // inferior to a hundredth of a second
        if( ( motion_duration_ - ( t + current_time_ ) ) < global_discretization_  )
        {
            end_simulation_ = true;
            break;
        }

        if( draw_execute_motion_ )
        {
            g3d_draw_allwin_active();
            usleep( floor( global_discretization_ * 1e6 * time_factor ) );
        }
    }

    // If duration left is inferior to half a time step do not replan
    if( ( motion_duration_ - ( t + current_time_ ) ) <= (time_step_/2.) )
    {
        while( motion_duration_ - ( t + current_time_ ) > global_discretization_ )
        {
            t += global_discretization_;
            q = path.configAtTime( t );
            executed_trajectory_.push_back( std::make_pair( global_discretization_, q ) );
        }
        end_simulation_ = true;
    }

    // Add the end configuration
    if( end_simulation_ ){
        double dt = motion_duration_ - ( t + current_time_ );
        t += dt;
        executed_trajectory_.push_back( std::make_pair( dt, q_goal_->copy() ) );
    }

    time_along_current_path_ = t;
    current_time_ += time_along_current_path_;
    current_motion_duration_ -= time_along_current_path_;

    q_init_ = q;

    cout << "End execute" << endl;
}

void HumanTrajSimulator::printCosts() const
{
//    for(int i =0;i<int(cost_.size());i++)
//    {
//        for(int j =0;j<int(cost_[i].size());j++)
//        {
//            cout << "cost_" << i << "_(" << j+1 << ") = " <<  cost_[i][j] << ";" <<endl;
//        }
//    }
}

double HumanTrajSimulator::run()
{
//    id_of_demonstration_ = 0; // TODO add global variable

    if( !loadActiveHumanGoalConfig() ) {
        cout << "Error could not load active human start and goal configurations" << endl;
        return 0.0;
    }

    cout << "Load human traj id  : " << GestEnv->getInt(GestParam::human_traj_id) << endl;

    // TODO REPLACE BY TRAJECTORY
    // loadHumanTrajectory( m_recorder->getStoredMotions()[GestEnv->getInt(GestParam::human_traj_id)] );

    cost_.clear();
    human_active_->setAndUpdate( *q_init_ );
    ENV.setBool( Env::isCostSpace, true );

    for(int i=0;(!PlanEnv->getBool(PlanParam::stopPlanner)) && updatePassiveMotion(); i++ )
    {
        g3d_draw_allwin_active();

        runStandardStomp( i );

        if( !PlanEnv->getBool(PlanParam::stopPlanner) )
            execute( path_, false );

//        cout << "wait for key" << endl;
//        cin.ignore();

//        path_.replaceP3dTraj();

        if( hrics_one_iteration && i == 0 ) // test no replanning
            break;
    }

//    if( !PlanEnv->getBool(PlanParam::stopPlanner) )
//    {
//        const double parameter =  double(human_active_increments_per_exection_) * human_active_step_;
//        execute( path_.extractSubTrajectory( parameter, path_.getParamMax(), false ), true );
//    }

    ENV.setBool( Env::isCostSpace, true );

    printCosts();

//    cout << "executed_path_.cost() : " << executed_path_.cost() << endl;

    Move3D::Trajectory path( hrics_one_iteration ? path_ : motion_to_traj( executed_trajectory_, human_active_ ));

    human_active_->setCurrentMove3DTraj( path );

    path.replaceP3dTraj();

    g3d_draw_allwin_active();

    cout << "executed_trajectory_.size() : " << executed_trajectory_.size() << endl;
    cout << "path.size() : " << path.getNbOfViaPoints() << endl;
    cout << "executed motion duration : " << motion_duration( executed_trajectory_ ) << endl;
    cout << "passive motion duration : " << motion_duration( human_passive_motion_ ) << endl;
    cout << " active motion duration : " << motion_duration( human_active_motion_ ) << endl;

    return 0.0;
}

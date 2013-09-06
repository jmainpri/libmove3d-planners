#include "HRICS_HumanCostSpace.hpp"

#include "HRICS_PlayMotion.hpp"
#include "API/project.hpp"
#include "planner/cost_space.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "p3d/env.hpp"

#include <boost/bind.hpp>

using namespace HRICS;
using std::cout;
using std::endl;

static HumanTrajCostSpace* ht_cost_space = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------

void HRICS_run_human_planning()
{
    std::string foldername = "/home/jmainpri/workspace/move3d/libmove3d/statFiles/collaboration/recorded_motion_01_09_13";

    Scene* sce = global_Project->getActiveScene();
    Robot* human1 = sce->getRobotByName( "HERAKLES_HUMAN1" );
    Robot* human2 = sce->getRobotByName( "HERAKLES_HUMAN2" );
    if( human1 == NULL || human2 == NULL )
    {
        cout << "No humans HERAKLES in the the scene" << endl;
        return;
    }

    global_motionRecorders.push_back( new HRICS::RecordMotion( human1 ) );
    global_motionRecorders.push_back( new HRICS::RecordMotion( human2 ) );

    // Set this bool to false is you want to print file names as they are loaded
    bool quiet = true;
    global_motionRecorders[0]->loadCSVFolder( foldername + "/human0", quiet );
    global_motionRecorders[1]->loadCSVFolder( foldername + "/human1", quiet );

// Uncomment the following to play the motion
//    PlayMotion player( global_motionRecorders );
////    for(int i=0;i<int(global_motionRecorders[0]->getStoredMotions().size());i++)
//    for(int i=0;i<int(1);i++)
//    {
//        player.play(i);
//    }

    if( ht_cost_space == NULL)
    {
        ht_cost_space = new HumanTrajCostSpace(  human2, human1 );

        // Define cost functions
        global_costSpace->addCost( "costHumanTrajecoryCost" , boost::bind( &HumanTrajCostSpace::cost, ht_cost_space, _1) );
    }

    ENV.setBool( Env::isCostSpace, true );
    global_costSpace->setCost( "costHumanTrajecoryCost" );

    HumanTrajSimulator sim( ht_cost_space );
    sim.init();
    sim.run();
}

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
}

bool HumanTrajSimulator::init()
{
    const motion_t& motion_pas = global_motionRecorders[0]->getStoredMotions()[0];
    const motion_t& motion_act = global_motionRecorders[1]->getStoredMotions()[0];

    if( motion_act.empty() )
    {
        return false;
    }
    q_init_ = motion_act[0].second;
    q_goal_ = motion_act.back().second;

    // Adds the trajectory from the passive robot
    // to the cost space
    cost_space_->setPassiveTrajectory( motion_pas );

    // Sets the active robot as active for planning
    global_Project->getActiveScene()->setActiveRobot( human_active_->getName() );

    // Change dofs limit
//    human_active_->se

    // Set init and goal config
    human_active_->setInitPos( *q_init_ );
    human_active_->setGoalPos( *q_goal_ );

    return true;
}

bool HumanTrajSimulator::run()
{
    cout << "run human traj simulator" << endl;

//    if( !init_scenario_ )
//    {
//        traj_optim_initScenario();
//    }

    traj_optim_runStomp(0);
    return true;
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Human Trajectory Cost Space

HumanTrajCostSpace::HumanTrajCostSpace( Robot* active, Robot* passive ) :
    human_active_(active),
    human_passive_(passive),
    dist_feat_( active, passive )
{
    nb_way_points_ = 20;

    w_ = getFeatures(*human_active_->getCurrentPos());

    for(int i=0;i<w_.size();i++)
    {
        w_[i] = 1;
    }
}

HumanTrajCostSpace::~HumanTrajCostSpace()
{

}

void HumanTrajCostSpace::setPassiveConfig( const Configuration& q )
{
    human_passive_->setAndUpdate(q);
}

void HumanTrajCostSpace::setPassiveTrajectory( const motion_t& motion )
{
    API::Trajectory t( human_passive_ );

    for(int i=0;i<int(motion.size());i++)
    {
        t.push_back( motion[i].second->copy() );
    }
    t.cutTrajInSmallLP( nb_way_points_-1 );
    passive_traj_ = t;
    passive_traj_.replaceP3dTraj();
}

FeatureVect HumanTrajCostSpace::getFeatureCount(const API::Trajectory& t)
{
    std::vector<FeatureVect> vect_stacked;
    vect_stacked.push_back( dist_feat_.getFeatureCount(t) );
    vect_stacked.push_back( visi_feat_.getFeatureCount(t) );
    vect_stacked.push_back( musk_feat_.getFeatureCount(t) );
    vect_stacked.push_back( reach_feat_.getFeatureCount(t) );
    vect_stacked.push_back( legib_feat_.getFeatureCount(t) );

    int size=0;
    for(int i=0; i<int(vect_stacked.size());i++)
        size += vect_stacked[i].size();

    FeatureVect features( size );
    int i=0;
    for( int d=0; d<int(features.size()); d++ )
    {
        for( int k=0; k<int(vect_stacked[k].size()); k++ )
        {
            features[i++] = vect_stacked[d][k];
        }
    }

    return features;
}

FeatureVect HumanTrajCostSpace::getFeatures(const Configuration& q)
{
    FeatureVect vect = dist_feat_.getFeatures(q);
    return vect;
}

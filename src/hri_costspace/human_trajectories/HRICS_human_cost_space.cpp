#include "HRICS_human_cost_space.hpp"

#include "HRICS_play_motion.hpp"
#include "API/project.hpp"

#include "planner/cost_space.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"

#include "p3d/env.hpp"

#include <boost/bind.hpp>
#include <iomanip>

using namespace Move3D;
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
        ht_cost_space = new HumanTrajCostSpace( human2, human1 );

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
    if( !global_motionRecorders[0]->getStoredMotions().empty() )
    {
        cout << "Load init and goal from file" << endl;
        const motion_t& motion_pas = global_motionRecorders[0]->getStoredMotions()[0];
        const motion_t& motion_act = global_motionRecorders[1]->getStoredMotions()[0];

        q_init_ = motion_act[0].second;
        q_goal_ = motion_act.back().second;

        // Adds the trajectory from the passive robot
        // to the cost space
        cost_space_->setPassiveTrajectory( motion_pas );
    }
    else
    {
        q_init_ = human_active_->getInitPos();
        q_goal_ = human_active_->getGoalPos();
    }

    // Set humans colors
    setHumanColor( human_active_, 3 );
    setHumanColor( human_passive_, 2 );

    // Sets the active robot as active for planning
    global_Project->getActiveScene()->setActiveRobot( human_active_->getName() );

    // Change dofs limit
    // human_active_->se

    // Set init and goal config
    human_active_->setInitPos( *q_init_ );
    human_active_->setGoalPos( *q_goal_ );

    // Set to init pose
    human_active_->setAndUpdate( *q_init_ );

    // Get first joint and change bounds
    Joint* joint = human_active_->getJoint(1);

    // take only (x, y, z) components of the base
    double bound_trans = 0.05;
    double bound_rotat = 0.1;
    double dof[6][2];
    for(int i = 0; i < 3; i++) { // Translation bounds
        dof[i][0] = joint->getJointDof(i) - bound_trans;
        dof[i][1] = joint->getJointDof(i) + bound_trans;
    }
    for(int i = 3; i < 6; i++) { // Rotation bounds
        dof[i][0] = joint->getJointDof(i) - bound_rotat;
        dof[i][1] = joint->getJointDof(i) + bound_rotat;
    }
    for(int i = 0; i < 6; i++){
        p3d_jnt_set_dof_rand_bounds( joint->getP3dJointStruct(), i, dof[i][0], dof[i][1] );
    }

    return true;
}

void HumanTrajSimulator::setHumanColor(Robot* human, int color)
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

bool HumanTrajSimulator::run()
{
    cout << "run human traj simulator" << endl;

    //    if( !init_scenario_ )
    //    {
    //        traj_optim_initScenario();
    //    }

    int nb_trajectories = 1;
    int max_iter = 100;

    traj_folder_ = std::string(getenv("HOME")) + "/Dropbox/move3d/assets/Collaboration/TRAJECTORIES/";

    for( int i=0;i<nb_trajectories;i++)
    {
        traj_optim_set_use_iteration_limit( true );
        traj_optim_set_iteration_limit( max_iter );
        traj_optim_runStomp(0);

        std::stringstream ss;
        ss << "trajectory" << std::setw(3) << std::setfill( '0' ) << i << ".traj";
        std::string filename = traj_folder_ + ss.str();

        p3d_save_traj( filename.c_str(), human_active_->getP3dRobotStruct()->tcur );

        cout << "save trajectory to : " << filename << endl;
    }
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

    addFeatureFunction( &dist_feat_ );

    cout << "HUMAN FEATURE SPACE : " << w_.size() << endl;
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

#include "HRICS_HumanIoc.hpp"

#include "HRICS_HumanCostSpace.hpp"
#include "HRICS_PlayMotion.hpp"
#include "HRICS_features.hpp"
#include "API/project.hpp"

using namespace HRICS;
using std::cout;
using std::endl;

void HRICS_run_human_ioc_from_recorded_motion()
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

    global_motionRecorders[0]->loadCSVFolder( foldername + "/human0" );
    global_motionRecorders[1]->loadCSVFolder( foldername + "/human1");

    HRICS::PlayMotion player( global_motionRecorders );
//    for(int i=0;i<int(global_motionRecorders[0]->getStoredMotions().size());i++)
    for(int i=0;i<int(1);i++)
    {
        player.play(i);
    }

    HumanIoc ioc( human2, human1 );
    ioc.setDemos( global_motionRecorders[0]->getStoredMotions() );
    ioc.runLearning();
}

void HRICS_run_human_ioc_evaluation()
{
    Scene* sce = global_Project->getActiveScene();
    Robot* human1 = sce->getRobotByName( "HERAKLES_HUMAN1" );
    Robot* human2 = sce->getRobotByName( "HERAKLES_HUMAN2" );
    if( human1 == NULL || human2 == NULL )
    {
        cout << "No humans HERAKLES in the the scene" << endl;
        return;
    }

    HumanIoc ioc( human2, human1 );
    ioc.loadDemonstrations();
    ioc.runLearning();
}

HumanIoc::HumanIoc( Robot* active, Robot* passive ) : IocEvaluation(active)
{
    nb_demos_ = 10;
    nb_samples_ = 1000;
    nb_way_points_ = 15;
    folder_ = "/home/jmainpri/workspace/move3d/assets/Collaboration/TRAJECTORIES/";

    feature_matrix_name_ = "matlab/features.txt";
    feature_fct_ = new HRICS::HumanTrajCostSpace( active, passive );

    original_vect_ = feature_fct_->getWeights();
    nb_weights_ = original_vect_.size();

    setPlanningGroup();
}

void HumanIoc::setPlanningGroup()
{
    // traj_optim_hrics_human_trajectory_manip_init_joints()
    // Set the planner joints
    active_joints_.clear();
    active_joints_.push_back( 1 );
    active_joints_.push_back( 2 );
    active_joints_.push_back( 3 );
    active_joints_.push_back( 4 );
    active_joints_.push_back( 8 );
    active_joints_.push_back( 9 );
    active_joints_.push_back( 10 );
    active_joints_.push_back( 11 );
    active_joints_.push_back( 12 );

    // Set the active joints (links)
//    active_joints_.clear();
//    active_joints_.push_back( 1 ); // Pelvis
//    active_joints_.push_back( 2 ); // TorsoX
//    active_joints_.push_back( 3 ); // TorsoY
//    active_joints_.push_back( 4 ); // TorsoZ
//    active_joints_.push_back( 8 ); // rShoulderX
//    active_joints_.push_back( 9 ); // rShoulderZ
//    active_joints_.push_back( 10 ); // rShoulderY
//    active_joints_.push_back( 11 ); // rArmTrans
//    active_joints_.push_back( 12 ); // rElbowZ

//    active_joints_.push_back( 13 );
//    active_joints_.push_back( 14 );
//    active_joints_.push_back( 15 );
//    active_joints_.push_back( 16 );
}

void HumanIoc::setDemos( const std::vector<motion_t>& stored_motions )
{
    nb_demos_ = stored_motions.size();
    demos_.clear();
    demos_.resize( nb_demos_ );

    for(int i=0;i<int(demos_.size());i++)
    {
        demos_[i] = getTrajectoryFromMotion( stored_motions[i] );
    }
}

API::Trajectory HumanIoc::getTrajectoryFromMotion( const motion_t& m ) const
{
    API::Trajectory t( robot_ );

    for(int i=0;i<int(m.size());i++)
        t.push_back( m[i].second->copy() );

    t.cutTrajInSmallLP( nb_way_points_-1 );

    return t;
}

void HumanIoc::runLearning()
{
    // Comment it to generate less demos
    nb_demos_ = 10;

    ChompPlanningGroup* plangroup = new ChompPlanningGroup( robot_, active_joints_ );
    std::vector<int> active_dofs = plangroup->getActiveDofs();

    // Create IOC sampling object
    HRICS::Ioc ioc( nb_way_points_, plangroup );

//    demos_[0].getBegin()->equal( *demos_[0].getEnd(), true );

    // Get features of demos
    std::vector<FeatureVect> phi_demo( nb_demos_ );
    for(int i=0;i<nb_demos_;i++)
    {
        demos_[i].cutTrajInSmallLP( nb_way_points_-1 );
        FeatureVect phi = feature_fct_->getFeatureCount( demos_[i] );
        cout << "Feature Demo : " << endl << phi.transpose() << endl;
        ioc.addDemonstration( demos_[i].getEigenMatrix( active_dofs ) );
        cout << "demo : " << endl <<  demos_[i].getEigenMatrix( active_dofs ) << endl;
        phi_demo[i] = phi;
    }

    return;

    // Get features of samples
    ioc.generateSamples( nb_samples_ );
    std::vector< std::vector<API::Trajectory> > samples = ioc.getSamples();
    std::vector< std::vector<FeatureVect> > phi_k( samples.size() );
    for( int d=0;d<int(samples.size());d++)
    {
        for( int i=0;i<int(samples[d].size());i++)
        {
            phi_k[d].push_back( feature_fct_->getFeatureCount( samples[d][i] ) );
            cout << "Feature(" << d << "," <<  i << ") : " << phi_k[d].back().transpose() << endl;
        }
    }

//    saveToMatrix( phi_demo, phi_k );

//    for( int i=0;i<1;i++)
//    {
//        Eigen::VectorXd w = ioc.solve( phi_demo, phi_k );
//        cout << "w : " << w.transpose() << endl;
//    }
}

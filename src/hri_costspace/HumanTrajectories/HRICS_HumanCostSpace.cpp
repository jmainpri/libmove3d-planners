#include "HRICS_HumanCostSpace.hpp"

#include "HRICS_PlayMotion.hpp"
#include "API/project.hpp"

using namespace HRICS;
using std::cout;
using std::endl;

HumanTrajCostSpace* global_humanTrajectoryCostSpace = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Cost Function

double HRICS_getHumanTrajectoryCost(Configuration& q)
{
    return global_humanTrajectoryCostSpace->getCost(q);
}

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

    global_motionRecorders[0]->loadCSVFolder( foldername + "/human0" );
    global_motionRecorders[1]->loadCSVFolder( foldername + "/human1");

    PlayMotion player( global_motionRecorders );
//    for(int i=0;i<int(global_motionRecorders[0]->getStoredMotions().size());i++)
    for(int i=0;i<int(1);i++)
    {
        player.play(i);
    }

    HumanTrajCostSpace cost_space( human1, human2 );
    cost_space.setPassiveTrajectory( global_motionRecorders[1]->getStoredMotions()[0] );
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Human Trajectory Cost Space

HumanTrajCostSpace::HumanTrajCostSpace( Robot* active, Robot* passive ) : human_active_(active), human_passive_(passive)
{
    nb_way_points_ = 20;
}

HumanTrajCostSpace::~HumanTrajCostSpace()
{

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
    vect_stacked.push_back( getDistance() );
    vect_stacked.push_back( getVisibility() );
    vect_stacked.push_back( getLegibility() );
    vect_stacked.push_back( getMuskuloskeletal() );

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

double HumanTrajCostSpace::getCost(Configuration& q) const
{
    return 0.0;
}

FeatureVect HumanTrajCostSpace::getDistance()
{
    FeatureVect vect;
    return vect;
}

FeatureVect HumanTrajCostSpace::getVisibility()
{
    FeatureVect vect;
    return vect;
}

FeatureVect HumanTrajCostSpace::getLegibility()
{
    FeatureVect vect;
    return vect;
}

FeatureVect HumanTrajCostSpace::getMuskuloskeletal()
{
    FeatureVect vect;
    return vect;
}

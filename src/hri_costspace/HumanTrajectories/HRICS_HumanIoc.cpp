#include "HRICS_HumanIoc.hpp"
#include "HRICS_RecordMotion.hpp"
#include "HRICS_PlayMotion.hpp"
#include "API/project.hpp"

using namespace HRICS;
using std::cout;
using std::endl;

void HRICS_run_human_ioc()
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

    int i=0;
    for( i=0;i<int(global_motionRecorders[0]->getStoredMotions().size());i++)
    {
        player.play(i);
    }
}

HumanIoc::HumanIoc()
{
}

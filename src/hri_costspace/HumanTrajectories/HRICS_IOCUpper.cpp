#include "HRICS_IOCUpper.hpp"

#include "API/project.hpp"

#include "planner/cost_space.hpp"

#include <boost/bind.hpp>

using std::cout;
using std::endl;

using namespace HRICS;

IOCUpper* global_IOCUpper = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Initialization

void HRICS_initIverseOptimalControlFramework()
{
    cout << "---------------------------------------------------------------" << endl;
    cout << " HRICS_initIverseOptimalControlFramework " << endl;
    cout << "---------------------------------------------------------------" << endl;

    Scene* sce = global_Project->getActiveScene();

    std::vector<double> size = sce->getBounds();

    Robot* human1 = sce->getRobotByName("HERAKLES_HUMAN1");
    Robot* human2 = sce->getRobotByName("HERAKLES_HUMAN2");

    if( human1 == NULL || human2 == NULL )
    {
        cout << "human or robot not defined" << endl;
        return;
    }

    std::vector<Robot*> humans;

    if( human1 ) humans.push_back(human1);
    if( human2 ) humans.push_back(human2);

    RecordMotion* recorder = new RecordMotion( human1 );

    IOCUpper* upper_ioc = new IOCUpper( humans );

    // GUI global variables
    global_motionRecorder = recorder;
    global_IOCUpper = upper_ioc;

    // Define cost functions
    global_costSpace->addCost( "costHumanTrajecoryCost" , boost::bind(HRICS_getHumanTrajectoryCost, _1) );
    global_costSpace->setCost( "costHumanTrajecoryCost" );
}

IOCUpper::IOCUpper(const std::vector<Robot*>& humans)
{
    m_humans = humans;

    if( !initialize() )
    {
        cout << "ERROR in initialization of " << __func__ << "!!!" << endl;
    }
}

IOCUpper::~IOCUpper()
{
    cout << __func__ << endl;

    delete m_costspace;
    delete m_ioc;
}

bool IOCUpper::initialize()
{
    cout << __func__ << endl;

    if( m_humans.empty() )
    {
        cout << "Humans are not well initialized" << endl;
        return false;
    }

    m_costspace = new HumanTrajCostSpace();
    m_costspace->setActiveHuman( m_humans[0] );
    m_costspace->setPassiveHuman( m_humans[1] );

    m_ioc = new IOCLower();
    m_ioc->setActiveRobot( m_humans[0] );

    return true;
}

void IOCUpper::run()
{
    cout << "Run the Inverse Optimal Control" << endl;
    m_ioc->lauchOptimization();
}

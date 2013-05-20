#include "HRICS_IOCSimulator.hpp"

#include "API/project.hpp"

using std::cout;
using std::endl;

using namespace HRICS;

IOCSimulator::IOCSimulator()
{
    if( !initialize() )
    {
        cout << "ERROR in initialization of " << __func__ << "!!!" << endl;
    }
}

IOCSimulator::~IOCSimulator()
{
    cout << __func__ << endl;

    delete m_costspace;
    delete m_ioc;
}

bool IOCSimulator::initialize()
{
    cout << __func__ << endl;

    Scene* sce = global_Project->getActiveScene();

    Robot* human1 = sce->getRobotByName("HERAKLES_HUMAN1");
    Robot* human2 = sce->getRobotByName("HERAKLES_HUMAN2");

    if( human1 == NULL || human2 == NULL )
    {
        cout << "Humans are not well initialized" << endl;
        return false;
    }

    m_costspace = new HumanTrajCostSpace();
    m_costspace->setActiveHuman( human1 );
    m_costspace->setPassiveHuman( human2 );

    m_ioc = new InverseOptimalControl();
    m_ioc->setActiveRobot( human1 );

    return true;
}

void IOCSimulator::run()
{
    cout << "Run the Inverse Optimal Control" << endl;
    m_ioc->lauchOptimization();
}

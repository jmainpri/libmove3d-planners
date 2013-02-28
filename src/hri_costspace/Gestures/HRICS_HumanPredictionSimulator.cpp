#include "HRICS_HumanPredictionSimulator.hpp"
#include "HRICS_HumanPredictionCostSpace.hpp"

#include "API/project.hpp"
#include "planner/cost_space.hpp"

#include "Graphic-pkg.h"

#include <boost/bind.hpp>

using namespace HRICS;

HumanPredictionSimulator* global_humanPredictionSimulator = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Initialization

void HRICS_initOccupancyPredictionFramework()
{
    std::vector<double> size = global_Project->getActiveScene()->getBounds();

    Robot* robot = global_Project->getActiveScene()->getRobotByName( "PR2_ROBOT" );
    Robot* human = global_Project->getActiveScene()->getRobotByName( "HERAKLES_HUMAN1" );

    if( robot == NULL || human == NULL )
    {
        cout << "human or robot not defined" << endl;
        return;
    }

    RecordMotion* recorder = new RecordMotion( human );

    if( recorder->loadRegressedFromCSV() )
    {
        cout << "Motions loaded successfully" << endl;
    }
    else {
         cout << "Error loading motions" << endl;
         delete recorder;
         return;
    }

    ClassifyMotion* classifier = new ClassifyMotion();

    if( classifier->load_model() )
    {
        cout << "GMMs loaded successfully" << endl;
    }
    else {
         cout << "Error loading GMMs" << endl;
         delete recorder;
         delete classifier;
         return;
    }

    WorkspaceOccupancyGrid* occupancyGrid = new WorkspaceOccupancyGrid( human, 0.05, size, classifier );
    occupancyGrid->setRegressedMotions( recorder->getStoredMotions() );
    occupancyGrid->computeOccpancy();
    occupancyGrid->setClassToDraw(1);

    // Create the prediction costspace
    HumanPredictionCostSpace* predictionSpace = new HumanPredictionCostSpace( robot, occupancyGrid );

    // Create the simulator
    HumanPredictionSimulator* simulator = new HumanPredictionSimulator( classifier );

    // GUI global variables
    global_motionRecorder = recorder;
    global_classifyMotion = classifier;
    global_workspaceOccupancy = occupancyGrid;
    global_humanPredictionCostSpace = predictionSpace;
    global_humanPredictionSimulator = simulator;

    // Define cost functions
    global_costSpace->addCost( "costHumanPredictionOccupancy" , boost::bind(HRICS_getPredictionOccupancyCost, _1) );
    global_costSpace->setCost( "costHumanPredictionOccupancy" );
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Simulator

HumanPredictionSimulator::HumanPredictionSimulator(ClassifyMotion* classifier)
{
    m_classifier = classifier;
}

void HumanPredictionSimulator::loadHumanTrajectory(const motion_t& motion)
{
    m_motion = motion;
}

bool HumanPredictionSimulator::updateMotion()
{
    int end = m_current_traj.cols() + m_increment;

    if( end > int(m_motion.size()) )
    {
        return false;
    }

    m_current_traj.resize( 13, end );

    for (int i=0; i<m_current_traj.cols(); i++)
    {
        confPtr_t q = m_motion[i].second;

        m_current_traj(0,i) = i;        // Time
        m_current_traj(1,i) = (*q)[6];  // Pelvis
        m_current_traj(2,i) = (*q)[7];  // Pelvis
        m_current_traj(3,i) = (*q)[8];  // Pelvis
        m_current_traj(4,i) = (*q)[11]; // Pelvis
        m_current_traj(5,i) = (*q)[12]; // TorsoX
        m_current_traj(6,i) = (*q)[13]; // TorsoY
        m_current_traj(7,i) = (*q)[14]; // TorsoZ

        m_current_traj(8,i) = (*q)[18];  // rShoulderX
        m_current_traj(9,i) = (*q)[19];  // rShoulderZ
        m_current_traj(10,i) = (*q)[20]; // rShoulderY
        m_current_traj(11,i) = (*q)[21]; // rArmTrans
        m_current_traj(12,i) = (*q)[22]; // rElbowZ
    }

    return true;
}

void HumanPredictionSimulator::predictVoxelOccupancy()
{
    std::vector<double> likelihood;
    likelihood = m_classifier->classify_motion( m_current_traj );
//    return std::max_element( likelihood.begin(),likelihood.end()) - likelihood.begin();
}

void HumanPredictionSimulator::run()
{
    while( updateMotion() )
    {
        predictVoxelOccupancy();
        g3d_draw_allwin_active();
    }
}

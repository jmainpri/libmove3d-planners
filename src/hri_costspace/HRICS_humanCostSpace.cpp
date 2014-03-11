//
//  HRICS_humanCostSpace.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 30/11/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "HRICS_humanCostSpace.hpp"

#include "HRICS_Distance.hpp"
#include "HRICS_Visibility.hpp"
#include "HRICS_Natural.hpp"

#include "API/Grids/gridsAPI.hpp"
#include "planner/planEnvironment.hpp"

//#include "move3d-headless.h"

#include "Util-pkg.h"

HRICS::HumanCostSpace* HRICS_humanCostMaps = NULL;

using namespace HRICS;
using namespace std;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

HumanCostSpace::HumanCostSpace()
{

}

//! @brief Initialize tge Human cost space with the list of human in the scene
//! @param The robot, to which this costspace is for
//! @param The humans, which will be associated to the costspace
//! @param costspace, the natural space which is associated to a human
//! @param The cell size, each human will be associated a grid (the cell size is the resolution of the grid)
HumanCostSpace::HumanCostSpace(Robot* rob, std::vector<Robot*> humans, Natural* costspace, double cellSize) 
    : m_Robot(rob) , m_Humans( humans)
{
    m_DistanceSpace = NULL;
    m_VisibilitySpace = NULL;
    m_ReachableSpace = NULL;

    m_PlatformJoint = NULL;

    initElementarySpaces();

    m_ReachableSpace = costspace;

    // The grid are initialized and the cost is computed for each cell
    //initHumanGrids(cellSize);

    if (rob->getName() == "PR2_ROBOT")
    {
        if( !initPr2() )
        {
            cout << "Fail to init pr2 robot in HumanCostSpace" << endl;
            return;
        }
    }

    if (rob->getName() == "JUSTIN_ROBOT")
    {
        if( !initJustin() )
        {
            cout << "Fail to init justin robot in HumanCostSpace" << endl;
            return;
        }
    }

    if (rob->getName() == "GREY_TAPE")
    {
        if( !initGreyTape() )
        {
            cout << "Fail to init for visball" << endl;
        }
    }
}

//! @brief Destructor of the costspace
//! all grids are deleted
HumanCostSpace::~HumanCostSpace()
{
    deleteHumanGrids();
    deleteElementarySpaces();
}

//! @brief Initializes the elementary costspaces (Distance,Visibility,...)
//! Each elemetary costspace holds the method used to compute the cost associated
//! to the propriety of the costspace
bool HumanCostSpace::initElementarySpaces()
{
    deleteElementarySpaces();

    if(m_Humans.empty())
    {
        cout << "Can not initialize elementary spazes (Humans is empty)" << endl;
        return false;
    }

    m_DistanceSpace = new Distance(m_Robot,m_Humans);
    m_VisibilitySpace = new Visibility(m_Humans[0]);
    m_ReachableSpace = NULL;
    return true;
}

//! @brief Delete all elementary costspace
void HumanCostSpace::deleteElementarySpaces()
{
    if(m_DistanceSpace)
        delete m_DistanceSpace;

    if(m_VisibilitySpace)
        delete m_VisibilitySpace;

    //  if(m_ReachableSpace)
    //    delete m_ReachableSpace;
}

//! @brief (Re) compute the cell cost of all grids
void HumanCostSpace::computeAllCellCost()
{
    for ( int i=0; i<int(m_Grids.size()); i++)
    {
        m_Grids[i]->computeAllCellCost();
    }
}

//! @brief Initialize the Human Grids composed of the different elementary costspace
//! @param The cell size, the resolution of the grids
//! The human grid computation relies on the elementary costspaces
//! Each human is associated one grid containing all propieties
bool HumanCostSpace::initHumanGrids(double cellsize)
{
    deleteHumanGrids();

    // Create 3 by 3 boxes around the human
    vector<double>  envsize(6);
    envsize[0] = -3; envsize[1] = 3;
    envsize[2] = -3; envsize[3] = 3;
    envsize[4] = -3; envsize[5] = 3;

    for ( int i=0; i<int(m_Humans.size()); i++)
    {
        m_Grids.push_back( new AgentGrid( cellsize, envsize, m_Humans[i], m_DistanceSpace, m_VisibilitySpace, m_ReachableSpace) );
    }

    computeAllCellCost();
    API_activeGrid = m_Grids[0];

    return true;
}

//! @brief Deletes all grids
void HumanCostSpace::deleteHumanGrids()
{
    API_activeGrid = NULL;

    for ( int i=0; i<int(m_Grids.size()); i++)
    {
        delete m_Grids[i];
    }
    m_Grids.clear();
}

AgentGrid* HumanCostSpace::getAgentGrid(Robot* agent)
{
    for ( int i=0; i<int(m_Grids.size()); i++)
    {
        if( m_Grids[i]->getRobot()->getName() == agent->getName() )
            return m_Grids[i];
    }
    return NULL;
}

//! @brief Robot specific initializer
bool HumanCostSpace::initGreyTape()
{
    m_CostJoints.clear();
    m_CostJoints.push_back( m_Robot->getJoint(1) );
    return true;
}

//! @brief Robot specific initializer
//! For the PR2 a fixed number of points are chosen on the kinematic
//! structure of the robot to qualify the robot configuration
bool HumanCostSpace::initPr2()
{
    m_CostJoints.clear();

    //  m_CostJoints.push_back( m_Robot->getJoint(1) );

    //  for (int i=0; i<int(m_Robot->getNumberOfJoints()); i++)
    //  {
    //    string name = m_Robot->getJoint(i)->getName();
    //
    //    if(name == "virtual_object_left" ||
    //       name == "virtual_object_right" ||
    //       name == "J0" )
    //      continue;
    //
    //    m_CostJoints.push_back( m_Robot->getJoint(i) );
    //    cout << "Add joint : " << m_CostJoints.back()->getName() << endl;
    //  }

    switch ( ENV.getInt(Env::setOfActiveJoints) )
    {
    case 0:
        m_planning_type = NAVIGATION;
        break;
    case 1:
        m_planning_type = MANIPULATION;
        break;
    case 2:
        m_planning_type = MOBILE_MANIP;
        break;
    default:
        cout << "Planner type not implemented" << endl;
        break;
    }

    if( m_planning_type == NAVIGATION )
    {
        for (int i=0; i<int(m_Robot->getNumberOfJoints()); i++)
        {
            string name = m_Robot->getJoint(i)->getName();

            if(name == "platformJoint" ||
                    name == "Torso")
            {
                m_CostJoints.push_back( m_Robot->getJoint(i) );
                cout << "Add joint : " << m_CostJoints.back()->getName() << endl;
            }
        }
    }

    if( m_planning_type == MANIPULATION )
    {
        for (int i=0; i<int(m_Robot->getNumberOfJoints()); i++)
        {
            string name = m_Robot->getJoint(i)->getName();

            if(name == "right-Arm4" ||
                    name == "right-Arm5" ||
                    name == "right-Arm6" ||
                    name == "right-Arm7" ||
                    name == "fingerJointGripper_0")
            {
                m_CostJoints.push_back( m_Robot->getJoint(i) );
                cout << "Add joint : " << m_CostJoints.back()->getName() << endl;
            }
        }
    }

    if(  m_planning_type == MOBILE_MANIP )
    {
        for (int i=0; i<int(m_Robot->getNumberOfJoints()); i++)
        {
            string name = m_Robot->getJoint(i)->getName();

            if(name == "platformJoint" ||
                    name == "Torso" ||
                    //name == "pan_cam" ||
                    //name == "right-Arm3" ||
                    name == "right-Arm4" ||
                    name == "right-Arm5" ||
                    name == "right-Arm6" ||
                    name == "right-Arm7" ||
                    name == "fingerJointGripper_0" ||
                    name == "left-Arm6"  ||
                    name == "left-Arm7" )
            {
                m_CostJoints.push_back( m_Robot->getJoint(i) );
                cout << "Add joint : " << m_CostJoints.back()->getName() << endl;
            }
        }
    }
    cout << m_CostJoints.size() << " nb of cost joints" << endl;

    m_PlatformJoint = m_Robot->getJoint("platformJoint");
    return true;
}

//! @brief Robot specific initializer
//! For the JUSTIN a fixed number of points are chosen on the kinematic
//! structure of the robot to qualify the robot configuration
bool HumanCostSpace::initJustin()
{
    m_CostJoints.clear();

    switch ( ENV.getInt(Env::setOfActiveJoints) )
    {
    case 0:
        m_planning_type = NAVIGATION;
        break;
    case 1:
        m_planning_type = MANIPULATION;
        break;
    case 2:
        m_planning_type = MOBILE_MANIP;
        break;
    default:
        cout << "Planner type not implemented" << endl;
        break;
    }

    if( m_planning_type == NAVIGATION )
    {
        for (int i=0; i<int(m_Robot->getNumberOfJoints()); i++)
        {
            string name = m_Robot->getJoint(i)->getName();

            if(name == "Platform" ||
                    name == "base2")
            {
                m_CostJoints.push_back( m_Robot->getJoint(i) );
                cout << "Add joint : " << m_CostJoints.back()->getName() << endl;
            }
        }
    }

    if( m_planning_type == MANIPULATION )
    {
        for (int i=0; i<int(m_Robot->getNumberOfJoints()); i++)
        {
            string name = m_Robot->getJoint(i)->getName();

            if(/*name == "RightArm1" ||
                         name == "RightArm2" ||
                         name == "RightArm3" ||
                         name == "RightArm4" ||
                         name == "RightArm5" ||
                         name == "RightArm6" ||
                         name == "RightArm7" ||
                         name == "RightWrist"
                          */
                    //         name == "LeftArm1" ||
                    //         name == "LeftArm2" ||
                    //         name == "LeftArm3" ||
                    name == "LeftArm4" ||
                    name == "LeftArm5" ||
                    name == "LeftArm6" ||
                    //         name == "LeftArm7" ||
                    name == "LeftWrist")
            {
                m_CostJoints.push_back( m_Robot->getJoint(i) );
                cout << "Add joint : " << m_CostJoints.back()->getName() << endl;
            }
        }
    }

    if(  m_planning_type == MOBILE_MANIP )
    {
        for (int i=0; i<int(m_Robot->getNumberOfJoints()); i++)
        {
            string name = m_Robot->getJoint(i)->getName();

            if(name == "Platform" ||
                    name == "base2" ||
                    name == "LeftArm3" ||
                    name == "LeftArm4" ||
                    name == "LeftArm5" ||
                    name == "LeftArm6" ||
                    name == "LeftArm7" ||
                    name == "LeftWrist" )
            {
                m_CostJoints.push_back( m_Robot->getJoint(i) );
                cout << "Add joint : " << m_CostJoints.back()->getName() << endl;
            }
        }
    }
    cout << m_CostJoints.size() << " nb of cost joints" << endl;

    m_PlatformJoint = m_Robot->getJoint("Platform");
    return true;
}

double HumanCostSpace::groupingCost()
{
    double cost = 0.0;

    if( m_PlatformJoint == NULL )
    {
        return 0.0;
    }

    Eigen::Vector3d pos3d = m_PlatformJoint->getVectorPos();
    Eigen::Vector2d pos2d;
    pos2d[0] = pos3d[0];
    pos2d[1] = pos3d[1];

    for (int i=0; i<int(m_CostJoints.size()); i++)
    {
        Eigen::Vector3d posjnt3d = m_CostJoints[i]->getVectorPos();
        Eigen::Vector2d posjnt2d;
        posjnt2d[0] = posjnt3d[0];
        posjnt2d[1] = posjnt3d[1];

        cost += (( posjnt2d - pos2d ).norm()/0.8);
    }

    return cost;
}

void HumanCostSpace::saveAgentGrids()
{
    for (unsigned int i=0; i<m_Grids.size(); i++)
    {
        std::ostringstream oss; oss << i;

        std::string home(getenv("HOME_MOVE3D"));
        std::string filename = "/statFiles/Cost3DGrids/human_grids_" + oss.str() + ".grid";

        filename = home + filename;

        m_Grids[i]->writeToXmlFile( filename );
    }
}

void HumanCostSpace::loadAgentGrids(const std::string& filename)
{
    m_Grids.clear();

    AgentGrid* agentGrid = new AgentGrid( m_Humans[0], m_DistanceSpace, m_VisibilitySpace, m_ReachableSpace );

    bool reading_OK=false;

    for (int i=0; (i<5)&&(!reading_OK) ; i++)
    {
        cout << "Reading agentGrid" << endl;
        reading_OK = agentGrid->loadFromXmlFile( filename );
    }

    m_Grids.push_back( agentGrid );
    API_activeGrid = agentGrid;

    agentGrid->computeCellVectors();
    agentGrid->computeRadius();
}

//! @brief This function tests how many configuration test can be done 
//! The test uses a real time chrono
void HumanCostSpace::testCostFunction()
{ 
    shared_ptr<Configuration> q;

    bool first_loop=true;
    unsigned int iter=0;
    double t=0.0,t_init=0.0;

    ChronoTimeOfDayOn();

    while( (t - t_init) < 10.0 )
    {
        q = m_Robot->shoot();

        //    if( !q->isInCollision() )
        //    {
        this->getCost(*q);
        //    }

        //g3d_draw_allwin_active();

        ChronoTimeOfDayTimes(&t);

        if(first_loop)
        {
            t_init = t;
            first_loop = false;
        }
        iter++;
    }

    ChronoTimeOfDayOff();

    cout << "iter : " << iter << " in " << (t - t_init) << " sec. " << endl;
    cout << "cost per sec : " << double(iter)/(t - t_init) << endl;
}

void HumanCostSpace::drawDistances()
{
    vector<double> costs(4);
    costs[0] = 0.0;
    costs[1] = 0.0;
    costs[2] = 0.0;
    costs[3] = 0.0;

    //cout << "-----------------------------------" << endl;
    for (int i=0; i<int(m_Grids.size()); i++)
    {
        for (int j=0; j<int(m_CostJoints.size()); j++)
        {
            Eigen::Vector3d pos = m_CostJoints[j]->getVectorPos();
            cout << "Joint name : " << m_CostJoints[j]->getName() << " , " ;
            m_Grids[i]->getCompleteCellCostAt( pos, costs );

            g3d_drawSphere( pos(0), pos(1), pos(2), 0.10 );
        }
    }
}

void HumanCostSpace::drawReachableGrid()
{
    if( m_ReachableCells.empty() )
        return;

    m_ReachableSpace->getGrid()->drawVector( m_ReachableCells );
}

//! Returns a vector of reachable points for object handovers
//! the vector is sorted relativly to the HRI constraints
//! @arm_type 0:noarm, 1:right, 2l:eft
bool HumanCostSpace::getHandoverPointList(std::vector<Eigen::Vector3d>& points, bool recompute_cells, int arm_type)
{ 
    if( recompute_cells )
    {
        std::vector<NaturalCell*> reachable_cells;

        if( arm_type==0) {
            reachable_cells = m_ReachableSpace->getGrid()->getAllReachableCells();
        }
        if( arm_type==1) {
            reachable_cells = m_ReachableSpace->getGrid()->getAllReachableCellsOneArm(true);
        }
        if( arm_type==2) {
            reachable_cells = m_ReachableSpace->getGrid()->getAllReachableCellsOneArm(false);
        }

        double max = ENV.getDouble(Env::Kdistance)+ENV.getDouble(Env::Kvisibility)+ENV.getDouble(Env::Kreachable);

        m_ReachableCells.clear();
        for(int i=0;i<int(reachable_cells.size());i++)
        {
            Eigen::Vector3d point = reachable_cells[i]->getWorkspacePoint();

            double d_cost = ENV.getDouble(Env::Kdistance)*m_DistanceSpace->getWorkspaceCost(point);
            double v_cost = ENV.getDouble(Env::Kvisibility)*m_VisibilitySpace->getWorkspaceCost(point);
            double r_cost = ENV.getDouble(Env::Kreachable)*m_ReachableSpace->getWorkspaceCost(point);

            m_ReachableCells.push_back( make_pair( (d_cost+v_cost+r_cost)/max, reachable_cells[i] ) );
        }

        sort( m_ReachableCells.begin(), m_ReachableCells.end() );
    }

    if( m_ReachableCells.empty() ) {
        return false;
    }

    points.resize(m_ReachableCells.size());
    for(int i=0;i<int(points.size());i++) {
        points[i] = m_ReachableCells[i].second->getWorkspacePoint();
        //cout << "sorted cell[" << i << "] : " << sorted_cells[i].first << endl;
    }
    return true;
}

//! Returns the HRI cost of the point
//! that on the first joint of the configuration
double HumanCostSpace::getPointCost(Configuration& q)
{
    double cost=0.0;
    Eigen::Vector3d pos;
    pos[0] = q[6];
    pos[1] = q[7];
    pos[2] = q[8];

    for (int i=0; i<int(m_Grids.size()); i++)
    {
        cost += m_Grids[i]->getCellCostAt( pos );
    }
    return cost;
}

//! @brief Cost computation for a given configuration
//! @param A robot configuration
//! Sums the cost of each each joint point
//! The class maintains a vector of joint pointer that are
//! used in the cost computation
double HumanCostSpace::getCost(Configuration& q)
{
    m_Robot->setAndUpdate(q);

    double cost=0.0;

    //cout << "Total : " ;
    for (int i=0; i<int(m_Grids.size()); i++)
    {
        for (int j=0; j<int(m_CostJoints.size()); j++)
        {
            cost += m_Grids[i]->getCellCostAt( m_CostJoints[j]->getVectorPos() );
            //cout << " " << cost;
        }
    }
    //cout << endl;
    cost = 10*cost/double(m_CostJoints.size());
    cost += ENV.getDouble(Env::Knatural)*groupingCost();

    return cost;
}

//! @brief Complete cost of configuration
//! @param Configuration of the robot
double HumanCostSpace::getCompleteCost(Configuration& q, vector<double>& cost_sum)
{
    cost_sum.resize(6);
    cost_sum[0] = 0.0;
    cost_sum[1] = 0.0;
    cost_sum[2] = 0.0;
    cost_sum[3] = 0.0;
    cost_sum[4] = 0.0;
    cost_sum[5] = 0.0;

    m_Robot->setAndUpdate(q);

    vector<double> costs(4);
    costs[0] = 0.0;
    costs[1] = 0.0;
    costs[2] = 0.0;
    costs[3] = 0.0;

    //cout << "Combine : " ;
    for (int i=0; i<int(m_Grids.size()); i++)
    {
        for (int j=0; j<int(m_CostJoints.size()); j++)
        {
            m_Grids[i]->getCompleteCellCostAt( m_CostJoints[j]->getVectorPos(), costs );

            cost_sum[0] += costs[0]; // Distance
            cost_sum[1] += costs[1]; // Visibility
            cost_sum[2] += costs[2]; // Reachability
            cost_sum[3] += costs[3]; // Combine
            //cout << " " << cost_sum[3];
        }
    }

    cost_sum[4] = groupingCost(); // Grouping
    cost_sum[5] = getCost(q);     // Total

    return cost_sum[5];
}


/*
 *  HRICS_CostSpace.h
 *  BioMove3D
 *
 *  Created by Mamoun Gharbi on 20/04/11.
 *  Copyright 2009 magharbi@laas.fr All rights reserved.
 *
 */

#include "HRICS_otpmotionpl.hpp"
#include "Grid/HRICS_Grid.hpp"

#include "P3d-pkg.h"
#include "move3d-headless.h"
#include "Planner-pkg.h"

#include "API/Trajectory/smoothing.hpp"
#include "planEnvironment.hpp"
#include "time.h"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

extern HRICS::HumanAwareMotionPlanner*		HRICS_MotionPLConfig;
extern string global_ActiveRobotName;
extern API::TwoDGrid* API_activeRobotGrid;

extern Eigen::Vector3d current_WSPoint;
extern pair<double,Eigen::Vector3d > current_cost;

std::vector<Eigen::Vector3d> OTPList;
int ConfigHR::index = 0;

OTPMotionPl::OTPMotionPl() : HumanAwareMotionPlanner() , m_PathExist(false) , m_HumanPathExist(false), m_noPath(true), m_pathIndex(-1)
{
    cout << "New OTPMotionPl HRI planner" << endl;

    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);

        if(name.find(global_ActiveRobotName) != string::npos )
        {
            _Robot = new Robot(XYZ_ENV->robot[i]);
            cout << "Robot is " << name << endl;
        }

        if(name.find("HUMAN") != string::npos )
        {
            m_Human = new Robot(XYZ_ENV->robot[i]);
            cout << "Humans is " << name << endl;
        }
    }

    initAll();
    initHumanCenteredGrid(0.1);
}

OTPMotionPl::OTPMotionPl(Robot* R, Robot* H) : HumanAwareMotionPlanner() , m_Human(H) , m_PathExist(false) , m_HumanPathExist(false), m_noPath(true), m_pathIndex(-1)
{
    this->setRobot(R);
    initAll();
}

OTPMotionPl::~OTPMotionPl()
{
    delete m_2DGrid;
}

void OTPMotionPl::initAll()
{
    // initCostSpace
    m_EnvSize.resize(4);
    m_EnvSize[0] = XYZ_ENV->box.x1; m_EnvSize[1] = XYZ_ENV->box.x2;
    m_EnvSize[2] = XYZ_ENV->box.y1; m_EnvSize[3] = XYZ_ENV->box.y2;

//    m_2DGrid = new EnvGrid(ENV.getDouble(Env::PlanCellSize),m_EnvSize,false);

    m_2DGrid = new EnvGrid(0.2,m_EnvSize,false,_Robot,m_Human);
//    m_2DGrid->setRobot(_Robot);
//    m_2DGrid->setHuman(m_Human);

	// initDistance
	vector<Robot*> m_Humans;
	m_Humans.push_back(m_Human);
	m_DistanceSpace = new Distance(_Robot,m_Humans);

	if (_Robot)
	{
		cout << "Robot " << _Robot->getName() << endl;
		cout << "Robot get struct " << _Robot->getRobotStruct() << endl;
	}

	m_DistanceSpace->parseHumans();

	// initVisibility
	m_VisibilitySpace = new Visibility(m_Human);

    // init pos
    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
    m_Human->setInitialPosition(*q_human_cur);

    // Init Manipulation Planner
    m_ManipPl = new ManipulationPlanner( _Robot->getRobotStruct() );
}

void OTPMotionPl::initHumanCenteredGrid(double cellsize)
{
    m_EnvSize.resize(4);
    m_EnvSize[0] = XYZ_ENV->box.x1; m_EnvSize[1] = XYZ_ENV->box.x2;
    m_EnvSize[2] = XYZ_ENV->box.y1; m_EnvSize[3] = XYZ_ENV->box.y2;

    m_2DHumanCenteredGrid = new EnvGrid(cellsize,m_EnvSize,true, _Robot, m_Human);
    m_2DHumanCenteredGrid->setRobot(_Robot);
    m_2DHumanCenteredGrid->setHuman(m_Human);

    API_activeRobotGrid = m_2DHumanCenteredGrid;
}

/**
  * Takes the robot initial config and calls the solve A*
  * to compute the 2D path
  */
bool OTPMotionPl::computeAStarIn2DGrid()
{
    ENV.setBool(Env::drawOTPTraj,false);

    shared_ptr<Configuration> config = _Robot->getInitialPosition();

//    config->print();

    Vector2d pos;

    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

    pos[0] = config->at(firstIndexOfRobotDof + 0);
    pos[1] = config->at(firstIndexOfRobotDof + 1);

    EnvCell* startCell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(pos));
    Vector2i startCoord = startCell->getCoord();

    cout << "Start Pos = (" <<
            pos[0] << " , " <<
            pos[1] << ")" << endl;

    cout << "Start Coord = (" <<
            startCoord[0] << " , " <<
            startCoord[1] << ")" << endl;

    EnvState* start = new EnvState(
            startCell,
            m_2DGrid);

    config = _Robot->getGoTo();

    pos[0] = config->at(firstIndexOfRobotDof + 0);
    pos[1] = config->at(firstIndexOfRobotDof + 1);

    EnvCell* goalCell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(pos));
    Vector2i goalCoord = goalCell->getCoord();

    cout << "Goal Pos = (" <<
            pos[0] << " , " <<
            pos[1] << ")" << endl;

    cout << "Goal Coord = (" <<
            goalCoord[0] << " , " <<
            goalCoord[1] << ")" << endl;

    if( startCoord == goalCoord )
    {
        cout << " no planning as cells are identical" << endl;
        return false;
    }

    EnvState* goal = new EnvState(
            goalCell,
            m_2DGrid);

    solveAStar(start,goal,false);

    double SumOfCost= 0.0;
    for(unsigned int i=0; i< m_2DPath.size() ; i++ )
    {
        SumOfCost +=  dynamic_cast<EnvCell*>(m_2DCellPath[i])->getCost();
    }

    // testing trajectory
    shared_ptr<Configuration> q_cur = _Robot->getInitialPosition();
    shared_ptr<Configuration> q = _Robot->getInitialPosition();

    cout << " SumOfCost = "  << SumOfCost << endl;
    cout << " The Robot will move throuth the followings cells " << endl;

    for (unsigned int i = 0; i < m_2DCellPath.size(); i++)
    {
        Eigen::Vector2d center = m_2DCellPath.at(i)->getCenter();
//        cout << i << " = " << endl << "abs Pos : "<< endl << center << endl << "coord : " << endl << dynamic_cast<EnvCell*>(m_2DCellPath.at(i))->getCoord() << endl << "---------------" << endl;

        (*q)[firstIndexOfRobotDof + 0] = center[0];
        (*q)[firstIndexOfRobotDof + 1] = center[1];
        _Robot->setAndUpdate(*q);
        if (_Robot->isInCollision())
        {
            m_2DCellPath.resize(i);
            m_2DPath.resize(i);
            break;
        }
    }
    _Robot->setAndUpdate(*q_cur);

    API::TwoDCell* lastReachedCell = m_2DCellPath.at(m_2DCellPath.size()-1);

    if (lastReachedCell != goalCell)
    {
        cout << "The cell that the robot should access to is unreacheable." <<
                "Motion Planing for the human." << endl;

        config = m_Human->getInitialPosition();

//        config->print();

        int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();

        pos[0] = config->at(firstIndexOfHumanDof + 0);
        pos[1] = config->at(firstIndexOfHumanDof + 1);

        EnvCell* startHumanCell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(pos));
        Vector2i startHumanCoord = startHumanCell->getCoord();

        cout << "Start Human Pos = (" <<
                pos[0] << " , " <<
                pos[1] << ")" << endl;

        cout << "Start Human Coord = (" <<
                startHumanCoord[0] << " , " <<
                startHumanCoord[1] << ")" << endl;

        EnvState* startHuman = new EnvState(
                startHumanCell,
                m_2DGrid);

        EnvCell* goalHumanCell = dynamic_cast<EnvCell*>(lastReachedCell);//(m_2DGrid->getCell(pos));
        Vector2i goalHumanCoord = goalHumanCell->getCoord();

        cout << "Goal Pos = (" <<
                goalHumanCell->getCenter()[0] << " , " <<
                goalHumanCell->getCenter()[1] << ")" << endl;

        cout << "Goal Coord = (" <<
                goalHumanCoord[0] << " , " <<
                goalHumanCoord[1] << ")" << endl;

        if( startHumanCoord == goalHumanCoord )
        {
            cout << " no planning as cells are identical" << endl;
            return false;
        }

        EnvState* goalHuman = new EnvState(
                goalHumanCell,
                m_2DGrid);

        solveAStar(startHuman,goalHuman, true);

        double SumOfCost= 0.0;
        for(unsigned int i=0; i< m_2DPath.size() ; i++ )
        {
            SumOfCost +=  dynamic_cast<EnvCell*>(m_2DCellPath[i])->getCost();
        }

        // testing trajectory
        shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
        shared_ptr<Configuration> q_human = _Robot->getCurrentPos();

        cout << " SumOfCost = "  << SumOfCost << endl;
        for (unsigned int i = 0; i < m_2DHumanCellPath.size(); i++)
        {
            Eigen::Vector2d center = m_2DHumanCellPath.at(i)->getCenter();
//            cout << i << " = " << endl << "abs Pos : "<< endl << center << endl << "coord : " << endl << dynamic_cast<EnvCell*>(m_2DHumanCellPath.at(i))->getCoord() << endl << "---------------" << endl;

            (*q_human)[firstIndexOfHumanDof + 0] = center[0];
            (*q_human)[firstIndexOfHumanDof + 1] = center[1];
            m_Human->setAndUpdate(*q_human);
            if (m_Human->isInCollision())
            {
                m_2DHumanCellPath.resize(i);
                m_2DHumanPath.resize(i);
                break;
            }
        }
        m_Human->setAndUpdate(*q_human_cur);

    }
    else
    {
        m_2DHumanPath.clear();

        m_2DHumanCellPath.clear();
    }

    return true;
}

void OTPMotionPl::solveAStar(EnvState* start, EnvState* goal, bool isHuman)
{
    if (!isHuman)
    {
        m_2DPath.clear();

        m_2DCellPath.clear();

    //    shared_ptr<Configuration> config = _Robot->getCurrentPos();

        /*
        * Change the way AStar
        * is computed to go down
        */
        if( start->getCell()->getCost() < goal->getCell()->getCost() )
        {
            API::AStar* search = new API::AStar(start);
            vector<API::State*> path = search->solve(goal);

            if(path.size() == 0 )
            {

                m_2DPath.clear();
                m_2DCellPath.clear();
                m_PathExist = false;
                m_noPath = true;
                return;

            }

            for (unsigned int i=0;i<path.size();i++)
            {
                API::TwoDCell* cell = dynamic_cast<EnvState*>(path[i])->getCell();
                m_2DPath.push_back( cell->getCenter() );
                m_2DCellPath.push_back( cell );
            }
        }
        else
        {
            API::AStar* search = new API::AStar(goal);
            vector<API::State*> path = search->solve(start);

            if(path.size() == 0 )
            {
                m_2DPath.clear();
                m_2DCellPath.clear();
                m_PathExist = false;
                m_noPath = true;
                return;
            }


            for (int i=path.size()-1;i>=0;i--)
            {
                EnvCell* cell = dynamic_cast<EnvState*>(path[i])->getCell();
                m_2DPath.push_back( cell->getCenter() );
                m_2DCellPath.push_back( cell );
            }
        }
        m_noPath = false;
        m_PathExist = true;
        return;
    }
    else
    {
        m_2DHumanPath.clear();

        m_2DHumanCellPath.clear();

    //    shared_ptr<Configuration> config = _Robot->getCurrentPos();

        /*
        * Change the way AStar
        * is computed to go down
        */
        if( start->getCell()->getCost() < goal->getCell()->getCost() )
        {
            API::AStar* search = new API::AStar(start);
            vector<API::State*> path = search->solve(goal);

            if(path.size() == 0 )
            {

                m_2DHumanPath.clear();
                m_2DHumanCellPath.clear();
                m_HumanPathExist = false;
                m_noPath = true;
                return;

            }

            for (unsigned int i=0;i<path.size();i++)
            {
                API::TwoDCell* cell = dynamic_cast<EnvState*>(path[i])->getCell();
                m_2DHumanPath.push_back( cell->getCenter() );
                m_2DHumanCellPath.push_back( cell );
            }
        }
        else
        {
            API::AStar* search = new API::AStar(goal);
            vector<API::State*> path = search->solve(start);

            if(path.size() == 0 )
            {
                m_2DHumanPath.clear();
                m_2DHumanCellPath.clear();
                m_HumanPathExist = false;
                m_noPath = true;
                return;
            }

            for (int i=path.size()-1;i>=0;i--)
            {
                EnvCell* cell = dynamic_cast<EnvState*>(path[i])->getCell();
                m_2DHumanPath.push_back( cell->getCenter() );
                m_2DHumanCellPath.push_back( cell );
            }
        }
        m_noPath = false;
        m_HumanPathExist = true;
        return;
    }
}

/**
  * Draws the 3D path as a yellow line for robot and green one for human
  */
void OTPMotionPl::draw2dPath()
{
    if( m_PathExist)
    {
//        cout << "Drawing 2D path" << endl;
        for(unsigned int i=0;i<m_2DPath.size()-1;i++)
        {
            glLineWidth(3.);
            g3d_drawOneLine(m_2DPath[i][0],      m_2DPath[i][1],      0.4,
                            m_2DPath[i+1][0],    m_2DPath[i+1][1],    0.4,
                            Yellow, NULL);
            glLineWidth(1.);
        }
    }

    if( m_HumanPathExist )
    {
//        cout << "Drawing 2D path" << endl;
        for(unsigned int i=0;i<m_2DHumanPath.size()-1;i++)
        {
            glLineWidth(3.);
            g3d_drawOneLine(m_2DHumanPath[i][0],      m_2DHumanPath[i][1],      0.4,
                            m_2DHumanPath[i+1][0],    m_2DHumanPath[i+1][1],    0.4,
                            Green, NULL);
            glLineWidth(1.);
        }
    }
}

bool OTPMotionPl::OTPonly(int th)
{

	m_Human->setAndUpdate(*m_Human->getInitialPosition());
	Vector3d transfPoint;
	if ( m_ReachableSpace == NULL )
	{
		cout << "No ReachableSpace cost space initialized" << endl;
		return false;
	}

	vector< pair<double,Vector3d > > ReachablePoints = m_ReachableSpace->getReachableWSPoint();
	vector< pair<double,Vector3d > > costs;

        // Fills each pair with
        // the cost of the sum of each criteria
    for (unsigned int i=0; i<ReachablePoints.size(); i++)
    {
        double KDist = ENV.getDouble(Env::Kdistance);
        double KVisi = ENV.getDouble(Env::Kvisibility);
        double KReac = ENV.getDouble(Env::Kreachable);

        Vector3d point   = ReachablePoints[i].second;

        double CostDist = m_DistanceSpace->getWorkspaceCost(point);
        double CostReach = ReachablePoints[i].first;
        double CostVisib = m_VisibilitySpace->getCost(point);

        Vector3d localCosts;
        localCosts[0] = CostDist;
        localCosts[1] = CostReach;
        localCosts[2] = CostVisib;

        ReachablePoints[i].first = (KDist*CostDist + KVisi*CostVisib + KReac*CostReach )/ 3;

		pair<double,Vector3d > p;
		p.first = ReachablePoints[i].first;
		p.second = localCosts;

		costs.push_back( p );
	}

	transfPoint = ReachablePoints[th].second;

	if (m_ReachableSpace->computeIsReachableAndMove(transfPoint,false))
	{
		current_cost.first = ReachablePoints[th].first;
		current_cost.second = costs[th].second;
		current_WSPoint = transfPoint;

		cout << "the OTP is : \n" << transfPoint << endl;

		computeProx(transfPoint,1);
		computeHight(transfPoint,1);
		computeAngle(transfPoint,1);

		return true;
	}

	m_Human->setAndUpdate(*m_Human->getInitialPosition());
//	placeRobot();
	return false;
}

bool OTPMotionPl::simpleComputeBaseAndOTP()
{
  cout << "Simple Compute the OTP" << endl;
  
  m_2DGrid->setCellsToblankCost();
  
  shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
  shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
  
  m_Human->setInitialPosition(*q_human_cur);
  _Robot->setInitialPosition(*q_robot_cur);
  
  cout << "This function is separeted in ? parts : " << endl;
  cout << "First Part" << endl;
  
  cout << "Second part : Motion Planing for robot (and human if nessessary)" << endl;
  
  //    ENV.setDouble(Env::robotMaximalDistFactor,0.0);
  initHumanCenteredGrid(0.1);
  vector<EnvCell*> sortedCells = m_2DHumanCenteredGrid->getSortedCells();
  
  Vector2d point;


  Vector2d HumanPos;

  HumanPos[0] = (*q_human_cur)[6];
  HumanPos[1] = (*q_human_cur)[7];


  
  for (unsigned int i=0; i < sortedCells.size(); i++)
  {
      shared_ptr<Configuration> q_robot_tmp = _Robot->getCurrentPos();

      point[0] = sortedCells[i]->getCenter()[0];
      point[1] = sortedCells[i]->getCenter()[1];
      Vector2d gazeDirect = HumanPos - point;

      (*q_robot_tmp)[6] = sortedCells[i]->getCenter()[0];
      (*q_robot_tmp)[7] = sortedCells[i]->getCenter()[1];
      (*q_robot_tmp)[11] = atan2(gazeDirect.y(),gazeDirect.x());

      if(!q_robot_tmp->isInCollision())
      {
          if (computeUpBodyOpt())
          {
//              ComputePR2Gik();
            _Robot->setGoTo(*_Robot->getCurrentPos());
//            _Robot->setAndUpdate(*q_robot_cur);

            m_Human->setGoTo(*m_Human->getCurrentPos());
//            m_Human->setAndUpdate(*q_human_cur);

            return true;
          }
      }
  }
  return false;
}

bool OTPMotionPl::ComputePR2Gik()
{
    initPR2GiveConf();
    cout << "Workspace::computePR2GIK()" << endl;

    int armId = 0;
    ArmManipulationData& armData = (*_Robot->getRobotStruct()->armManipulationData)[armId];
    ManipulationConfigs manipConf(_Robot->getRobotStruct());
    gpGrasp grasp;
    double confCost = -1;

    vector<double> target(6);
    target.at(0) = current_WSPoint(0);
    target.at(1) = current_WSPoint(1);
    target.at(2) = current_WSPoint(2);
    target.at(3) = P3D_HUGE;
    target.at(4) = P3D_HUGE;
    target.at(5) = P3D_HUGE;

    double* q = manipConf.getFreeHoldingConf(NULL, armId, grasp, armData.getCcCntrt()->Tatt, confCost, target, NULL);
    _Robot->activateCcConstraint();
    if (q != NULL)
    {
        shared_ptr<Configuration> m_q = shared_ptr<Configuration>(
                                              new Configuration(_Robot,p3d_copy_config(_Robot->getRobotStruct(),q)));
        _Robot->setAndUpdate( *m_q );
        return true;
    }
    else
    {
        initPR2GiveConf();
        return false;
    }
}

bool OTPMotionPl::computeObjectTransfertPoint()
{
    cout << "Compute the OTP" << endl;

    m_2DGrid->setCellsToblankCost();

    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();

    m_Human->setInitialPosition(*q_human_cur);
    _Robot->setInitialPosition(*q_robot_cur);

    cout << "This function is separeted in ? parts : " << endl;
    cout << "First Part" << endl;

    cout << "Second part : Motion Planing for robot (and human if nessessary)" << endl;

//    ENV.setDouble(Env::robotMaximalDistFactor,0.0);
    initHumanCenteredGrid(ENV.getDouble(Env::PlanCellSize));
    vector<EnvCell*> sortedCells = m_2DHumanCenteredGrid->getSortedCells();

    Vector2d startPos;
    for (unsigned int i=0; i < sortedCells.size(); i++)
    {
        cout << i <<" = "<< sortedCells.at(i)->getCost() << "---------------------------------------" << endl;

        int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
        int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();

        Vector2d startPos;
        startPos[0] = q_robot_cur->at(firstIndexOfRobotDof + 0);
        startPos[1] = q_robot_cur->at(firstIndexOfRobotDof + 1);

        Vector2d goalPos;
        EnvCell* e = dynamic_cast<EnvCell*>(sortedCells.at(i));
        goalPos[0] = e->getCenter()[0];
        goalPos[1] = e->getCenter()[1];

        bool trajFound = FindTraj(startPos,goalPos,false);
//        return true;
        if (trajFound)
        {
//            m_2DHumanCenteredGrid->recomputeCostRobotOnly();
            startPos[0] = q_human_cur->at(firstIndexOfHumanDof + 0);
            startPos[1] = q_human_cur->at(firstIndexOfHumanDof + 1);

            goalPos[0] = m_2DCellPath.at(m_2DCellPath.size()-1)->getCenter()[0];
            goalPos[1] = m_2DCellPath.at(m_2DCellPath.size()-1)->getCenter()[1];

            FindTraj(startPos,goalPos,true);
            shared_ptr<Configuration> q_human = m_Human->getCurrentPos();

            for (unsigned int i = 0; i < m_2DHumanCellPath.size(); i++)
            {
                Eigen::Vector2d center = m_2DHumanCellPath.at(i)->getCenter();
    //            cout << i << " = " << endl << "abs Pos : "<< endl << center << endl << "coord : " << endl << dynamic_cast<EnvCell*>(m_2DHumanCellPath.at(i))->getCoord() << endl << "---------------" << endl;

                (*q_human)[firstIndexOfHumanDof + 0] = center[0];
                (*q_human)[firstIndexOfHumanDof + 1] = center[1];
                m_Human->setAndUpdate(*q_human);
                q_human->setAsNotTested();

                if (q_human->isInCollision())
                {
                    m_2DHumanCellPath.resize(i);
                    m_2DHumanPath.resize(i);
                }
            }
            m_Human->setAndUpdate(*q_human_cur);

             //compute the OTP for the configs found.
            shared_ptr<Configuration> q_human_otp = m_Human->getCurrentPos();
            shared_ptr<Configuration> q_robot_otp = _Robot->getCurrentPos();

            (*q_human_otp)[firstIndexOfHumanDof + 0] = m_2DHumanCellPath.at(m_2DHumanCellPath.size()-1)->getCenter()[0];
            (*q_human_otp)[firstIndexOfHumanDof + 1] = m_2DHumanCellPath.at(m_2DHumanCellPath.size()-1)->getCenter()[1];
            m_Human->setAndUpdate(*q_human_otp);

            (*q_robot_otp)[firstIndexOfRobotDof + 0] = m_2DCellPath.at(m_2DCellPath.size()-1)->getCenter()[0];
            (*q_robot_otp)[firstIndexOfRobotDof + 1] = m_2DCellPath.at(m_2DCellPath.size()-1)->getCenter()[1];
            _Robot->setAndUpdate(*q_robot_otp);

            if (computeUpBodyOpt())
            {
                _Robot->setGoTo(*_Robot->getCurrentPos());
                _Robot->setAndUpdate(*q_robot_cur);

                m_Human->setGoTo(*m_Human->getCurrentPos());
                m_Human->setAndUpdate(*q_human_cur);

                return true;
            }

        }
    }
    return false;

}

double OTPMotionPl::FindTraj(Vector2d startPos, Vector2d goalPos, bool isHuman)
{

//    p3d_col_activate_robot(_Robot);
    EnvCell* startCell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(startPos));
    cout << "start cell coord : x = " << startCell->getCoord()[0] << " y = " << startCell->getCoord()[1] << endl;
    EnvState* start = new EnvState(
            startCell,
            m_2DGrid);

    EnvCell* goalCell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(goalPos));
    cout << "goal cell coord : x = " << goalCell->getCoord()[0] << " y = " << goalCell->getCoord()[1] << endl;
    EnvState* goal = new EnvState(
            goalCell,
            m_2DGrid);

    if ( startCell == NULL ||  goalCell == NULL)
    {
        return -1;
    }

    if (goalCell == startCell)
    {
        cout << "Same start and goal cell" << endl;
        if (isHuman)
        {
            m_2DHumanPath.clear();
            m_2DHumanCellPath.clear();
            m_HumanPathExist = false;
        }
        else
        {
            m_2DPath.clear();
            m_2DCellPath.clear();
            m_PathExist = false;
        }
        m_noPath = false;
        return 0;
    }
    solveAStar(start,goal,isHuman);

    double dist = 0;

    if (isHuman)
    {
        for (unsigned int i = 1; i < m_2DHumanCellPath.size(); i++)
        {
            Eigen::Vector2d center = m_2DHumanCellPath.at(i)->getCenter();
            Eigen::Vector2d oldCenter = m_2DHumanCellPath.at(i-1)->getCenter();
            dist += std::sqrt(pow( center[0] - oldCenter[0], 2) + pow( center[1] - oldCenter[1], 2));
        }
    }
    else
    {
        for (unsigned int i = 1; i < m_2DCellPath.size(); i++)
        {
            Eigen::Vector2d center = m_2DCellPath.at(i)->getCenter();
            Eigen::Vector2d oldCenter = m_2DCellPath.at(i-1)->getCenter();
            dist += std::sqrt(pow( center[0] - oldCenter[0], 2) + pow( center[1] - oldCenter[1], 2));
        }
    }

    if (!isHuman)
    {
        double SumOfCost= 0.0;
        for(unsigned int i=0; i< m_2DPath.size() ; i++ )
        {
            SumOfCost +=  dynamic_cast<EnvCell*>(m_2DCellPath[i])->getCost();
        }
        cout << "SumOfCost=" << SumOfCost << endl;

//        int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
        shared_ptr<Configuration> q_cur = _Robot->getCurrentPos();
        shared_ptr<Configuration> q_tmp = _Robot->getCurrentPos();



        for (unsigned int i = 0; i < m_2DCellPath.size(); i++)
        {
            Eigen::Vector2d center = m_2DCellPath.at(i)->getCenter();
    //        cout << i << " = " << endl << "abs Pos : "<< endl << center << endl << "coord : " << endl << dynamic_cast<EnvCell*>(m_2DCellPath.at(i))->getCoord() << endl << "---------------" << endl;

//            (*q_tmp)[firstIndexOfRobotDof + 0] = center[0];
//            (*q_tmp)[firstIndexOfRobotDof + 1] = center[1];
//            _Robot->setAndUpdate(*q_tmp);

//            q_tmp->setAsNotTested();
//            if (q_tmp->isInCollision())
//            {
//                _Robot->setAndUpdate(*q_cur);
//                return false;
//            }
        }
        _Robot->setAndUpdate(*q_cur);
    }
    return dist;
}

class NaturalPoints2
{
public:

	bool operator()(pair<double,Vector3d> first, pair<double,Vector3d> second)
	{
		return ( first.first < second.first );
	}

} NaturalPointsCompObject2;

bool OTPMotionPl::computeUpBodyOpt()
{
	Vector3d transfPoint;
	if ( m_ReachableSpace == NULL )
	{
		cout << "No ReachableSpace cost space initialized" << endl;
		return false;
	}

	vector< pair<double,Vector3d > > ReachablePoints = m_ReachableSpace->getReachableWSPoint();
	vector< pair<double,Vector3d > > costs;

        // Fills each pair with
        // the cost of the sum of each criteria
    for (unsigned int i=0; i<ReachablePoints.size(); i++)
    {
        double KDist = ENV.getDouble(Env::Kdistance);
        double KVisi = ENV.getDouble(Env::Kvisibility);
        double KReac = ENV.getDouble(Env::Kreachable);

        Vector3d point   = ReachablePoints[i].second;

        double CostDist = m_DistanceSpace->getWorkspaceCost(point);
        double CostReach = ReachablePoints[i].first;
        double CostVisib = m_VisibilitySpace->getCost(point);

        Vector3d localCosts;
        localCosts[0] = CostDist;
        localCosts[1] = CostReach;
        localCosts[2] = CostVisib;

        ReachablePoints[i].first = (KDist*CostDist + KVisi*CostVisib + KReac*CostReach )/ 3;

		pair<double,Vector3d > p;
		p.first = ReachablePoints[i].first;
		p.second = localCosts;

		costs.push_back( p );
	}

	sort(ReachablePoints.begin(),ReachablePoints.end(),NaturalPointsCompObject2);
	sort(costs.begin(),costs.end(),NaturalPointsCompObject2);

	for (unsigned int i=0; i<ReachablePoints.size(); i++)
	{
		transfPoint = ReachablePoints[i].second;
		if (m_ReachableSpace->computeIsReachableAndMove(transfPoint,m_ReachableSpace->getGrid()->isReachableWithLA(transfPoint)))
		{
			current_cost.first = ReachablePoints[i].first;
			current_cost.second = costs[i].second;
			current_WSPoint = transfPoint;

			return true;
		}

	}
	return false;
}

bool OTPMotionPl::moveToNextPos()
{
	if (m_pathIndex < 0)
	{
		_Robot->setAndUpdate(*_Robot->getInitialPosition());
		m_Human->setAndUpdate(*m_Human->getInitialPosition());
		m_pathIndex = 0;
	}

	bool robotPathLimit = true;
	if (m_PathExist && m_pathIndex < m_2DCellPath.size())
	{
		int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
		shared_ptr<Configuration> q_cur = _Robot->getCurrentPos();
		(*q_cur)[firstIndexOfRobotDof + 0] = m_2DCellPath[m_pathIndex]->getCenter()[0];
		(*q_cur)[firstIndexOfRobotDof + 1] = m_2DCellPath[m_pathIndex]->getCenter()[1];
		_Robot->setAndUpdate(*q_cur);
		robotPathLimit = false;
	}

	bool humanPathLimit = true;
	if (m_HumanPathExist && m_pathIndex < m_2DHumanCellPath.size())
	{
		int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
		shared_ptr<Configuration> q_cur = m_Human->getCurrentPos();
		(*q_cur)[firstIndexOfHumanDof + 0] = m_2DHumanCellPath[m_pathIndex]->getCenter()[0];
		(*q_cur)[firstIndexOfHumanDof + 1] = m_2DHumanCellPath[m_pathIndex]->getCenter()[1];
		m_Human->setAndUpdate(*q_cur);
		humanPathLimit = false;
	}
	cout << m_pathIndex++ << endl;

	if (humanPathLimit)
	{
		m_Human->setAndUpdate(*m_Human->getGoTo());
	}


	return robotPathLimit && humanPathLimit;

}

void OTPMotionPl::initPR2GiveConf()
{
    cout << "OTPMotionPl::initPR2GiveConf()" << endl;

    shared_ptr<Configuration> q_cur = _Robot->getCurrentPos();

   configPt q;
   q = p3d_alloc_config(_Robot->getRobotStruct());


   for (unsigned int i = 0; i < 48; i++)
   {
       q[i] = (*q_cur)[i];
   }

//   q[12] = 0;
   q[13] = 0;
   q[14] = 0;
   q[15] = 0;
   q[17] = 37.00*M_PI/180;
   q[18] = -29.00*M_PI/180;
   q[19] = -133.00*M_PI/180;
   q[20] = -187.00*M_PI/180;
   q[21] = -75.00*M_PI/180;
   q[22] = 180.00*M_PI/180;
   q[23] = 0;
   q[24] = 0;
   q[25] = 0;
   q[26] = 80.00*M_PI/180;
   q[27] = 85.00*M_PI/180;
   q[28] = -90.00*M_PI/180;
   q[29] = 0;
   q[30] = 0;
   q[31] = 90.00*M_PI/180;
   q[32] = 0;
   q[33] = 0;
   q[34] = 0;
   q[35] = 0;
   q[36] = 0;



   shared_ptr<Configuration> m_q = shared_ptr<Configuration>(
                                         new Configuration(_Robot,p3d_copy_config(_Robot->getRobotStruct(),q)));
   _Robot->setAndUpdate( *m_q );
   _Robot->setInitialPosition(*m_q );
}

bool OTPMotionPl::computeProx(Vector3d WSPoint, int proximiti)
{
	int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
	shared_ptr<Configuration> q_cur = m_Human->getCurrentPos();
	double x = (*q_cur)[firstIndexOfHumanDof + 0];
	double y = (*q_cur)[firstIndexOfHumanDof + 1];
	double dist = std::sqrt(std::pow(WSPoint[0] - x, 2) + std::pow(WSPoint[1] - y, 2));

	cout << "planar distance is                  : " << dist << endl;

	if (proximiti == 0 && dist < 0.35) { return true; }
	else if (proximiti == 1 && dist < 0.7 && dist >= 0.35) { return true; }
	else if (proximiti == 2 && dist >= 0.7 ) { return true; }

	return false;

}

bool OTPMotionPl::computeHight(Eigen::Vector3d WSPoint, int hight)
{

	cout << "height is                           : " << WSPoint[2] << endl;
	if (hight == 0 && WSPoint[2] < 1.05) { return true; }
	else if (hight == 1 && WSPoint[2] < 1.35 && WSPoint[2] >= 1.05) { return true; }
	else if (hight == 2 && WSPoint[2] >= 1.35 ) { return true; }

	return false;
}

bool OTPMotionPl::computeAngle(Eigen::Vector3d WSPoint, int angle)
{
	int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
	shared_ptr<Configuration> q_cur = m_Human->getCurrentPos();
	double x = (*q_cur)[firstIndexOfHumanDof + 0];
	double y = (*q_cur)[firstIndexOfHumanDof + 1];
	double gaze = (*q_cur)[firstIndexOfHumanDof + 5];
	double refAngle = std::atan2(WSPoint[1] - y, WSPoint[0] - x) - gaze;

	cout << "the angle between gaze and point is : " << refAngle*180/M_PI << endl;

	if (angle == 0 && refAngle < M_PI/3 + M_PI/36 && refAngle > M_PI/3 - M_PI/36) { return true; }
	else if (angle == 1 && refAngle <  M_PI/6 + M_PI/36 && refAngle >  M_PI/6 - M_PI/36) { return true; }
	else if (angle == 2 && refAngle <  0      + M_PI/36 && refAngle >  0      - M_PI/36) { return true; }
	else if (angle == 3 && refAngle < -M_PI/6 + M_PI/36 && refAngle > -M_PI/6 - M_PI/36) { return true; }
	else if (angle == 4 && refAngle < -M_PI/3 + M_PI/36 && refAngle > -M_PI/3 - M_PI/36) { return true; }
	else if (angle == 5 && refAngle < -M_PI/2 + M_PI/36 && refAngle > -M_PI/2 - M_PI/36) { return true; }

	return false;
}

void OTPMotionPl::addToList()
{
	int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
	shared_ptr<Configuration> q_cur = m_Human->getCurrentPos();
	Vector3d OTPHumanCenter;
	OTPHumanCenter[0] = current_WSPoint[0] - (*q_cur)[firstIndexOfHumanDof + 0];
	OTPHumanCenter[1] = current_WSPoint[1] - (*q_cur)[firstIndexOfHumanDof + 1];
	OTPHumanCenter[2] = current_WSPoint[2] - (*q_cur)[firstIndexOfHumanDof + 2];

	m_OTPList.push_back(OTPHumanCenter);
}

void OTPMotionPl::addToList(Eigen::Vector3d WSPoint)
{
//	int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
//	shared_ptr<Configuration> q_cur = m_Human->getCurrentPos();
	Vector3d OTPHumanCenter;
	OTPHumanCenter[0] = WSPoint[0];
	OTPHumanCenter[1] = WSPoint[1];
	OTPHumanCenter[2] = WSPoint[2];

	m_OTPList.push_back(OTPHumanCenter);
}

void OTPMotionPl::showOTPList()
{
	for (unsigned int i =0 ; i < m_OTPList.size(); i++)
	{
		cout << "-----------------------" << endl << "the OTP is : \n" << m_OTPList.at(i) << endl;
		computeProx(m_OTPList.at(i),1);
		computeHight(m_OTPList.at(i),1);
		computeAngle(m_OTPList.at(i),1);

	}
}

void OTPMotionPl::setCurrentOTP(Eigen::Vector3d WSPoint)
{

	m_Human->setAndUpdate(*m_Human->getInitialPosition());
	int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
	shared_ptr<Configuration> q_cur = m_Human->getCurrentPos();


	current_WSPoint[0] = WSPoint[0] + (*q_cur)[firstIndexOfHumanDof + 0];
	current_WSPoint[1] = WSPoint[1] + (*q_cur)[firstIndexOfHumanDof + 1];
	current_WSPoint[2] = WSPoint[2] + (*q_cur)[firstIndexOfHumanDof + 2];

	shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
	shared_ptr<Configuration> q_robot_tmp = _Robot->getCurrentPos();
	int firstIndexOfDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

	(*q_robot_tmp)[firstIndexOfDof + 0] = 0;
	(*q_robot_tmp)[firstIndexOfDof + 1] = 0;
	_Robot->setAndUpdate(*q_robot_tmp);

	m_ReachableSpace->computeIsReachableAndMove(current_WSPoint,false);
	_Robot->setAndUpdate(*q_robot_cur);
}

bool OTPMotionPl::placeRobot()
{
	int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
	shared_ptr<Configuration> q_cur = m_Human->getCurrentPos();
	shared_ptr<Configuration> q_tmp = m_Human->getCurrentPos();

	Vector2d HumanPos;

	HumanPos[0] = (*q_cur)[firstIndexOfHumanDof + 0];
	HumanPos[1] = (*q_cur)[firstIndexOfHumanDof + 1];

	double gaze = (*q_cur)[firstIndexOfHumanDof + 5];
	double refAngle = std::atan2(current_WSPoint[1] - HumanPos[1], current_WSPoint[0] - HumanPos[0]) - gaze;

	shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
	int firstIndexOfDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

	(*q_robot_cur)[firstIndexOfDof + 0] = cos(refAngle)*0.7 + current_WSPoint[0];
	(*q_robot_cur)[firstIndexOfDof + 1] = sin(refAngle)*0.7 + current_WSPoint[1];

	Vector2d point;
	point[0] = current_WSPoint[0];
	point[1] = current_WSPoint[1];

	Vector2d gazeDirect = HumanPos - point;

	(*q_robot_cur)[firstIndexOfDof + 5] = atan2(gazeDirect.y(),gazeDirect.x());
	(*q_robot_cur)[firstIndexOfDof + 6] = 0.0;
	_Robot->setAndUpdate(*q_robot_cur);

	m_Human->setAndUpdate(*m_Human->getInitialPosition());

	initPR2GiveConf();

	if (!ComputePR2Gik())
	{
		(*q_robot_cur)[firstIndexOfDof + 6] = 0.31;
		_Robot->setAndUpdate(*q_robot_cur);
		if (!ComputePR2Gik())
		{
			(*q_robot_cur)[firstIndexOfDof + 6] = 0;
			_Robot->setAndUpdate(*q_robot_cur);
			return false;
		}

	}

	m_Human->setAndUpdate(*q_cur);


	return true;
}

void OTPMotionPl::drawOTPList(bool value)
{
	if (value)
	{
		OTPList = m_OTPList;
	}
	else
	{
		OTPList.clear();
	}
}

void OTPMotionPl::placeHuman()
{
	shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
	_Robot->setAndUpdate(*_Robot->getInitialPosition());

	m_ReachableSpace->computeIsReachableAndMove(current_WSPoint,false);
	_Robot->setAndUpdate(*q_robot_cur);


//	char file[256];
//	sprintf(file,"/home/magharbi/MySofts/test.ttt");
//	cout << "Saving graph to : " << file << endl;
//	p3d_writeGraph(XYZ_GRAPH, file, DEFAULTGRAPH);

}

std::vector<ConfigHR> OTPMotionPl::addConfToList()
{
	shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
	shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();

//    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
//    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

//	(*q_robot_cur)[firstIndexOfRobotDof + 0] = (*q_robot_cur)[firstIndexOfRobotDof + 0] - (*q_human_cur)[firstIndexOfHumanDof + 0];
//	(*q_robot_cur)[firstIndexOfRobotDof + 1] = (*q_robot_cur)[firstIndexOfRobotDof + 1] - (*q_human_cur)[firstIndexOfHumanDof + 1];
//	(*q_robot_cur)[firstIndexOfRobotDof + 5] = (*q_robot_cur)[firstIndexOfRobotDof + 5] - (*q_human_cur)[firstIndexOfHumanDof + 5];
//	(*q_human_cur)[firstIndexOfHumanDof + 0] = 0;
//	(*q_human_cur)[firstIndexOfHumanDof + 1] = 0;
////	q_human_cur[firstIndexOfHumanDof + 2] = 1.07;
//	(*q_human_cur)[firstIndexOfHumanDof + 5] = 0;



	ConfigHR chr;
	chr.setHumanConf(m_Human, q_human_cur->getConfigStruct());
	chr.setRobotConf(_Robot, q_robot_cur->getConfigStruct());

	m_configList.push_back(chr);

	return m_configList;
}

void OTPMotionPl::removeLastConf()
{
	m_configList.pop_back();
	ConfigHR::index--;
}

void OTPMotionPl::clearConfList()
{
	m_configList.clear();
	ConfigHR::index = 0;
}

void OTPMotionPl::saveToXml(string filename)
{
    stringstream ss;
    string str;

    //Creating the file Variable version 1.0
    xmlDocPtr doc = xmlNewDoc(xmlCharStrdup("1.0"));


    //Writing the root node
    xmlNodePtr root = xmlNewNode (NULL, xmlCharStrdup("StoredConfiguration"));

    //nb node
    str.clear(); ss << m_configList.size(); ss >> str; ss.clear();
    xmlNewProp (root, xmlCharStrdup("nb_node"), xmlCharStrdup(str.c_str()));

//    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
//    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

    for (unsigned int i = 0; i< m_configList.size(); i++)
    {
        std::ostringstream oss;
        oss << i;
        string name = "node_" + oss.str();
        xmlNodePtr cur = xmlNewChild (root, NULL, xmlCharStrdup(name.c_str()), NULL);
        xmlNodePtr curHuman = xmlNewChild (cur, NULL, xmlCharStrdup("humanConfig"), NULL);

//        shared_ptr<Configuration> hq(new Configuration(m_Human,m_configList.at(i).getHumanConf()));
//        (*hq)[firstIndexOfHumanDof + 0] = 0;
//        (*hq)[firstIndexOfHumanDof + 1] = 0;
//        (*hq)[firstIndexOfHumanDof + 5] = 0;
        writeXmlRobotConfig(curHuman,m_Human->getRobotStruct(),m_configList.at(i).getHumanConf());

        xmlNodePtr curRobot = xmlNewChild (cur, NULL, xmlCharStrdup("robotConfig"), NULL);

//        shared_ptr<Configuration> rq(new Configuration(_Robot,m_configList.at(i).getRobotConf()));
//        (*rq)[firstIndexOfRobotDof + 0] -= (*hq)[firstIndexOfHumanDof + 0];
//        (*rq)[firstIndexOfRobotDof + 1] -= (*hq)[firstIndexOfHumanDof + 1];
//        (*rq)[firstIndexOfRobotDof + 5] -= (*hq)[firstIndexOfHumanDof + 5];
        writeXmlRobotConfig(curRobot,_Robot->getRobotStruct(),m_configList.at(i).getRobotConf());
    }


    xmlDocSetRootElement(doc, root);
    //	writeRootNode(graph, root);
    //	writeSpeGraph(graph, file, root);

    //Writing the file on HD
    xmlSaveFormatFile (filename.c_str(), doc, 1);
    xmlFreeDoc(doc);

    cout << "Writing Grid to : " << filename << endl;

}


int OTPMotionPl::loadConfsFromXML(string filename, bool isStanding)
{
    if (isStanding)
    {
        m_configList = loadFromXml(filename);
        sortConfigList(m_configList.size(),isStanding);
        return m_configList.size();
    }
    else
    {
        m_sittingConfigList = loadFromXml(filename);
        sortConfigList(m_sittingConfigList.size(),isStanding);
        return m_sittingConfigList.size();
    }

}

std::vector<ConfigHR> OTPMotionPl::loadFromXml(string filename)
{
    vector<ConfigHR> vectConfs;

    int nbNode = 0;

    xmlDocPtr doc;
    xmlNodePtr cur;
    xmlNodePtr root;

    doc = xmlParseFile(filename.c_str());

    if(doc==NULL)
    {
        cout << "Document not parsed successfully (doc==NULL)" << endl;
        return vectConfs;
    }

    root = xmlDocGetRootElement(doc);

    if (root == NULL)
    {
        cout << "Document not parsed successfully" << endl;
        xmlFreeDoc(doc);
        return vectConfs;
    }

	if (xmlStrcmp(root->name, xmlCharStrdup("StoredConfiguration")))
	{
		cout << "Document of the wrong type root node not StoredConfiguration" << endl;
		xmlFreeDoc(doc);
		return vectConfs;
	}

	xmlChar* tmp;

	tmp = xmlGetProp(root, xmlCharStrdup("nb_node"));

	sscanf((char *) tmp, "%d", &nbNode );
	xmlFree(tmp);

	/***************************************/
	cur = root->xmlChildrenNode->next;
	int i =0;

    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

	vectConfs.clear();
	while (i < nbNode)
	{
		if (cur == NULL)
		{
			cout << "Document error on the number of node" << endl;
			return vectConfs;
		}
		std::string name(reinterpret_cast<const char*>(cur->name));


		if (name.find("node") != string::npos )
		{
			i++;
			xmlNodePtr ptrTmp = cur->xmlChildrenNode->next;
			if (xmlStrcmp(ptrTmp->name, xmlCharStrdup("humanConfig")))
			{
				cout << "Error: no human config" << endl;
				return vectConfs;
			}
			configPt q_hum =readXmlConfig(m_Human->getRobotStruct(),ptrTmp->xmlChildrenNode->next);


//			cout << cur->name << endl;
//			readXmlConfig()

			ptrTmp = ptrTmp->next->next;
			if (xmlStrcmp(ptrTmp->name, xmlCharStrdup("robotConfig")))
			{
				cout << "Error: no human config" << endl;
				return vectConfs;
			}
			configPt q_rob =readXmlConfig(_Robot->getRobotStruct(),ptrTmp->xmlChildrenNode->next);

			ConfigHR chr;



			q_rob[firstIndexOfRobotDof + 0] = q_rob[firstIndexOfRobotDof + 0] - q_hum[firstIndexOfHumanDof + 0];
			q_rob[firstIndexOfRobotDof + 1] = q_rob[firstIndexOfRobotDof + 1] - q_hum[firstIndexOfHumanDof + 1];
			q_rob[firstIndexOfRobotDof + 5] = q_rob[firstIndexOfRobotDof + 5] - q_hum[firstIndexOfHumanDof + 5];
			q_hum[firstIndexOfHumanDof + 0] = 0;
			q_hum[firstIndexOfHumanDof + 1] = 0;
//			q_hum[firstIndexOfHumanDof + 2] = 1.07;
			q_hum[firstIndexOfHumanDof + 5] = 0;

			chr.setHumanConf(m_Human, q_hum);
			chr.setRobotConf(_Robot, q_rob);

			vectConfs.push_back(chr);
		}



		cur = cur->next;
	}




	return vectConfs;

//	configPt readXmlConfig(p3d_rob *robot, xmlNodePtr cur)

}

pair<shared_ptr<Configuration>,shared_ptr<Configuration> > OTPMotionPl::setRobotsToConf(int id, bool isStanding)
{
	pair<shared_ptr<Configuration>,shared_ptr<Configuration> > resConf;
	vector<ConfigHR> vectConfs;
	if (isStanding)
	{
		vectConfs = m_configList;
	}
	else
	{
		vectConfs = m_sittingConfigList;
	}
	for (unsigned int i = 0; i< vectConfs.size(); i++)
	{
		if (vectConfs.at(i).getId() == id)
		{

			int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
			int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

//			shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
			shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();

			shared_ptr<Configuration> ptrQ_human(new Configuration(m_Human,vectConfs.at(i).getHumanConf()));
			(*ptrQ_human)[firstIndexOfHumanDof + 0] = (*q_human_cur)[firstIndexOfHumanDof + 0];
			(*ptrQ_human)[firstIndexOfHumanDof + 1] = (*q_human_cur)[firstIndexOfHumanDof + 1];
			(*ptrQ_human)[firstIndexOfHumanDof + 5] = (*q_human_cur)[firstIndexOfHumanDof + 5];
			m_Human->setAndUpdate(*ptrQ_human);
			resConf.first = ptrQ_human;

			shared_ptr<Configuration> ptrQ_robot(new Configuration(_Robot,vectConfs.at(i).getRobotConf()));
			(*ptrQ_robot)[firstIndexOfRobotDof + 5] += (*q_human_cur)[firstIndexOfHumanDof + 5];
			double dist = sqrt(pow( (*ptrQ_robot)[firstIndexOfRobotDof + 0], 2) + pow( (*ptrQ_robot)[firstIndexOfRobotDof + 1], 2));
			(*ptrQ_robot)[firstIndexOfRobotDof + 0] = -cos((*ptrQ_robot)[firstIndexOfRobotDof + 5])*dist + (*ptrQ_human)[firstIndexOfHumanDof + 0];
			(*ptrQ_robot)[firstIndexOfRobotDof + 1] = -sin((*ptrQ_robot)[firstIndexOfRobotDof + 5])*dist + (*ptrQ_human)[firstIndexOfHumanDof + 1];

			_Robot->setAndUpdate(*ptrQ_robot);
			resConf.second = ptrQ_robot;


			return resConf;
		}
	}
	return resConf;
}

pair<shared_ptr<Configuration>,shared_ptr<Configuration> > OTPMotionPl::setRobotsToConf(int id, bool isStanding ,double x, double y, double Rz)
{
	int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
	shared_ptr<Configuration> ptrQ_human = m_Human->getCurrentPos();
	(*ptrQ_human)[firstIndexOfHumanDof + 0] = x;
	(*ptrQ_human)[firstIndexOfHumanDof + 1] = y;
	(*ptrQ_human)[firstIndexOfHumanDof + 5] = Rz;
	m_Human->setAndUpdate(*ptrQ_human);

	return setRobotsToConf(id, isStanding);
}

double OTPMotionPl::computeConfigCost(configPt q_rob_initial,
									  configPt q_rob_final,
									  configPt q_hum_initial,
									  configPt q_hum_final)
{
	shared_ptr<Configuration> ptrQ_human(new Configuration(m_Human,q_hum_final));
	m_Human->setAndUpdate(*ptrQ_human);

	// compute the confort of human posture
	double confortCost = m_ReachableSpace->getConfigCost();

	m_Human->setAndUpdate(*m_Human->getInitialPosition());

	// compute the trajectory between initial and final configuration

	int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;


    Vector2d startPos;
    startPos[0] = q_rob_initial[firstIndexOfRobotDof + 0];
    startPos[1] = q_rob_initial[firstIndexOfRobotDof + 1];

    Vector2d goalPos;
    goalPos[0] = q_rob_final[firstIndexOfRobotDof + 0];
    goalPos[1] = q_rob_final[firstIndexOfRobotDof + 1];

    /*bool robotTrajFound = */FindTraj(startPos,goalPos,false);

    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();

    startPos[0] = q_hum_initial[firstIndexOfHumanDof + 0];
    startPos[1] = q_hum_initial[firstIndexOfHumanDof + 1];

    goalPos[0] = q_hum_final[firstIndexOfHumanDof + 0];
    goalPos[1] = q_hum_final[firstIndexOfHumanDof + 1];

    /*bool humanTrajFound = */FindTraj(startPos,goalPos,true);

    return confortCost;
}

double OTPMotionPl::testComputeConfigCost()
{
    if (m_configList.size() > 0)
    {
        return computeConfigCost(_Robot->getInitialPosition()->getConfigStruct(),
                                 m_configList.at(7).getRobotConf(),
                                 m_Human->getInitialPosition()->getConfigStruct(),
                                 m_configList.at(7).getHumanConf());
    }
    return 0;
}

/*!
 * configuration cost sorter
 */
class ConfigurationCost
{
public:

    bool operator()(ConfigHR first, ConfigHR second)
    {
        return ( first.getCost() < second.getCost() );
    }

} ConfigurationCostCompObject;


void OTPMotionPl::sortConfigList(double nbNode, bool isStanding)
{
    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();

    vector<ConfigHR> vectConfs;
    if (isStanding)
    {
        vectConfs = m_configList;
    }
    else
    {
        vectConfs = m_sittingConfigList;
    }

    for (int i = 0; i < nbNode; i++)
    {
        shared_ptr<Configuration> ptrQ_robot(new Configuration(_Robot,vectConfs.at(i).getRobotConf()));
        _Robot->setAndUpdate(*ptrQ_robot);

		shared_ptr<Configuration> ptrQ_human(new Configuration(m_Human,vectConfs.at(i).getHumanConf()));
		m_Human->setAndUpdate(*ptrQ_human);

		vectConfs.at(i).setCost(m_ReachableSpace->getConfigCost());
	}
	_Robot->setAndUpdate(*q_robot_cur);
	m_Human->setAndUpdate(*q_human_cur);

	sort(vectConfs.begin(),vectConfs.end(),ConfigurationCostCompObject);

    for (int i = 0; i < nbNode; i++)
    {
        vectConfs.at(i).setId(i);
    }

    if (isStanding)
    {
        m_configList = vectConfs;
    }
    else
    {
       m_sittingConfigList = vectConfs;
    }


}

class OutputConfSort
{
public:

	bool operator()(OutputConf first, OutputConf second)
	{
		return ( first.cost < second.cost );
	}

} OutputConfSortObj;

string str;

Vector3d OTPMotionPl::getRandomPoints(double id)
{
    Vector3d vect;

    if (PlanEnv->getBool(PlanParam::env_normalRand))
    {
        shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
        int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();

        double x = (*q_human_cur)[firstIndexOfHumanDof + 0];
        double y = (*q_human_cur)[firstIndexOfHumanDof + 1];
//        double Rz = (*q_human_cur)[firstIndexOfHumanDof + 5];

        double randomXMinLimit = PlanEnv->getDouble(PlanParam::env_randomXMinLimit);//-3.0
        double randomXMaxLimit = PlanEnv->getDouble(PlanParam::env_randomXMaxLimit);//3.0
        double randomYMinLimit = PlanEnv->getDouble(PlanParam::env_randomYMinLimit);//-3.0
        double randomYMaxLimit = PlanEnv->getDouble(PlanParam::env_randomYMaxLimit);//3.0
        int nbRandomRotOnly = PlanEnv->getInt(PlanParam::env_nbRandomRotOnly);//10

        vect[0] = p3d_random(x + randomXMinLimit, x + randomXMaxLimit);
        vect[1] = p3d_random(y + randomYMinLimit, y + randomYMaxLimit);
        if (id < nbRandomRotOnly)
        {
            vect[0] = x;
            vect[1] = y;
        }
        vect[2] = p3d_random(-M_PI,M_PI);
    }
    else if (PlanEnv->getBool(PlanParam::env_fusedGridRand))
    {
        std::vector<std::pair<double,EnvCell*> > sortedCell = m_2DGrid->getSortedGrid();
        double rand = p3d_random(0,1);
        rand = pow(rand,PlanEnv->getInt(PlanParam::env_pow)); // giving a higher probability of getting small numbers
        int cellNb = floor(rand*(sortedCell.size()-1));
        Vector2d center = sortedCell.at(cellNb).second->getCenter();
        Vector2d cellSize = sortedCell.at(cellNb).second->getCellSize();
        vect[0] = p3d_random(center[0] - cellSize[0]/2, center[0] + cellSize[0]/2);
        vect[1] = p3d_random(center[1] - cellSize[1]/2, center[1] + cellSize[1]/2);
        vect[2] = p3d_random(-M_PI,M_PI);

    }
    else
    {
        cout << "ERROR in random" << endl ;
        vect[0] = 0;
        vect[1] = 0;
        vect[2] = 0;
    }

    return vect;
}

void OTPMotionPl::newComputeOTP()
{
    clock_t start = clock();
//    m_2DGrid->init(computeHumanRobotDist());
    m_2DGrid->initGrid();
    m_2DGrid->setCellsToblankCost();

    clock_t gridInit = clock();

    bool isStanding = PlanEnv->getBool(PlanParam::env_isStanding);
    double objectNecessity = PlanEnv->getDouble(PlanParam::env_objectNessecity);

    int maxIter = PlanEnv->getInt(PlanParam::env_maxIter); //300
//    int totMaxIter = PlanEnv->getInt(PlanParam::env_totMaxIter); //1000
//    int nbRandomRotOnly = PlanEnv->getInt(PlanParam::env_nbRandomRotOnly);//10
//    double randomXMinLimit = PlanEnv->getDouble(PlanParam::env_randomXMinLimit);//-3.0
//    double randomXMaxLimit = PlanEnv->getDouble(PlanParam::env_randomXMaxLimit);//3.0
//    double randomYMinLimit = PlanEnv->getDouble(PlanParam::env_randomYMinLimit);//-3.0
//    double randomYMaxLimit = PlanEnv->getDouble(PlanParam::env_randomYMaxLimit);//3.0

    clearCostsfile();
    if (m_configList.empty() || m_sittingConfigList.empty())
    {
        cout << "No configuration lists" << endl;
        return;
    }
    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();
    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();


    double x = (*q_human_cur)[firstIndexOfHumanDof + 0];
    double y = (*q_human_cur)[firstIndexOfHumanDof + 1];
    double Rz = (*q_human_cur)[firstIndexOfHumanDof + 5];

    cout << "---------------------------------------------------" << endl;
    cout << "start OTP search" << endl <<"---------------------------------------------------" << endl;
    OutputConf bestConf;


    confList.clear();

    clock_t varInit = clock();
    if (isStanding)
    {
        bestConf = lookForBestLocalConf(x,y,Rz,objectNecessity);
        confList.push_back(bestConf);
        saveCostsTofile(bestConf.cost,bestConf.cost);
        isInitSiting = false;
    }
    else
    {
        bestConf = findBestPosForHumanSitConf(objectNecessity);
        isInitSiting = true;
    }


//    cout << "init conf cost :"<< bestConf.cost << endl;
    int i = 0;
    int id = confList.size();
    bestConf.id = id;
//    cout << "current id = "<< id << endl;

    m_Human->setAndUpdate(*q_human_cur);
    _Robot->setAndUpdate(*q_robot_cur);

    m_2DGrid->setAsNotSorted();

    clock_t firstConfs = clock();
    while (true)
    {
//        cout << "---------------------------------------------------" << endl;
//        cout << "new section, init pos : x = " << x << " y = " << y << " Rz = " << Rz << endl;
        Vector3d vect = getRandomPoints(id);
        double randomX = vect[0];
        double randomY = vect[1];
        double randomRz = vect[2];
//        cout << "random : x = " << randomX << " y = " << randomY << " Rz = " << randomRz << endl;

        OutputConf tmpConf = lookForBestLocalConf(randomX,randomY,randomRz,objectNecessity);
        tmpConf.id = id;
//        cout << "Cost : "<< tmpConf.cost << endl;
        if (tmpConf.cost < numeric_limits<double>::max( ))
        {
            confList.push_back(tmpConf);
//            saveCostsTofile(bestConf.cost);

        }
        saveCostsTofile(bestConf.cost,tmpConf.cost);

        if (tmpConf.cost < bestConf.cost)
        {
            bestConf = tmpConf;
            i = 0;

        }
        else if (tmpConf.cost < numeric_limits<double>::max( )){ i++; }
        id++;
//        cout << "current id = "<< id << endl;
//        cout << "current iteration worst than the best one : " << i << endl;
        if (i > maxIter || id > PlanEnv->getInt(PlanParam::env_totMaxIter)) { break; }

        int time = PlanEnv->getInt(PlanParam::env_timeShow)*1000;
        if (PlanEnv->getBool(PlanParam::env_drawOnlyBest) && (time > 0))
        {
            if (bestConf.cost < numeric_limits<double>::max( ))\
            {
                double xf = (*bestConf.humanConf)[firstIndexOfHumanDof + 0];
                double yf = (*bestConf.humanConf)[firstIndexOfHumanDof + 1];
                double Rzf = (*bestConf.humanConf)[firstIndexOfHumanDof + 5];
                setRobotsToConf(bestConf.configNumberInList,bestConf.isStandingInThisConf,xf,yf,Rzf);
                g3d_draw_allwin_active();
                usleep(time);
            }
        }


        m_Human->setAndUpdate(*q_human_cur);
        _Robot->setAndUpdate(*q_robot_cur);

    }
    clock_t endLoop = clock();
    if (bestConf.cost >= numeric_limits<double>::max( ))
    {
        cout << "No transfer configuration for this initials positions" << endl;
        cout << "Total time = " << ((double)endLoop - start) / CLOCKS_PER_SEC << " s"<< endl;

        _Robot->setAndUpdate(*q_robot_cur);
        m_Human->setAndUpdate(*q_human_cur);
        return;
    }


    sort(confList.begin(),confList.end(),OutputConfSortObj);

    cout << "--------------------------------------" << endl << "End of OTP search" << endl << "--------------------------------------" << endl;
    cout << "Number of tested configuration = " << id << endl;
    cout << "Number of solutions found = " << confList.size() << endl;
    cout << "Success rate = " << (double)confList.size() / (double)id << endl;
//    bestConf.humanConf->print();
//    bestConf.robotConf->print();
//    cout << "the choosed conf id is : " << bestConf.id << endl;
    cout << "the choosed configuration cost is : " << bestConf.cost << endl;
    m_2DHumanPath.clear();
    m_2DPath.clear();
    m_2DHumanPath = bestConf.humanTraj;
    m_HumanPathExist = bestConf.humanTrajExist;
    m_2DPath = bestConf.robotTraj;
    m_PathExist = bestConf.robotTrajExist;
    double xf = (*bestConf.humanConf)[firstIndexOfHumanDof + 0];
    double yf = (*bestConf.humanConf)[firstIndexOfHumanDof + 1];
    double Rzf = (*bestConf.humanConf)[firstIndexOfHumanDof + 5];
    setRobotsToConf(bestConf.configNumberInList,bestConf.isStandingInThisConf,xf,yf,Rzf);
//    createTrajectoryFromOutputConf(bestConf);
    clock_t end = clock();
    cout << "==========" << endl <<"time elapsed to : " << endl;
    cout << "init grid = " << ((double)gridInit - start) / CLOCKS_PER_SEC << " s"<< endl;
    cout << "init variable = " << ((double)varInit - gridInit) / CLOCKS_PER_SEC << " s"<< endl;
    cout << "look for the first conf = " << ((double)firstConfs - varInit) / CLOCKS_PER_SEC << " s"<< endl;
    cout << "pass the loop = " << ((double)endLoop - firstConfs) / CLOCKS_PER_SEC << " s"<< endl;
    cout << "end the function = " << ((double)end - endLoop) / CLOCKS_PER_SEC << " s"<< endl;
    cout << "----------" << endl << "Total time = " << ((double)end - start) / CLOCKS_PER_SEC << " s"<< endl;
    cout << "==========" << endl;
}

OutputConf OTPMotionPl::lookForBestLocalConf(double x, double y, double Rz, double objectNecessity)
{
    OutputConf bestLocalConf;
    bestLocalConf.isStandingInThisConf = true;

    if (x < XYZ_ENV->box.x1 ||  x > XYZ_ENV->box.x2 || y < XYZ_ENV->box.y1 || y > XYZ_ENV->box.y2)
    {
//        cout << "human out of bound : x = " << x << " y = " << y << " Rz = " << Rz << endl;
        bestLocalConf.clearAll();
        return bestLocalConf;
    }

    double robotSpeed = PlanEnv->getDouble(PlanParam::env_robotSpeed);//1;
    double humanSpeed = PlanEnv->getDouble(PlanParam::env_humanSpeed);//1;
    double timeStamp = PlanEnv->getDouble(PlanParam::env_timeStamp);//0.1;
    double psi = PlanEnv->getDouble(PlanParam::env_psi);//0.99;
    double delta = PlanEnv->getDouble(PlanParam::env_delta);//0.01;

    double ksi = PlanEnv->getDouble(PlanParam::env_ksi);//0.5;
    double rho = PlanEnv->getDouble(PlanParam::env_rho);//0.5;

    double sittingOffset = PlanEnv->getDouble(PlanParam::env_sittingOffset);//0.2;

    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();

    double configCost = -1;
    vector<ConfigHR> vectConfs = m_configList;

    if (!vectConfs.empty())
    {
        for(unsigned int i = 0; i < vectConfs.size(); i++)
        {
            pair<shared_ptr<Configuration>,shared_ptr<Configuration> > conf = setRobotsToConf(i,true,x,y,Rz);

            conf.first->setAsNotTested();
            conf.second->setAsNotTested();
//            if (!conf.first->isInCollision() && !conf.second->isInCollision())
            string humanStr = testCol(true,false)?"is in collision":"is NOT in collision";
            string robotStr = testCol(false,false)?"is in collision":"is NOT in collision";
//            cout << "test colision : human " << humanStr << " and robot " << robotStr << endl;
            if (!testCol(true,false) && !testCol(false,true))
            {
                bestLocalConf.humanConf = conf.first;
                bestLocalConf.robotConf = conf.second;
                bestLocalConf.configNumberInList = i;
                configCost = vectConfs.at(i).getCost();
//                cout << "the choosed configuration in the configuration list is : " << i << endl;
                break;
            }
        }

    }

    if (configCost < 0)
    {
//        cout << "No conf found for that position : x = " << x << " y = " << y << " Rz = " << Rz << endl;
//        bestLocalConf.cost = numeric_limits<double>::max( );
        bestLocalConf.clearAll();
        return bestLocalConf;
    }

//    cout << "Config cost = " << configCost << endl;

    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();

//    Vector2d startPos;
//    startPos[0] = (*q_human_cur)[firstIndexOfHumanDof + 0];
//    startPos[1] = (*q_human_cur)[firstIndexOfHumanDof + 1];

//    EnvCell* scell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(startPos));
//    cout << "human start coord : x " << scell->getCoord()[0] << " y " << scell->getCoord()[1] << endl;

    Vector2d goalPos;
    goalPos[0] = (*bestLocalConf.humanConf)[firstIndexOfHumanDof + 0];
    goalPos[1] = (*bestLocalConf.humanConf)[firstIndexOfHumanDof + 1];

    EnvCell* cell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(goalPos));
    if (!cell)
    {
  //      cout << "No cell for this human position : x = " << x << " y = " << y << " Rz = " << Rz << endl;
        bestLocalConf.clearAll();
        return bestLocalConf;
    }
    if (!cell->isHumAccessible())
    {
//        cout << "No human traj found for that position : x = " << x << " y = " << y << " Rz = " << Rz << endl;
//        cout << "coords : x = " << cell->getCoord()[0] << " y = " << cell->getCoord()[1] << endl;
        bestLocalConf.clearAll();
        return bestLocalConf;
    }

//    cout << "human goal coord : x " << cell->getCoord()[0] << " y " << cell->getCoord()[1] << endl;
//    for (unsigned int i =0; i < cell->getHumanTraj().size(); i++)
//    {
//        cout << "traj at " << i << " coord is : x "<< cell->getHumanTraj().at(i)->getCoord()[0] << " y " << cell->getHumanTraj().at(i)->getCoord()[1] << endl;
//    }

    double hDist = cell->getHumanDist() / m_2DGrid->getHumanMaxDist();

    if (isInitSiting)
    {
        hDist += sittingOffset;
    }

    bestLocalConf.humanTraj = cell->getHumanVectorTraj();
    bestLocalConf.humanTrajExist = false;
    if (bestLocalConf.humanTraj.size()> 0)
    {
        bestLocalConf.humanTrajExist = true;
    }

    double hRot = (*bestLocalConf.humanConf)[firstIndexOfHumanDof + 5] - (*q_human_cur)[firstIndexOfHumanDof + 5];

    double mvCost = (psi*hDist + delta*fabs(hRot) )/(psi+delta);

//    cout << "moving cost = " << mvCost << endl;

    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

//    startPos[0] = (*q_robot_cur)[firstIndexOfRobotDof + 0];
//    startPos[1] = (*q_robot_cur)[firstIndexOfRobotDof + 1];

//    scell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(startPos));
//    cout << "robot start coord : x " << scell->getCoord()[0] << " y " << scell->getCoord()[1] << endl;


    goalPos[0] = (*bestLocalConf.robotConf)[firstIndexOfRobotDof + 0];
    goalPos[1] = (*bestLocalConf.robotConf)[firstIndexOfRobotDof + 1];

    if (goalPos[0] < XYZ_ENV->box.x1 ||  goalPos[0] > XYZ_ENV->box.x2 || goalPos[1] < XYZ_ENV->box.y1 || goalPos[1] > XYZ_ENV->box.y2)
    {
//        cout << "Robot out of bound : x = " << goalPos[0] << " y = " << goalPos[1] << " Rz = " << endl;
        bestLocalConf.clearAll();
        return bestLocalConf;
    }

    EnvCell* rCell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(goalPos));
    if (!rCell)
    {
//        cout << "No cell for this robot position : x = " << goalPos[0] << " y = " << goalPos[1] << endl;
        bestLocalConf.clearAll();
        return bestLocalConf;
    }
    if (!rCell->isRobAccessible())
    {
//        cout << "No robot traj found for that position : x = " << x << " y = " << y << " Rz = " << Rz << endl;
//        cout << "coords : x = " << cell->getCoord()[0] << " y = " << cell->getCoord()[1] << endl;
        bestLocalConf.clearAll();
//        bestLocalConf.cost = numeric_limits<double>::max( );
        return bestLocalConf;
    }

//    cout << "robot goal coord : x " << cell->getCoord()[0] << " y " << cell->getCoord()[1] << endl;
//    for (unsigned int i =0; i < cell->getRobotTraj().size(); i++)
//    {
//        cout << "traj at " << i << " coord is : x "<< cell->getRobotTraj().at(i)->getCoord()[0] << " y " << cell->getRobotTraj().at(i)->getCoord()[1] << endl;
//    }

    double rDist = rCell->getRobotDist() / m_2DGrid->getRobotMaxDist();

    bestLocalConf.robotTraj = rCell->getRobotVectorTraj();
    bestLocalConf.robotTrajExist = false;
    if (bestLocalConf.robotTraj.size()> 0)
    {
        bestLocalConf.robotTrajExist = true;
    }

    double hTime = hDist/humanSpeed;

    double rTime = rDist/robotSpeed;

    double tempCost = hTime * timeStamp;
    if (hTime < rTime)
    {
        tempCost = rTime * timeStamp;
    }

//    cout << "temporal cost = " << tempCost << endl;

    bestLocalConf.cost = (ksi * mvCost + rho * configCost) * (1 - objectNecessity)/(rho + ksi) + tempCost * objectNecessity;

    int time = PlanEnv->getInt(PlanParam::env_timeShow)*1000;
    if (ENV.getBool(Env::drawGraph) && !PlanEnv->getBool(PlanParam::env_drawOnlyBest) && (time > 0))
    {
        g3d_draw_allwin_active();
        usleep(time);
    }

    cell->addPoint(Rz);

    _Robot->setAndUpdate(*q_robot_cur);
    m_Human->setAndUpdate(*q_human_cur);

    return bestLocalConf;
}

OutputConf OTPMotionPl::findBestPosForHumanSitConf(double objectNecessity)
{

    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();
    shared_ptr<Configuration> q_human_cur = m_Human->getCurrentPos();

    OutputConf bestLocalConf;
    bestLocalConf.cost = numeric_limits<double>::max( );
    OutputConf bestConf;
    bestConf.cost = numeric_limits<double>::max( );
    double ksi = PlanEnv->getDouble(PlanParam::env_ksi);//0.5;
    double rho = PlanEnv->getDouble(PlanParam::env_rho);//0.5;

    double robotSpeed = PlanEnv->getDouble(PlanParam::env_robotSpeed);//1;
    double timeStamp = PlanEnv->getDouble(PlanParam::env_timeStamp);//0.1;

    if (m_sittingConfigList.empty())
    {
        return bestLocalConf;
    }

//    cout << "sitting test" << endl;
    double configCost = 0;
    int id = 0;
    for(unsigned int i = 0; i < m_sittingConfigList.size(); i++)
    {
//        cout << "---------------------------" << endl;
        pair<shared_ptr<Configuration>,shared_ptr<Configuration> > conf = setRobotsToConf(i,false);

        conf.first->setAsNotTested();
        conf.second->setAsNotTested();


        string humanStr = testCol(true,false)?"is in collision":"is NOT in collision";
        string robotStr = testCol(false,false)?"is in collision":"is NOT in collision";
//        cout << "test colision : human " << humanStr << " and robot " << robotStr << endl;

        if (!testCol(true,false) && !testCol(false,true))
        {
            int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
            Vector2d goalPos;
            goalPos[0] = (*conf.second)[firstIndexOfRobotDof + 0];
            goalPos[1] = (*conf.second)[firstIndexOfRobotDof + 1];

            if (goalPos[0] < XYZ_ENV->box.x1 ||  goalPos[0] > XYZ_ENV->box.x2 || goalPos[1] < XYZ_ENV->box.y1 || goalPos[1] > XYZ_ENV->box.y2)
            {
//                cout << "Robot out of bound : x = " << goalPos[0] << " y = " << goalPos[1] << " Rz = " << endl;
                continue;
            }

            EnvCell* cell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(goalPos));

            if (!cell)
            {
//                cout << "No cell for this robot position : x = " << goalPos[0] << " y = " << goalPos[1] << endl;
                continue;
            }
            if (cell->isRobAccessible())
            {

                OutputConf localConf;
                localConf.humanConf = conf.first;
                localConf.robotConf = conf.second;
                localConf.configNumberInList = i;
                configCost = m_sittingConfigList.at(i).getCost();
                double tempCost = (cell->getRobotDist() / m_2DGrid->getRobotMaxDist())/robotSpeed * timeStamp;
                double mvCost = 0 ;
                localConf.cost = (ksi * mvCost + rho * configCost) * (1 - objectNecessity)/(rho + ksi) + tempCost * objectNecessity;
                localConf.robotTraj = cell->getRobotVectorTraj();
                localConf.humanTrajExist = false;
                localConf.robotTrajExist = false;
                if (localConf.robotTraj.size()> 0)
                {
                    localConf.robotTrajExist = true;
                }

                localConf.isStandingInThisConf = false;
//                cout << "global cost = " << localConf.cost<< endl;
                localConf.id = id++;
                if (localConf.cost < bestConf.cost)
                {
                    bestConf = localConf;
                }
                confList.push_back(localConf);
                saveCostsTofile(bestConf.cost,localConf.cost);

                int time = PlanEnv->getInt(PlanParam::env_timeShow)*1000;
                if (ENV.getBool(Env::drawGraph) && !PlanEnv->getBool(PlanParam::env_drawOnlyBest) && (time > 0))
                {
                    g3d_draw_allwin_active();
                    usleep(time);
                }

//                cout << "the choosed configuration in the sitting configuration list is : " << i << endl;
            }
        }
    }

    _Robot->setAndUpdate(*q_robot_cur);
    m_Human->setAndUpdate(*q_human_cur);

    Robot* humCyl;
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);
        if(name.find("HUMCYLINDER") != string::npos )
        {
            humCyl = new Robot(XYZ_ENV->robot[i]);
            break;
        }

    }

    if (!humCyl)
    {
        cout << "No human cylinder found" << endl;
        return bestConf;
    }

    int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
    shared_ptr<Configuration> q_human = m_Human->getCurrentPos();
    double x = (*q_human)[firstIndexOfHumanDof + 0];
    double y = (*q_human)[firstIndexOfHumanDof + 1];
    double Rz = (*q_human)[firstIndexOfHumanDof + 5];

    (*q_human)[firstIndexOfHumanDof + 0] = -3;
    (*q_human)[firstIndexOfHumanDof + 1] = 0;
    m_Human->setAndUpdate(*q_human);

    shared_ptr<Configuration> q_humCyl_cur = humCyl->getCurrentPos();
    shared_ptr<Configuration> q_humCyl = humCyl->getCurrentPos();

    double standingDist = 0;
    double maxStandingDist = 0.6;
    while (humCyl->isInCollisionWithOthersAndEnv() && standingDist < maxStandingDist)
    {
        (*q_humCyl)[6] = x + standingDist*cos(Rz);
        (*q_humCyl)[7] = y + standingDist*sin(Rz);
        humCyl->setAndUpdate(*q_humCyl);
        standingDist += 0.05;
    }

    if (!humCyl->isInCollisionWithOthersAndEnv())
    {
        getReachability()->setRobotToConfortPosture();
        q_human = m_Human->getCurrentPos();
        (*q_human)[firstIndexOfHumanDof + 0] = (*q_humCyl)[6];
        (*q_human)[firstIndexOfHumanDof + 1] = (*q_humCyl)[7];
        (*q_human)[firstIndexOfHumanDof + 2] = 1.07;
        m_Human->setAndUpdate(*q_human);
    }
    else
    {
        m_Human->setAndUpdate(*q_human_cur);
        cout << "Human can not stand up : too much obstacle in front of him" << endl;
    }


    humCyl->setAndUpdate(*q_humCyl_cur);

    m_2DGrid->initGrid();


    return bestConf;

}


bool  OTPMotionPl::testCol(bool isHuman, bool useConf)
{
    if (isHuman)
    {
        if (useConf)
        {
            shared_ptr<Configuration> q(m_Human->getCurrentPos());
            q->setAsNotTested();
            return q->isInCollision();
        }
        else
        {
            return m_Human->isInCollisionWithOthersAndEnv();
        }
    }
    else
    {
        if (useConf)
        {
            shared_ptr<Configuration> q(_Robot->getCurrentPos());
            q->setAsNotTested();
            return q->isInCollision();
        }
        else
        {
            return _Robot->isInCollision();
        }
    }
}

void OTPMotionPl::saveInitConf()
{
    savedConf.humanConf = m_Human->getCurrentPos();
    savedConf.robotConf = _Robot->getCurrentPos();

}

void OTPMotionPl::loadInitConf()
{
    if (savedConf.humanConf && savedConf.robotConf)
    {
        m_Human->setAndUpdate(*savedConf.humanConf);
        _Robot->setAndUpdate(*savedConf.robotConf);
    }
}

double OTPMotionPl::showConf(unsigned int i)
{

    if (confList.size() > i)
    {
        int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();
        OutputConf conf = confList.at(i);
        m_2DHumanPath = conf.humanTraj;
        m_HumanPathExist = conf.humanTrajExist;
        m_2DPath = conf.robotTraj;
        m_PathExist = conf.robotTrajExist;
        double xf = (*conf.humanConf)[firstIndexOfHumanDof + 0];
        double yf = (*conf.humanConf)[firstIndexOfHumanDof + 1];
        double Rzf = (*conf.humanConf)[firstIndexOfHumanDof + 5];
        setRobotsToConf(conf.configNumberInList,conf.isStandingInThisConf,xf,yf,Rzf);
        return conf.cost;
    }
    return -1;
}


OutputConf OTPMotionPl::showBestConf()
{
    int id = 0;
    for (unsigned int i = 1; i < confList.size(); i++)
    {
        if (confList.at(i).cost < confList.at(id).cost)
        {
            id = i;
        }
    }
    showConf(id);
    return confList.at(id);
}

void OTPMotionPl::initGrid()
{
    m_2DGrid->init(computeHumanRobotDist());
    m_2DGrid->initGrid();
}

double OTPMotionPl::getDistFromCell(int x, int y, bool isHuman)
{
    EnvCell* cell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(x,y));
    if (isHuman)
    {
        if (cell->isHumanDistComputed())    { return cell->getHumanDist();}
        else                                { return -1.0; }
    }
    else
    {
        if (cell->isRobotDistComputed())    { return cell->getRobotDist();}
        else                                { return -1.0; }
    }
}

void OTPMotionPl::saveCostsTofile(double cost, double randomCost)
{
    ofstream myfile;
    string fileName = "/statFiles/OtpComputing/configurationsCosts.lst";
    string home = getenv("HOME_MOVE3D") + fileName;
    myfile.open (home.c_str(),ios::app);
    myfile << cost << "\t" << randomCost << endl;
    myfile.close();
}

void OTPMotionPl::saveCostsTofile(string cost)
{
    ofstream myfile;
    string fileName = "/statFiles/OtpComputing/configurationsCosts.lst";
    string home = getenv("HOME_MOVE3D") + fileName;
    myfile.open (home.c_str(),ios::app);
    myfile << cost << endl;
    myfile.close();
}

void OTPMotionPl::clearCostsfile()
{
    ofstream myfile;
    string fileName = "/statFiles/OtpComputing/configurationsCosts.lst";
    string home = getenv("HOME_MOVE3D") + fileName;
    myfile.open (home.c_str());
    myfile << "" << endl;
    myfile.close();
}

int OTPMotionPl::getConfListSize()
{
    if (PlanEnv->getBool(PlanParam::env_isStanding))
    {
        return m_configList.size();
    }
    else
    {
        return m_sittingConfigList.size();
    }
}

configPt OTPMotionPl::getRobotConfigAt(int i)
{
    if (PlanEnv->getBool(PlanParam::env_isStanding))
    {
        return m_configList.at(i).getRobotConf();
    }
    else
    {
        return m_sittingConfigList.at(i).getRobotConf();
    }
}

std::vector<ConfigHR> OTPMotionPl::getConfList()
{
    if (PlanEnv->getBool(PlanParam::env_isStanding))
    {
        return m_configList;
    }
    else
    {
        return m_sittingConfigList;
    }
}

double angle_limit_PI(double angle){

  while (angle < -M_PI){
    angle += 2*M_PI;
  }
  while (angle > M_PI){
    angle -= 2*M_PI;
  }
  return angle;
}


API::Trajectory OTPMotionPl::createTrajectoryFromOutputConf(OutputConf conf)
{
    vector<shared_ptr<Configuration> > vectorConf;
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Traj2D = conf.robotTraj;
//    initPR2GiveConf();
    shared_ptr<Configuration> q_cur(_Robot->getCurrentPos());
    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

    int nbConf = Traj2D.size();
    for(int i =0; i < nbConf - 1; i++)
    {
        shared_ptr<Configuration> q_tmp(_Robot->getCurrentPos());
        double x = Traj2D.at (i)[0];
        double y = Traj2D.at(i)[1];
        (*q_tmp)[firstIndexOfRobotDof + 0] = x;
        (*q_tmp)[firstIndexOfRobotDof + 1] = y;
//        (*q_tmp)[firstIndexOfRobotDof + 5] = atan2(Traj2D.at(i+1)[0] - Traj2D.at(i)[0], Traj2D.at(i+1)[1] - Traj2D.at(i)[1]);

        (*q_tmp)[firstIndexOfRobotDof + 5] = atan2(
                Traj2D.at(i+1)[1] - Traj2D.at(i)[1],
                Traj2D.at(i+1)[0] - Traj2D.at(i)[0]);

        vectorConf.push_back(q_tmp);
//        q_tmp->print();

    }

    vectorConf.push_back(conf.robotConf);

    p3d_multiLocalPath_disable_all_groupToPlan( _Robot->getRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( _Robot->getRobotStruct(), m_ManipPl->getBaseMLP(), 1, false);

    //////////////////
    API::Trajectory base_traj(vectorConf);
    base_traj.replaceP3dTraj();

    cout << base_traj.getRangeMax() << endl;
    cout << base_traj.getNbOfPaths() << endl;

    //////////////////
    API::Trajectory arm_traj;

//    shared_ptr<Configuration> qInit( base_traj.configAtParam( base_traj.getRangeMax()) );
//    configPt qConfInit =;
   (*conf.robotConf)[11] = angle_limit_PI( (*conf.robotConf)[11] );
   (*conf.robotConf)[22] = angle_limit_PI( (*conf.robotConf)[22] );

    (*q_cur)[6]  = (*conf.robotConf)[6];
    (*q_cur)[7]  = (*conf.robotConf)[7];
    (*q_cur)[11] = (*conf.robotConf)[11];

    shared_ptr<Configuration> qGoal( conf.robotConf );

//    q_cur->print();
//    qGoal->print();

//    if( this->computeArmMotion(   q_cur->getConfigStruct(),
//                                  qGoal->getConfigStruct(),
//                                  arm_traj ) )
//    {
//        cout << "Concat traj ..." << endl;
//       base_traj.concat( arm_traj );
//    }

    base_traj.replaceP3dTraj();

    return base_traj;
}

bool OTPMotionPl::computeArmMotion(double* qInit, double* qGoal, API::Trajectory& traj)
{
	MANIPULATION_TASK_TYPE_STR type = ARM_FREE;

	bool succeed = false;

	std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
	std::vector <SM_TRAJ> smTrajs;
	std::vector <p3d_traj*> trajs;

	string OBJECT_NAME = "GREY_TAPE";

	vector<double> objGoto,objStart;

	objGoto.resize(6);
//    m_objGoto[0] = 4.23;
//    m_objGoto[1] = -2.22;
//    m_objGoto[2] = 1.00;

    objGoto[0] = P3D_HUGE;
    objGoto[1] = P3D_HUGE;
    objGoto[2] = P3D_HUGE;

    objGoto[3] = P3D_HUGE;
    objGoto[4] = P3D_HUGE;
    objGoto[5] = P3D_HUGE;

	cout << "Manipulation planning for " << OBJECT_NAME << endl;
	MANIPULATION_TASK_MESSAGE status;
	string str = m_ManipPl->robot()->name;

	if( str == "PR2_ROBOT" )
	{
		fixAllJointsWithoutArm(m_ManipPl->robot(),0);
	}

	switch ( (unsigned int) m_ManipPl->robot()->lpl_type )
	{
	case LINEAR :
		{
			gpGrasp grasp;
			status = m_ManipPl->armPlanTask(type,0,qInit,qGoal, objStart, objGoto,
												 /* m_OBJECT_NAME.c_str() */ "", "", (char*)"", grasp, trajs);

			if(status == MANIPULATION_TASK_OK )
			{
				m_ManipPl->robot()->tcur = p3d_create_traj_by_copy(trajs[0]);

				for(unsigned int i = 1; i < trajs.size(); i++){
					p3d_concat_traj(m_ManipPl->robot()->tcur, trajs[i]);
				}

				//m_manipulation->setRobotPath(m_manipulation->robot()->tcur);
			}
			break;
		}

	case MULTI_LOCALPATH : {
			gpGrasp grasp;
			status = m_ManipPl->armPlanTask(type,0,qInit,qGoal, objStart, objGoto,
							 /*m_OBJECT_NAME.c_str()*/ "", "", (char*)"", grasp, confs, smTrajs);
			break;
		}

	case SOFT_MOTION:{
			cout << "Manipulation : localpath softmotion should not be called" << endl;
			succeed = false;
			break;
		}
	}

	if (status != MANIPULATION_TASK_OK )
	{ succeed = false; }
	else
	{ succeed = true; }

	if( succeed )
	{
		traj = API::Trajectory( _Robot, m_ManipPl->robot()->tcur );
	}

	return succeed;
}

std::pair<double,double> OTPMotionPl::computeHumanRobotDist()
{
	pair<double,double> minMax;
	minMax.first = numeric_limits<double>::max( );
	minMax.second = 0;
	std::vector<ConfigHR> list;
	if (PlanEnv->getBool(PlanParam::env_isStanding))
	{
		list = m_configList;
	}
	else
	{
		list = m_sittingConfigList;
	}
	shared_ptr<Configuration> q_human (m_Human->getCurrentPos());
	shared_ptr<Configuration> q_robot (_Robot->getCurrentPos());

	for(unsigned int i = 0; i < list.size(); i++)
	{
		setRobotsToConf(i,PlanEnv->getBool(PlanParam::env_isStanding));
		double dist = getHumanRobotDist();

		if (dist > minMax.second)
		{
			minMax.second = dist;
		}
		if (dist < minMax.first)
		{
			minMax.first = dist;
		}
	}

	m_Human->setAndUpdate(*q_human);
	_Robot->setAndUpdate(*q_robot);


	return minMax;

}

double OTPMotionPl::getHumanRobotDist()
{
	shared_ptr<Configuration> q_human (m_Human->getCurrentPos());
	shared_ptr<Configuration> q_robot (_Robot->getCurrentPos());

	int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
	int firstIndexOfHumanDof = m_Human->getJoint("Pelvis")->getIndexOfFirstDof();

	double xh = (*q_human)[firstIndexOfHumanDof + 0];
	double yh = (*q_human)[firstIndexOfHumanDof + 1];
	double xr = (*q_robot)[firstIndexOfRobotDof + 0];
	double yr = (*q_robot)[firstIndexOfRobotDof + 1];


	return sqrt( pow( xh - xr , 2) + pow( yh - yr , 2) );
}




void ConfigHR::setHumanConf(Robot* human, configPt q)
{
	q_hum = p3d_copy_config(human->getRobotStruct(),q);
}

void ConfigHR::setRobotConf(Robot* robot, configPt q)
{
	q_rob = p3d_copy_config(robot->getRobotStruct(),q);
}

void OutputConf::clearAll()
{
//    humanConf = NULL;
    humanTraj.clear();
    humanTrajExist = false;

//    robotConf = NULL;
    robotTraj.clear();
    robotTrajExist = false;

    cost = numeric_limits<double>::max( );
    configNumberInList = -1;
    id = -1;
    isStandingInThisConf = true; }

//OutputConf& OutputConf::operator= (const OutputConf& o)
//{
//    OutputConf conf;
//    conf.configNumberInList = o.configNumberInList;
//    conf.cost = o.cost;
//    conf.humanConf = o.humanConf;
//    conf.humanTraj = o.humanTraj;
//    conf.humanTrajExist = o.humanTrajExist;
//    conf.id = o.id;
//    conf.isStandingInThisConf = o.isStandingInThisConf;
//    conf.robotConf = o.robotConf;
//    conf.robotTraj = o.robotTraj;
//    conf.robotTrajExist = o.robotTrajExist;
//    return conf;
//}

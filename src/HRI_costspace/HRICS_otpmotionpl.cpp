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

OTPMotionPl::OTPMotionPl() : HumanAwareMotionPlanner() , m_PathExist(false) , m_HumanPathExist(false), m_pathIndex(-1)
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
    initCostSpace();
    initDistance();
    m_VisibilitySpace = new Visibility(m_Human);
}


OTPMotionPl::OTPMotionPl(Robot* R, Robot* H) : HumanAwareMotionPlanner() , m_Human(H) , m_PathExist(false) , m_HumanPathExist(false), m_pathIndex(-1)
{
    this->setRobot(R);
    initCostSpace();
    initDistance();
    m_VisibilitySpace = new Visibility(m_Human);
}

OTPMotionPl::~OTPMotionPl()
{
    delete m_2DGrid;
}


void OTPMotionPl::initCostSpace()
{
    m_EnvSize.resize(4);
    m_EnvSize[0] = XYZ_ENV->box.x1; m_EnvSize[1] = XYZ_ENV->box.x2;
    m_EnvSize[2] = XYZ_ENV->box.y1; m_EnvSize[3] = XYZ_ENV->box.y2;

    m_2DGrid = new EnvGrid(ENV.getDouble(Env::PlanCellSize),m_EnvSize,false);
    m_2DGrid->setRobot(_Robot);
    m_2DGrid->setHuman(m_Human);
}


void OTPMotionPl::initHumanCenteredGrid()
{
    m_EnvSize.resize(4);
    m_EnvSize[0] = XYZ_ENV->box.x1; m_EnvSize[1] = XYZ_ENV->box.x2;
    m_EnvSize[2] = XYZ_ENV->box.y1; m_EnvSize[3] = XYZ_ENV->box.y2;

    m_2DHumanCenteredGrid = new EnvGrid(ENV.getDouble(Env::PlanCellSize),m_EnvSize,true);
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

//        config = _Robot->getGoTo();

//        pos[0] = config->at(firstIndexOfRobotDof + 0);
//        pos[1] = config->at(firstIndexOfRobotDof + 1);

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




    //    Trajectory* traj = new Trajectory(new Robot(XYZ_ROBOT));
    //
    //    traj->replaceP3dTraj();
    //    string str = "g3d_draw_allwin_active";
    //    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    //    ENV.setBool(Env::drawTraj,true);
    //    cout << "solution : End Search" << endl;

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
                return;
            }

            for (int i=path.size()-1;i>=0;i--)
            {
                EnvCell* cell = dynamic_cast<EnvState*>(path[i])->getCell();
                m_2DPath.push_back( cell->getCenter() );
                m_2DCellPath.push_back( cell );
            }
        }
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
                return;
            }

            for (int i=path.size()-1;i>=0;i--)
            {
                EnvCell* cell = dynamic_cast<EnvState*>(path[i])->getCell();
                m_2DHumanPath.push_back( cell->getCenter() );
                m_2DHumanCellPath.push_back( cell );
            }
        }
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

    if( m_HumanPathExist)
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
  initHumanCenteredGrid();
  vector<EnvCell*> sortedCells = m_2DHumanCenteredGrid->getSortedCells();
  
  Vector2d startPos;
  
  for (unsigned int i=0; i < sortedCells.size(); i++)
  {
      if (computeUpBodyOpt())
      {
        _Robot->setGoTo(*_Robot->getCurrentPos());
        _Robot->setAndUpdate(*q_robot_cur);
        
        m_Human->setGoTo(*m_Human->getCurrentPos());
        m_Human->setAndUpdate(*q_human_cur);
        
        return true;
      }
  }
  return false;
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
    initHumanCenteredGrid();
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

bool OTPMotionPl::FindTraj(Vector2d startPos, Vector2d goalPos, bool isHuman)
{

//    p3d_col_activate_robot(_Robot);
    EnvCell* startCell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(startPos));
    EnvState* start = new EnvState(
            startCell,
            m_2DGrid);

    EnvCell* goalCell = dynamic_cast<EnvCell*>(m_2DGrid->getCell(goalPos));
    EnvState* goal = new EnvState(
            goalCell,
            m_2DGrid);

    if ( startCell == NULL ||  goalCell == NULL)
    {
        return false;
    }

    if (goalCell == startCell)
    {
        cout << "Same start and goal cell" << endl;
        return true;
    }
    solveAStar(start,goal,isHuman);

    if (!isHuman)
    {
        double SumOfCost= 0.0;
        for(unsigned int i=0; i< m_2DPath.size() ; i++ )
        {
            SumOfCost +=  dynamic_cast<EnvCell*>(m_2DCellPath[i])->getCost();
        }
        cout << "SumOfCost=" << SumOfCost << endl;

        int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
        shared_ptr<Configuration> q_cur = _Robot->getCurrentPos();
        shared_ptr<Configuration> q_tmp = _Robot->getCurrentPos();

        for (unsigned int i = 0; i < m_2DCellPath.size(); i++)
        {
            Eigen::Vector2d center = m_2DCellPath.at(i)->getCenter();
    //        cout << i << " = " << endl << "abs Pos : "<< endl << center << endl << "coord : " << endl << dynamic_cast<EnvCell*>(m_2DCellPath.at(i))->getCoord() << endl << "---------------" << endl;

            (*q_tmp)[firstIndexOfRobotDof + 0] = center[0];
            (*q_tmp)[firstIndexOfRobotDof + 1] = center[1];
//            _Robot->setAndUpdate(*q_tmp);

            q_tmp->setAsNotTested();
            if (q_tmp->isInCollision())
            {
                _Robot->setAndUpdate(*q_cur);
                return false;
            }
        }
        _Robot->setAndUpdate(*q_cur);
    }
    return true;
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

void OTPMotionPl::initDistance()
{
	vector<Robot*> m_Humans;
	m_Humans.push_back(m_Human);
	m_DistanceSpace = new Distance(_Robot,m_Humans);

	if (_Robot)
	{
		cout << "Robot " << _Robot->getName() << endl;
		cout << "Robot get struct " << _Robot->getRobotStruct() << endl;
	}

	m_DistanceSpace->parseHumans();
}

bool OTPMotionPl::moveToNextPos()
{
	if (m_pathIndex == -1)
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

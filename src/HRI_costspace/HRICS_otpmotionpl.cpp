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

OTPMotionPl::OTPMotionPl() : HumanAwareMotionPlanner() , mPathExist(false) , mHumanPathExist(false), pathIndex(-1)
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
            mHuman = new Robot(XYZ_ENV->robot[i]);
            cout << "Humans is " << name << endl;
        }
    }
    initCostSpace();
    initDistance();
    m_VisibilitySpace = new Visibility(mHuman);
}


OTPMotionPl::OTPMotionPl(Robot* R, Robot* H) : HumanAwareMotionPlanner() , mHuman(H) , mPathExist(false) , mHumanPathExist(false), pathIndex(-1)
{
    this->setRobot(R);
    initCostSpace();
    initDistance();
    m_VisibilitySpace = new Visibility(mHuman);
}

OTPMotionPl::~OTPMotionPl()
{
    delete m2DGrid;
}


void OTPMotionPl::initCostSpace()
{
    mEnvSize.resize(4);
    mEnvSize[0] = XYZ_ENV->box.x1; mEnvSize[1] = XYZ_ENV->box.x2;
    mEnvSize[2] = XYZ_ENV->box.y1; mEnvSize[3] = XYZ_ENV->box.y2;

    m2DGrid = new EnvGrid(ENV.getDouble(Env::PlanCellSize),mEnvSize,false);
    m2DGrid->setRobot(_Robot);
    m2DGrid->setHuman(mHuman);
}


void OTPMotionPl::initHumanCenteredGrid()
{
    mEnvSize.resize(4);
    mEnvSize[0] = XYZ_ENV->box.x1; mEnvSize[1] = XYZ_ENV->box.x2;
    mEnvSize[2] = XYZ_ENV->box.y1; mEnvSize[3] = XYZ_ENV->box.y2;

    M2DHumanCenteredGrid = new EnvGrid(ENV.getDouble(Env::PlanCellSize),mEnvSize,true);
    M2DHumanCenteredGrid->setRobot(_Robot);
    M2DHumanCenteredGrid->setHuman(mHuman);

    API_activeRobotGrid = M2DHumanCenteredGrid;
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

    EnvCell* startCell = dynamic_cast<EnvCell*>(m2DGrid->getCell(pos));
    Vector2i startCoord = startCell->getCoord();

    cout << "Start Pos = (" <<
            pos[0] << " , " <<
            pos[1] << ")" << endl;

    cout << "Start Coord = (" <<
            startCoord[0] << " , " <<
            startCoord[1] << ")" << endl;

    EnvState* start = new EnvState(
            startCell,
            m2DGrid);

    config = _Robot->getGoTo();

    pos[0] = config->at(firstIndexOfRobotDof + 0);
    pos[1] = config->at(firstIndexOfRobotDof + 1);

    EnvCell* goalCell = dynamic_cast<EnvCell*>(m2DGrid->getCell(pos));
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
            m2DGrid);

    solveAStar(start,goal,false);

    double SumOfCost= 0.0;
    for(unsigned int i=0; i< m2DPath.size() ; i++ )
    {
        SumOfCost +=  dynamic_cast<EnvCell*>(m2DCellPath[i])->getCost();
    }

    // testing trajectory
    shared_ptr<Configuration> q_cur = _Robot->getInitialPosition();
    shared_ptr<Configuration> q = _Robot->getInitialPosition();

    cout << " SumOfCost = "  << SumOfCost << endl;
    cout << " The Robot will move throuth the followings cells " << endl;

    for (unsigned int i = 0; i < m2DCellPath.size(); i++)
    {
        Eigen::Vector2d center = m2DCellPath.at(i)->getCenter();
//        cout << i << " = " << endl << "abs Pos : "<< endl << center << endl << "coord : " << endl << dynamic_cast<EnvCell*>(m2DCellPath.at(i))->getCoord() << endl << "---------------" << endl;

        (*q)[firstIndexOfRobotDof + 0] = center[0];
        (*q)[firstIndexOfRobotDof + 1] = center[1];
        _Robot->setAndUpdate(*q);
        if (_Robot->isInCollision())
        {
            m2DCellPath.resize(i);
            m2DPath.resize(i);
            break;
        }
    }
    _Robot->setAndUpdate(*q_cur);

    API::TwoDCell* lastReachedCell = m2DCellPath.at(m2DCellPath.size()-1);

    if (lastReachedCell != goalCell)
    {
        cout << "The cell that the robot should access to is unreacheable." <<
                "Motion Planing for the human." << endl;

        config = mHuman->getInitialPosition();

//        config->print();

        int firstIndexOfHumanDof = mHuman->getJoint("Pelvis")->getIndexOfFirstDof();

        pos[0] = config->at(firstIndexOfHumanDof + 0);
        pos[1] = config->at(firstIndexOfHumanDof + 1);

        EnvCell* startHumanCell = dynamic_cast<EnvCell*>(m2DGrid->getCell(pos));
        Vector2i startHumanCoord = startHumanCell->getCoord();

        cout << "Start Human Pos = (" <<
                pos[0] << " , " <<
                pos[1] << ")" << endl;

        cout << "Start Human Coord = (" <<
                startHumanCoord[0] << " , " <<
                startHumanCoord[1] << ")" << endl;

        EnvState* startHuman = new EnvState(
                startHumanCell,
                m2DGrid);

//        config = _Robot->getGoTo();

//        pos[0] = config->at(firstIndexOfRobotDof + 0);
//        pos[1] = config->at(firstIndexOfRobotDof + 1);

        EnvCell* goalHumanCell = dynamic_cast<EnvCell*>(lastReachedCell);//(m2DGrid->getCell(pos));
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
                m2DGrid);

        solveAStar(startHuman,goalHuman, true);

        double SumOfCost= 0.0;
        for(unsigned int i=0; i< m2DPath.size() ; i++ )
        {
            SumOfCost +=  dynamic_cast<EnvCell*>(m2DCellPath[i])->getCost();
        }

        // testing trajectory
        shared_ptr<Configuration> q_human_cur = mHuman->getCurrentPos();
        shared_ptr<Configuration> q_human = _Robot->getCurrentPos();

        cout << " SumOfCost = "  << SumOfCost << endl;
        for (unsigned int i = 0; i < m2DHumanCellPath.size(); i++)
        {
            Eigen::Vector2d center = m2DHumanCellPath.at(i)->getCenter();
//            cout << i << " = " << endl << "abs Pos : "<< endl << center << endl << "coord : " << endl << dynamic_cast<EnvCell*>(m2DHumanCellPath.at(i))->getCoord() << endl << "---------------" << endl;

            (*q_human)[firstIndexOfHumanDof + 0] = center[0];
            (*q_human)[firstIndexOfHumanDof + 1] = center[1];
            mHuman->setAndUpdate(*q_human);
            if (mHuman->isInCollision())
            {
                m2DHumanCellPath.resize(i);
                m2DHumanPath.resize(i);
                break;
            }
        }
        mHuman->setAndUpdate(*q_human_cur);

    }
    else
    {
        m2DHumanPath.clear();

        m2DHumanCellPath.clear();
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
        m2DPath.clear();

        m2DCellPath.clear();

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

                m2DPath.clear();
                m2DCellPath.clear();
                mPathExist = false;
                return;

            }

            for (unsigned int i=0;i<path.size();i++)
            {
                API::TwoDCell* cell = dynamic_cast<EnvState*>(path[i])->getCell();
                m2DPath.push_back( cell->getCenter() );
                m2DCellPath.push_back( cell );
            }
        }
        else
        {
            API::AStar* search = new API::AStar(goal);
            vector<API::State*> path = search->solve(start);

            if(path.size() == 0 )
            {
                m2DPath.clear();
                m2DCellPath.clear();
                mPathExist = false;
                return;
            }

            for (int i=path.size()-1;i>=0;i--)
            {
                EnvCell* cell = dynamic_cast<EnvState*>(path[i])->getCell();
                m2DPath.push_back( cell->getCenter() );
                m2DCellPath.push_back( cell );
            }
        }
        mPathExist = true;
        return;
    }
    else
    {
        m2DHumanPath.clear();

        m2DHumanCellPath.clear();

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

                m2DHumanPath.clear();
                m2DHumanCellPath.clear();
                mHumanPathExist = false;
                return;

            }

            for (unsigned int i=0;i<path.size();i++)
            {
                API::TwoDCell* cell = dynamic_cast<EnvState*>(path[i])->getCell();
                m2DHumanPath.push_back( cell->getCenter() );
                m2DHumanCellPath.push_back( cell );
            }
        }
        else
        {
            API::AStar* search = new API::AStar(goal);
            vector<API::State*> path = search->solve(start);

            if(path.size() == 0 )
            {
                m2DHumanPath.clear();
                m2DHumanCellPath.clear();
                mHumanPathExist = false;
                return;
            }

            for (int i=path.size()-1;i>=0;i--)
            {
                EnvCell* cell = dynamic_cast<EnvState*>(path[i])->getCell();
                m2DHumanPath.push_back( cell->getCenter() );
                m2DHumanCellPath.push_back( cell );
            }
        }
        mHumanPathExist = true;
        return;
    }
}

/**
  * Draws the 3D path as a yellow line for robot and green one for human
  */
void OTPMotionPl::draw2dPath()
{
    if( mPathExist)
    {
//        cout << "Drawing 2D path" << endl;
        for(unsigned int i=0;i<m2DPath.size()-1;i++)
        {
            glLineWidth(3.);
            g3d_drawOneLine(m2DPath[i][0],      m2DPath[i][1],      0.4,
                            m2DPath[i+1][0],    m2DPath[i+1][1],    0.4,
                            Yellow, NULL);
            glLineWidth(1.);
        }
    }

    if( mHumanPathExist)
    {
//        cout << "Drawing 2D path" << endl;
        for(unsigned int i=0;i<m2DHumanPath.size()-1;i++)
        {
            glLineWidth(3.);
            g3d_drawOneLine(m2DHumanPath[i][0],      m2DHumanPath[i][1],      0.4,
                            m2DHumanPath[i+1][0],    m2DHumanPath[i+1][1],    0.4,
                            Green, NULL);
            glLineWidth(1.);
        }
    }
}




bool OTPMotionPl::computeObjectTransfertPoint()
{
    cout << "Compute the OTP" << endl;

    m2DGrid->setCellsToblankCost();

    shared_ptr<Configuration> q_human_cur = mHuman->getCurrentPos();
    shared_ptr<Configuration> q_robot_cur = _Robot->getCurrentPos();

    mHuman->setInitialPosition(*q_human_cur);
    _Robot->setInitialPosition(*q_robot_cur);

    cout << "This function is separeted in ? parts : " << endl;
    cout << "First Part" << endl;

    cout << "Second part : Motion Planing for robot (and human if nessessary)" << endl;

//    ENV.setDouble(Env::robotMaximalDistFactor,0.0);
    initHumanCenteredGrid();
    vector<EnvCell*> sortedCells = M2DHumanCenteredGrid->getSortedCells();

    Vector2d startPos;
    for (unsigned int i=0; i < sortedCells.size(); i++)
    {
        cout << i <<" = "<< sortedCells.at(i)->getCost() << "---------------------------------------" << endl;

        int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
        int firstIndexOfHumanDof = mHuman->getJoint("Pelvis")->getIndexOfFirstDof();

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
//            M2DHumanCenteredGrid->recomputeCostRobotOnly();
            startPos[0] = q_human_cur->at(firstIndexOfHumanDof + 0);
            startPos[1] = q_human_cur->at(firstIndexOfHumanDof + 1);

            goalPos[0] = m2DCellPath.at(m2DCellPath.size()-1)->getCenter()[0];
            goalPos[1] = m2DCellPath.at(m2DCellPath.size()-1)->getCenter()[1];

            FindTraj(startPos,goalPos,true);
            shared_ptr<Configuration> q_human = mHuman->getCurrentPos();

            for (unsigned int i = 0; i < m2DHumanCellPath.size(); i++)
            {
                Eigen::Vector2d center = m2DHumanCellPath.at(i)->getCenter();
    //            cout << i << " = " << endl << "abs Pos : "<< endl << center << endl << "coord : " << endl << dynamic_cast<EnvCell*>(m2DHumanCellPath.at(i))->getCoord() << endl << "---------------" << endl;

                (*q_human)[firstIndexOfHumanDof + 0] = center[0];
                (*q_human)[firstIndexOfHumanDof + 1] = center[1];
                mHuman->setAndUpdate(*q_human);
                q_human->setAsNotTested();

                if (q_human->isInCollision())
                {
                    m2DHumanCellPath.resize(i);
                    m2DHumanPath.resize(i);
                }
            }
            mHuman->setAndUpdate(*q_human_cur);

             //compute the OTP for the configs found.
            shared_ptr<Configuration> q_human_otp = mHuman->getCurrentPos();
            shared_ptr<Configuration> q_robot_otp = _Robot->getCurrentPos();

            (*q_human_otp)[firstIndexOfHumanDof + 0] = m2DHumanCellPath.at(m2DHumanCellPath.size()-1)->getCenter()[0];
            (*q_human_otp)[firstIndexOfHumanDof + 1] = m2DHumanCellPath.at(m2DHumanCellPath.size()-1)->getCenter()[1];
            mHuman->setAndUpdate(*q_human_otp);

            (*q_robot_otp)[firstIndexOfRobotDof + 0] = m2DCellPath.at(m2DCellPath.size()-1)->getCenter()[0];
            (*q_robot_otp)[firstIndexOfRobotDof + 1] = m2DCellPath.at(m2DCellPath.size()-1)->getCenter()[1];
            _Robot->setAndUpdate(*q_robot_otp);

            if (computeUpBodyOpt())
            {
                _Robot->setGoTo(*_Robot->getCurrentPos());
                _Robot->setAndUpdate(*q_robot_cur);

                mHuman->setGoTo(*mHuman->getCurrentPos());
                mHuman->setAndUpdate(*q_human_cur);

                return true;
            }

        }
    }
    return false;

}

bool OTPMotionPl::FindTraj(Vector2d startPos, Vector2d goalPos, bool isHuman)
{

//    p3d_col_activate_robot(_Robot);
    EnvCell* startCell = dynamic_cast<EnvCell*>(m2DGrid->getCell(startPos));
    EnvState* start = new EnvState(
            startCell,
            m2DGrid);

    EnvCell* goalCell = dynamic_cast<EnvCell*>(m2DGrid->getCell(goalPos));
    EnvState* goal = new EnvState(
            goalCell,
            m2DGrid);

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
        for(unsigned int i=0; i< m2DPath.size() ; i++ )
        {
            SumOfCost +=  dynamic_cast<EnvCell*>(m2DCellPath[i])->getCost();
        }
        cout << "SumOfCost=" << SumOfCost << endl;

        int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
        shared_ptr<Configuration> q_cur = _Robot->getCurrentPos();
        shared_ptr<Configuration> q_tmp = _Robot->getCurrentPos();

        for (unsigned int i = 0; i < m2DCellPath.size(); i++)
        {
            Eigen::Vector2d center = m2DCellPath.at(i)->getCenter();
    //        cout << i << " = " << endl << "abs Pos : "<< endl << center << endl << "coord : " << endl << dynamic_cast<EnvCell*>(m2DCellPath.at(i))->getCoord() << endl << "---------------" << endl;

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
	vector<Robot*> mHumans;
	mHumans.push_back(mHuman);
	m_DistanceSpace = new Distance(_Robot,mHumans);

	if (_Robot)
	{
		cout << "Robot " << _Robot->getName() << endl;
		cout << "Robot get struct " << _Robot->getRobotStruct() << endl;
	}

	m_DistanceSpace->parseHumans();
}

bool OTPMotionPl::moveToNextPos()
{
	if (pathIndex == -1)
	{
		pathIndex = 0;
	}

	bool robotPathLimit = true;
	if (mPathExist && pathIndex < m2DCellPath.size())
	{
		int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;
		shared_ptr<Configuration> q_cur = _Robot->getCurrentPos();
		(*q_cur)[firstIndexOfRobotDof + 0] = m2DCellPath[pathIndex]->getCenter()[0];
		(*q_cur)[firstIndexOfRobotDof + 1] = m2DCellPath[pathIndex]->getCenter()[1];
		_Robot->setAndUpdate(*q_cur);
		robotPathLimit = false;
	}

	bool humanPathLimit = true;
	if (mHumanPathExist && pathIndex < m2DHumanCellPath.size())
	{
		int firstIndexOfHumanDof = mHuman->getJoint("Pelvis")->getIndexOfFirstDof();
		shared_ptr<Configuration> q_cur = mHuman->getCurrentPos();
		(*q_cur)[firstIndexOfHumanDof + 0] = m2DHumanCellPath[pathIndex]->getCenter()[0];
		(*q_cur)[firstIndexOfHumanDof + 1] = m2DHumanCellPath[pathIndex]->getCenter()[1];
		mHuman->setAndUpdate(*q_cur);
		humanPathLimit = false;
	}
	cout << pathIndex++ << endl;

	if (humanPathLimit)
	{
		mHuman->setAndUpdate(*mHuman->getGoTo());
	}


	return robotPathLimit && humanPathLimit;

}

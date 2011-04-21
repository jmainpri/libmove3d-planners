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

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

extern HRICS::HumanAwareMotionPlanner*		HRICS_MotionPLConfig;
extern string global_ActiveRobotName;

OTPMotionPl::OTPMotionPl() : HumanAwareMotionPlanner() , mPathExist(false) , mHumanPathExist(false)
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
}


OTPMotionPl::OTPMotionPl(Robot* R, Robot* H) : HumanAwareMotionPlanner() , mHuman(H) , mPathExist(false) , mHumanPathExist(false)
{
    this->setRobot(R);
    initCostSpace();
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

    m2DGrid = new EnvGrid(ENV.getDouble(Env::PlanCellSize),mEnvSize);
    m2DGrid->setRobot(_Robot);
}



/**
  * Takes the robot initial config and calls the solve A*
  * to compute the 2D path
  */
bool OTPMotionPl::computeAStarIn2DGrid()
{
    ENV.setBool(Env::drawOTPTraj,false);

    shared_ptr<Configuration> config = _Robot->getInitialPosition();

    config->print();

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
        cout << i << " = " << endl << "abs Pos : "<< endl << center << endl << "coord : " << endl << dynamic_cast<EnvCell*>(m2DCellPath.at(i))->getCoord() << endl << "---------------" << endl;

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
        shared_ptr<Configuration> q_human_cur = mHuman->getInitialPosition();
        shared_ptr<Configuration> q_human = _Robot->getInitialPosition();

        cout << " SumOfCost = "  << SumOfCost << endl;
        for (unsigned int i = 0; i < m2DHumanCellPath.size(); i++)
        {
            Eigen::Vector2d center = m2DHumanCellPath.at(i)->getCenter();
            cout << i << " = " << endl << "abs Pos : "<< endl << center << endl << "coord : " << endl << dynamic_cast<EnvCell*>(m2DHumanCellPath.at(i))->getCoord() << endl << "---------------" << endl;

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

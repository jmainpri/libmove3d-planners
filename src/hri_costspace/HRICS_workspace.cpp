/*
 *  HRICS_CostSpace.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 04/12/09.
 *  Copyright 2009 LAAS/CNRS. All rights reserved.
 *
 */

#include "HRICS_costspace.hpp"
#include "grid/HRICS_grid.hpp"
#include "grid/HRICS_grid_state.hpp"
#include "RRT/HRICS_rrt.hpp"
#include "RRT/HRICS_rrtExpansion.hpp"

#include "planner/Diffusion/Variants/Transition-RRT.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"

#include "API/project.hpp"

//#include "../../qtWindow/cppToQt.hpp"

//#include <Eigen/Dense>

#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Collision-pkg.h>
#include <libmove3d/include/Planner-pkg.h>

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

#ifdef HRI_PLANNER
HRICS::HriSpaceCost* hriSpace = NULL;
#endif

HRICS::Distance*		HRICS_activeDist = NULL;
HRICS::Visibility*      HRICS_activeVisi = NULL;
HRICS::Natural*			HRICS_activeNatu = NULL;
HRICS::Natural*			HRICS_activeReac = NULL;
HRICS::Legibility*   	HRICS_activeLegi = NULL;

HRICS::HumanAwareMotionPlanner*		HRICS_MotionPL = NULL;

Move3D::ThreeDCell*	BiasedCell3D = NULL;
Move3D::TwoDCell*		BiasedCell2D = NULL;

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE
using namespace HRICS;
using namespace Move3D;

// import most common Eigen types 
//using namespace Eigen;
using namespace Eigen;

extern string global_ActiveRobotName;
Eigen::Vector3d current_WSPoint;
pair<double,Eigen::Vector3d > current_cost;

Workspace::Workspace() : HumanAwareMotionPlanner() , mPathExist(false)
{
    mHumans.clear();
    _Robot = NULL;
    cout << "New Workspace" << endl;

    current_WSPoint << 0.0 ,0.0, 0.0;

    Scene* environnement = global_Project->getActiveScene();

    this->setRobot(	   environnement->getRobotByNameContaining(global_ActiveRobotName) );
    mHumans.push_back( environnement->getRobotByNameContaining("HUMAN") );

    if (_Robot)
    {
        cout << "Robot is " << _Robot->getName() << endl;
    }
    else
    {
        cout << "No Robot in Workspace planner" << endl;
    }

    cout << "Human is " << mHumans[0]->getName() << endl;

#ifdef LIGHT_PLANNER
    if(_Robot)
    {
        if((*_Robot->getP3dRobotStruct()->armManipulationData).size() >= 1 )
        {
            p3d_jnt* FF_Joint = (*_Robot->getP3dRobotStruct()->armManipulationData)[0].getManipulationJnt();
            ENV.setInt(Env::akinJntId,FF_Joint->num);
            mIndexObjectDof = FF_Joint->index_dof;//_Robot->getP3dRobotStruct()->baseJnt;
        }
        else
        {
            cout << "Error : Manipulation Data uninitialized" << endl;
        }
    }
#else
    cout << "Warning: Lihght Planner not compiled" << endl;
#endif

    //#ifdef P3D_PLANNER
    //	p3d_del_graph(XYZ_GRAPH);
    //#endif
    //
    //	XYZ_GRAPH = NULL;

    if(_Robot)
    {
        cout << "VIRTUAL Joint is " << ENV.getInt(Env::akinJntId) << endl;
        cout << "VIRTUAL_OBJECT_DOF Joint is " << mIndexObjectDof << endl;
        cout << "HRI Cost type is "  << ENV.getInt(Env::hriCostType) << endl;
        cout << "Ball Dist is " << ENV.getBool(Env::useBallDist) << endl;

        _Graph = new Graph(_Robot,XYZ_GRAPH);
    }
    else
    {
        ENV.setBool(Env::HRINoRobot,true);
    }


    m3DPath.clear();

    m_OTP = Vector3d::Zero();

    mIndexObjectDof = 0;
}

Workspace::Workspace(Robot* rob, Graph* graph) :
    HumanAwareMotionPlanner(rob, graph) , mPathExist(false)
{
    cout << "Robot is " << rob->getName() << endl;
    current_WSPoint << 0.0 ,0.0, 0.0;

    if(rob->getName().find(global_ActiveRobotName) == string::npos )
    {
        cout << "Workspace::Error robot des not contain ROBOT" << endl;
    }

    Scene* environnement = global_Project->getActiveScene();

    mHumans.push_back( environnement->getRobotByNameContaining("HUMAN") );
    cout << "Human is " << mHumans[0]->getName() << endl;

    m3DPath.clear();

    mIndexObjectDof = 0;
}

Workspace::~Workspace()
{
    if ( this->getGrid() == API_activeGrid)
    {
        API_activeGrid = NULL;
    }

    delete (this->getGrid());
}


void Workspace::initGrid()
{
    //    vector<int> size;
    vector<double>  envSize(6);
    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
    envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;

    double pace = ENV.getDouble(Env::CellSize);

    //    GridToGraph theGrid(pace,envSize);
    //    theGrid.putGridInGraph();

    m3DGrid = new Grid(pace,envSize);

    m3DGrid->setRobot(_Robot);

    BiasedCell3D = m3DGrid->getCell(0,0,0);
    cout << "Biased Cell is " << BiasedCell3D << endl;
    //#ifdef QT_LIBRARY
    //    std::string str = "g3d_draw_allwin_active";
    //    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    // #endif
}

void Workspace::initAgentGrids(double cellsize)
{
    deleteAgentGrids();

    vector<double>  envsize(6);

    envsize[0] = -3; envsize[1] = 3;
    envsize[2] = -3; envsize[3] = 3;
    envsize[4] = -3; envsize[5] = 3;

    for ( int i=0; i<int(mHumans.size()); i++)
    {
        m_HumanGrids.push_back( new AgentGrid( cellsize, envsize,
                                               mHumans[i],m_DistanceSpace,m_VisibilitySpace,m_ReachableSpace) );

        m_HumanGrids[i]->computeAllCellCost();
    }

    API_activeGrid = m_HumanGrids[0];
}

void Workspace::deleteAgentGrids()
{
    API_activeGrid = NULL;

    for ( int i=0; i<int(m_HumanGrids.size()); i++)
    {
        delete m_HumanGrids[i];
    }
    m_HumanGrids.clear();
}

void Workspace::initDistance()
{
    m_DistanceSpace = new Distance(_Robot,mHumans);

    if (_Robot)
    {
        cout << "Robot " << _Robot->getName() << endl;
        cout << "Robot get struct " << _Robot->getP3dRobotStruct() << endl;
    }

    m_DistanceSpace->parseHumans();
}

void Workspace::initVisibility()
{
    m_VisibilitySpace = new Visibility(mHumans[0]);
}

void Workspace::initReachable()
{
    //m_ReachableSpace = new Natural( mHumans[0] );
    m_ReachableSpace = NULL;
}

void Workspace::initNatural()
{
    //m_NaturalSpace = new Natural( _Robot );
    m_NaturalSpace = NULL;
}

void Workspace::initOtpPlanner()
{
    HRICS_MotionPLConfig  = new HRICS::OTPMotionPl;
    HRICS_activeDist = HRICS_MotionPL->getDistance();

    //	ENV.setBool(Env::HRIPlannerCS,true);
    ENV.setBool(Env::enableHri,true);
    ENV.setBool(Env::isCostSpace,true);

    ENV.setBool(Env::useBallDist,false);
    ENV.setBool(Env::useBoxDist,true);

    API_activeGrid = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid();

    ENV.setBool(Env::drawGrid,true);
    //		m_mainWindow->setBoolFloor(false);
    //		m_mainWindow->drawAllWinActive();
}

Eigen::Vector3d Workspace::getVisball()
{
    Eigen::Vector3d pos;
    Robot* visball = global_Project->getActiveScene()->getRobotByName("VISBALL");

    if (visball) {
        confPtr_t q = visball->getCurrentPos();
        pos[0] = (*q)[6];
        pos[1] = (*q)[7];
        pos[2] = (*q)[8];
    }
    return pos;
}

/**
 * Takes the robot initial config and calls the solve A*
 * to compute the 3D path
 */
bool Workspace::computeAStarIn3DGrid()
{
    //	if(!ENV.getBool(Env::isHriTS))
    //	{
    //		return this->computeAStar();
    //	}
    //
    ENV.setBool(Env::drawTraj,false);

    confPtr_t config = _Robot->getInitPos();

    config->print();

    Vector3d pos;

    pos[0] = config->at(mIndexObjectDof+0);
    pos[1] = config->at(mIndexObjectDof+1);
    pos[2] = config->at(mIndexObjectDof+2);

    Cell* startCell = dynamic_cast<Cell*>(m3DGrid->getCell(pos));
    Vector3i startCoord = startCell->getCoord();

    cout << "Start Pos = (" <<
            pos[0] << " , " <<
            pos[1] << " , " <<
            pos[2] << ")" << endl;

    cout << "Start Coord = (" <<
            startCoord[0] << " , " <<
            startCoord[1] << " , " <<
            startCoord[2] << ")" << endl;

    State* start = new State(
                startCell,
                m3DGrid);

    config = _Robot->getGoalPos();

    pos[0] = config->at(mIndexObjectDof+0);
    pos[1] = config->at(mIndexObjectDof+1);
    pos[2] = config->at(mIndexObjectDof+2);

    Cell* goalCell = dynamic_cast<Cell*>(m3DGrid->getCell(pos));
    Vector3i goalCoord = goalCell->getCoord();

    cout << "Goal Pos = (" <<
            pos[0] << " , " <<
            pos[1] << " , " <<
            pos[2] << ")" << endl;

    cout << "Goal Coord = (" <<
            goalCoord[0] << " , " <<
            goalCoord[1] << " , " <<
            goalCoord[2] << ")" << endl;

    if( startCoord == goalCoord )
    {
        cout << " no planning as cells are identical" << endl;
        return false;
    }

    State* goal = new State(
                goalCell,
                m3DGrid);

    solveAStar(start,goal);

    if(mPathExist)
    {
        cout << " Path Cost = "  << pathCost() <<  endl;
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

/**
 * Solve A Star in a 3D grid using the API A Star on
 * takes as input A* States
 */
void Workspace::solveAStar(State* start,State* goal)
{
    m3DPath.clear();
    m3DCellPath.clear();

    confPtr_t config = _Robot->getCurrentPos();

    /*
     * Change the way AStar
     * is computed to go down
     */
    if( start->getCell()->getCost() < goal->getCell()->getCost() )
    {
        Move3D::AStar* search = new Move3D::AStar(start);
        vector<Move3D::State*> path = search->solve(goal);

        if(path.size() == 0 )
        {
            m3DPath.clear();
            m3DCellPath.clear();
            mPathExist = false;
            return;
        }

        for (unsigned int i=0;i<path.size();i++)
        {
            Move3D::ThreeDCell* cell = dynamic_cast<State*>(path[i])->getCell();
            m3DPath.push_back( cell->getCenter() );
            m3DCellPath.push_back( cell );
        }
    }
    else
    {
        Move3D::AStar* search = new Move3D::AStar(goal);
        vector<Move3D::State*> path = search->solve(start);

        if(path.size() == 0 )
        {
            m3DPath.clear();
            m3DCellPath.clear();
            mPathExist = false;
            return;
        }

        for (int i=path.size()-1;i>=0;i--)
        {
            Move3D::ThreeDCell* cell = dynamic_cast<State*>(path[i])->getCell();
            m3DPath.push_back( cell->getCenter() );
            m3DCellPath.push_back( cell );
        }
    }

    mPathExist = true;
    return;
}

double Workspace::pathCost()
{
    if( m3DPath.size() != m3DCellPath.size() )
    {
        cout << "Error:pathCost() => m3DPath.size() != m3DCellPath.size()" << endl;
    }
    Vector3d currentPos, prevPos;
    double currentCost, prevCost;

    prevCost = dynamic_cast<Cell*>(m3DCellPath[0])->getCost();
    prevPos = m3DPath[0];

    double SumOfCost=0.0;
    double distStep;

    for (unsigned int i = 1; i < m3DPath.size(); i++)
    {
        currentCost = dynamic_cast<Cell*>(m3DCellPath[i])->getCost();
        currentPos = m3DPath[i];

        // Case of task space
        distStep = ( currentPos - prevPos ).norm();

        //        cout << "Current Cost = " << currentCost << endl;
        //        cout << "Delta Cost = " << p3d_ComputeDeltaStepCost(prevCost, currentCost, distStep) << endl;

        SumOfCost += global_costSpace->deltaStepCost(prevCost, currentCost, distStep);

        prevCost = currentCost;
        prevPos = currentPos;
    }

    return SumOfCost;
}

/**
 * Draws the 3D path as a yellow line
 */
void Workspace::draw3dPath()
{
    if( mPathExist )
    {
        for(unsigned int i=0;i<m3DPath.size()-1;i++)
        {
            glLineWidth(3.);
            g3d_drawOneLine(m3DPath[i][0],      m3DPath[i][1],      m3DPath[i][2],
                            m3DPath[i+1][0],    m3DPath[i+1][1],    m3DPath[i+1][2],
                            Yellow, NULL);
            glLineWidth(1.);
        }
    }
}

/**
 * Computes a distance from the robot
 * Current config to the 3D path
 */
double Workspace::distanceToEntirePath()
{
    double minDist = numeric_limits<double>::max();

    Vector3d point;
    Vector3d interPolSaved;

    confPtr_t config = _Robot->getCurrentPos();

    point[0] = config->at(mIndexObjectDof+0);
    point[1] = config->at(mIndexObjectDof+1);
    point[2] = config->at(mIndexObjectDof+2);

    double nbSamples = 20;

    for(unsigned int i=0;i<m3DPath.size()-1;i++)
    {
        for(int j=0;j<nbSamples;j++)
        {
            double alpha = (double)(j/nbSamples);
            Vector3d interpol = m3DPath[i] + alpha*(m3DPath[i+1] - m3DPath[i]);
            double dist = ( point - interpol ).norm();
            if(minDist > dist )
            {
                minDist = dist;
                interPolSaved = interpol;
            }
        }
    }

    if(ENV.getBool(Env::drawDistance))
    {
        vector<double> vect_jim;

        vect_jim.push_back(point[0]);
        vect_jim.push_back(point[1]);
        vect_jim.push_back(point[2]);
        vect_jim.push_back(interPolSaved[0]);
        vect_jim.push_back(interPolSaved[1]);
        vect_jim.push_back(interPolSaved[2]);

        m_DistanceSpace->setVector(vect_jim);
    }

    //    cout << "minDist = " <<minDist << endl;
    return 100*minDist;
}


/**
 * Computes a distance to the Cells in the 3D Path
 * Coarse grain compared to the above distance
 */
double Workspace::distanceToCellPath()
{
    double minDist = numeric_limits<double>::max();

    Vector3d point;

    confPtr_t config = _Robot->getCurrentPos();

    point[0] = config->at(mIndexObjectDof+0);
    point[1] = config->at(mIndexObjectDof+1);
    point[2] = config->at(mIndexObjectDof+2);

    for(unsigned int i=0;i<m3DPath.size();i++)
    {
        double dist = ( point - m3DPath[i] ).norm();
        if( minDist > dist )
        {
            minDist = dist;
        }
    }

    cout << "minDist = " <<minDist << endl;
    return 100*minDist;
}

/**
 * Runs a HRI RRT
 */
bool Workspace::initHriRRT()
{
#ifdef P3D_PLANNER
    p3d_del_graph(XYZ_GRAPH);
#endif
    XYZ_GRAPH = NULL;
    _Graph = new Graph(this->getActivRobot(),XYZ_GRAPH);

    if(ENV.getBool(Env::drawPoints)&&PointsToDraw)
    {
        delete PointsToDraw;
        PointsToDraw = NULL;
    }

    //    ENV.setBool(Env::costBeforeColl,true);
#ifdef LIGHT_PLANNER
    if(ENV.getBool(Env::isInverseKinematics))
    {
        activateCcCntrts(_Robot->getP3dRobotStruct(),-1,true);
    }
    else
    {
        deactivateCcCntrts(_Robot->getP3dRobotStruct(),-1);//true);
    }
#else
    cout << "Warning: Lihght Planner not compiled" << endl;
#endif

    cout << " -----------------------------------------------" << endl;
    cout << " HRISCS Workspace RRT Initialized : "  << endl;
    cout << " Inverse Kinemactics : " << ENV.getBool(Env::isInverseKinematics) << endl;
    cout << " Number of Cell : "  << m3DCellPath.size() << endl;
    cout << " Path Cost : "  << pathCost() << endl;

    return true;
}

void Workspace::activateOnlyBaseCollision()
{
    p3d_col_deactivate_rob(_Robot->getP3dRobotStruct());
    p3d_col_deactivate_rob_env(_Robot->getP3dRobotStruct());
    p3d_col_deactivate_rob_rob(_Robot->getP3dRobotStruct(),mHumans[0]->getP3dRobotStruct());

    p3d_col_activate_obj_env( static_cast<p3d_jnt*>( _Robot->getJoint(1)->getP3dJointStruct() )->o);
    p3d_col_activate_obj_rob( static_cast<p3d_jnt*>( _Robot->getJoint(1)->getP3dJointStruct() )->o,mHumans[0]->getP3dRobotStruct());
}

void Workspace::deactivateOnlyBaseCollision()
{
    p3d_col_deactivate_rob(_Robot->getP3dRobotStruct());
    p3d_col_deactivate_rob_env(_Robot->getP3dRobotStruct());
    p3d_col_deactivate_rob_rob(_Robot->getP3dRobotStruct(),mHumans[0]->getP3dRobotStruct());

    p3d_col_activate_rob(_Robot->getP3dRobotStruct());
    p3d_col_activate_rob_env(_Robot->getP3dRobotStruct());
    p3d_col_activate_rob_rob(_Robot->getP3dRobotStruct(),mHumans[0]->getP3dRobotStruct());
}

bool Workspace::transPFromBaseConf(confPtr_t q_base, vector< Vector3d > points )
{
    confPtr_t  q_actual = _Robot->getCurrentPos(); //pour memoriser la configuration courante du robot

    //On met à jour la configuration du robot pour que sa base soit dans la configuration
    //souhaitée:
    _Robot->setAndUpdate(*q_base);

    //pour chaque point de la liste:
    for(unsigned int i=0; i < points.size(); i++)
    {
        Vector3d WSPoint = points[i];

        (*q_base)[mIndexObjectDof+0] = WSPoint[0];
        (*q_base)[mIndexObjectDof+1] = WSPoint[1];
        (*q_base)[mIndexObjectDof+2] = WSPoint[2];

        for(unsigned int j=0;j<15;j++)
        {
            Vector3d randomPoint(Vector3d::Random());

            //			(*q_base)[mIndexObjectDof+3] = randomPoint[0];
            //			(*q_base)[mIndexObjectDof+4] = randomPoint[1];
            (*q_base)[mIndexObjectDof+5] = M_PI*randomPoint[2];

            if( q_base->setConstraintsWithSideEffect() )
            {
                q_base->setAsNotTested();
                if( !q_base->isInCollision() )
                {
                    //cout << "WSPoint =" << endl << WSPoint << endl;
                    //cout << "Configuration found!!!" << endl;
                    //_Robot->setAndUpdate(*q_base);
                    return true;
                }
            }
        }
    }
    _Robot->setAndUpdate(*q_actual);
    return false;
}

bool Workspace::testCol(confPtr_t q_base)
{
    //	confPtr_t q_base = _Robot->getCurrentPos();
    bool jidoBaseActivation = false;
    if (q_base->getRobot()->getName().find("JIDO") != string::npos)
    {
        jidoBaseActivation = true;
    }

    if (jidoBaseActivation)
    {
        activateOnlyBaseCollision();
    }
    q_base->setAsNotTested();

    if (q_base->isInCollision())
    {
        if (jidoBaseActivation)
        {
            deactivateOnlyBaseCollision();
        }
        return true;
    }
    if (jidoBaseActivation)
    {
        deactivateOnlyBaseCollision();
    }
    return false;
}

/*!
 * Natural cell comparator
 */
class NaturalPoints
{
public:

    bool operator()(pair<double,Vector3d> first, pair<double,Vector3d> second)
    {
        return ( first.first < second.first );
    }

} NaturalPointsCompObject;

const double HRICS_innerRadius = 1.3;
const double HRICS_outerRadius = 1.6;


bool Workspace::findGrapingPosition(std::string robot_name, std::vector<std::string> objects)
{
//    double innerRadius = 0.4;
//    double outerRadius = 1;
    int nbOfTests = 20;

    if (objects.size() > 1)
    {
        cout << "WARNING: if there is more than one object and the objects are too far one from another, finding" <<
                "a grasping position might be ipossible" << endl;
    }
//    Scene* sce = global_Project->getActiveScene();
//    Robot* robot = sce->getRobotByNameContaining(robot_name);

    std::vector<Robot*> objs;
    for (unsigned int i = 0; i < objects.size(); i++)
    {
        for (int j=0; j<XYZ_ENV->nr; j++)
        {
            string name(XYZ_ENV->robot[j]->name);
            if(name.find(objects.at(i)) != string::npos )
            {
                //                objs.push_back(XYZ_ENV->robot[j]);
                break;
            }
        }
    }

    cout << "TODO: find how to change hardcoded distances to computed one form real IK" << endl;
//    double center = (innerRadius + outerRadius)/2;

    for (int k = 0 ; k < nbOfTests; k++)
    {

    }

    return true;
}


bool Workspace::sampleRobotBase(confPtr_t q_base, const Vector3d& WSPoint)
{

    /*confPtr_t q_cur = _Robot->getCurrentPos(); //store the current configuration
    const int plantformIndexDof = 6;

    vector< pair<double,Vector3d > > PossiblePoints = m_ReachableSpace->getBaseGridPoint();


    sort(PossiblePoints.begin(),PossiblePoints.end(),NaturalPointsCompObject);

    for (unsigned int i= 0; i < PossiblePoints.size(); i++)
    {
        Vector2d point;
        point[0] = PossiblePoints[i].second[0];
        point[1] = PossiblePoints[i].second[1];

        Vector2d HumanPos;

        HumanPos[0] = WSPoint[0];
        HumanPos[1] = WSPoint[1];

        Vector2d gazeDirect = HumanPos - point;

        (*q_base)[plantformIndexDof + 0]    = point[0];
        (*q_base)[plantformIndexDof + 1]    = point[1];
        (*q_base)[plantformIndexDof + 5] = atan2(gazeDirect.y(),gazeDirect.x());

        Vector3d CirclePoint;

        CirclePoint(0) = point[0];
        CirclePoint(1) = point[1];
        CirclePoint(2) = 0.30;
        PointsToDraw->push_back(CirclePoint);

        if(!testCol(q_base))
        {
            _Robot->setInitialPosition(*q_cur);
            _Robot->setGoTo(*q_base);
            return true;
        }
        _Robot->setAndUpdate(*q_cur);
    }

    _Robot->setAndUpdate(*q_cur);
        return false;*/


    confPtr_t q_cur = _Robot->getCurrentPos(); //store the current configuration

    unsigned int iterMax = 20;

    activateOnlyBaseCollision();
    int radI = 0;
    int radMaxI = 4;
    while (radI < radMaxI)
    {
        double radius;
        double rotationAngle;

        radI = radMaxI;
        rotationAngle = 4;
        radius = HRICS_outerRadius;

        for (unsigned int i=0; i<iterMax; i++)
        {
            //			radius = p3d_random(HRICS_innerRadius,HRICS_outerRadius);

            Vector2d gazePoint;
            if (radius == HRICS_innerRadius){gazePoint = m_VisibilitySpace->get2dPointAlongGaze(HRICS_innerRadius);}
            else
            {
                gazePoint = m_VisibilitySpace->get2dPointAlongGaze(p3d_random(HRICS_innerRadius, radius));
            }

            // Build the 2d transformation matrix
            // That rotates a point around the human gaze
            Transform2d t;

            Vector2d HumanPos;

            HumanPos[0] = WSPoint[0];
            HumanPos[1] = WSPoint[1];


            t.translation() = HumanPos;

            Rotation2Dd	rot( p3d_random(-M_PI/rotationAngle,
                                        M_PI/rotationAngle));
            //p3d_random(-M_PI/4, M_PI/4));


            t.linear() = rot.toRotationMatrix();
            t(2,0) = 0;
            t(2,1) = 0;
            t(2,2) = 1;

            Vector2d point = t * gazePoint;

            const int plantformIndexDof = 6;

            (*q_base)[plantformIndexDof + 0]    = point[0];
            (*q_base)[plantformIndexDof + 1]    = point[1];
            //(*q_base)[plantformIndexDof + 5]    = p3d_random(-M_PI, M_PI);;

            Vector2d gazeDirect = HumanPos - point;
            (*q_base)[plantformIndexDof + 5] = atan2(gazeDirect.y(),gazeDirect.x());
            //cout << "(*q_base)[plantformIndexDof + 5]" << 180*(*q_base)[plantformIndexDof + 5]/M_PI << endl;

            Vector3d CirclePoint;

            CirclePoint(0) = point[0];
            CirclePoint(1) = point[1];
            CirclePoint(2) = 0.30;

            PointsToDraw->push_back(CirclePoint);
            //cout << "Add Point to draw" << endl;

            q_base->setAsNotTested();

            if (!q_base->isInCollision())
            {
                deactivateOnlyBaseCollision();
                _Robot->setAndUpdate(*q_cur);
                return true;
            }
        }
    }
    deactivateOnlyBaseCollision();
    _Robot->setAndUpdate(*q_cur);
    return false;
}

void Workspace::drawCurrentOTP()
{
    double color_array[4];
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    color_array[0]= 1.0;
    color_array[1]= 0.0;
    color_array[2]= 0.0;
    color_array[3]= 1.0;
    g3d_set_color(Any,color_array);
    g3d_draw_solid_sphere(m_OTP[0], m_OTP[1], m_OTP[2], 0.03, 10);

    color_array[0]= 0.0;
    color_array[1]= 1.0;
    color_array[2]= 0.0;
    color_array[3]= 0.5;
    g3d_set_color(Any,color_array);
    g3d_draw_solid_sphere(m_OTP[0], m_OTP[1], m_OTP[2], 0.20, 10);

    glDisable(GL_BLEND);
}

struct target_info 
{
    unsigned int id;
    double* q;
};

double* Workspace::testTransferPointToTrajectory( const Vector3d& WSPoint, Move3D::Trajectory& traj , unsigned int& id)
{
    cout << " * Start testing for new OTP ***************" << endl;
    ChronoOn();

    int armId = 0;
    ArmManipulationData& armData = (*_Robot->getP3dRobotStruct()->armManipulationData)[armId];
    ManipulationConfigs manipConf(_Robot->getP3dRobotStruct());
    gpGrasp grasp;
    double confCost = -1;

    vector<double> target(6);
    target.at(0) = WSPoint(0);
    target.at(1) = WSPoint(1);
    target.at(2) = WSPoint(2);
    target.at(3) = 0;
    target.at(4) = 0;
    target.at(5) = P3D_HUGE;

    double* q = manipConf.getFreeHoldingConf(NULL, armId, grasp, armData.getCcCntrt()->Tatt, confCost, target, NULL);
    _Robot->activateCcConstraint();

    ChronoPrint("");

    vector< pair<double,unsigned int> > costOfPoint;

    for(unsigned int i=0;i<(unsigned int)traj.getNbOfPaths();i++)
    {
        confPtr_t q_target(new Configuration(_Robot,q));

        LocalPath path(traj.getLocalPath(i)->getEnd(),q_target);

        if (path.isValid())
        {
            costOfPoint.push_back( make_pair(path.cost(),i));
        }
    }

    if (!costOfPoint.empty())
    {
        sort(costOfPoint.begin(),costOfPoint.end());
        id = costOfPoint[0].second;
        ChronoPrint("");
        ChronoOff();
        setCurrentOTP(WSPoint);
        cout << " * End testing for new OTP (Succeed) ***********" << endl;
        return q;
    }
    else {
        p3d_destroy_config(_Robot->getP3dRobotStruct(),q);
    }

    ChronoPrint("");
    ChronoOff();
    cout << " * End testing for new OTP (Error) ***************" << endl;
    return NULL;
}

/**
double* Workspace::testTransferPointToTrajectory( const Vector3d& WSPoint, Move3D::Trajectory& traj , unsigned int& id)
{
  cout << " * Start testing for new OTP ***************" << endl;
  ChronoOn();
  
//  int armId = 0;
//  ArmManipulationData& armData = (*_Robot->getP3dRobotStruct()->armManipulationData)[armId];
//  ManipulationConfigs manipConf(_Robot->getP3dRobotStruct());
//  gpGrasp grasp;
//  double confCost = -1;
//  
//  vector<double> target(6);
//  
//  target.at(0) = WSPoint(0);
//  target.at(1) = WSPoint(1);
//  target.at(2) = WSPoint(2);
//  target.at(3) = P3D_HUGE;
//  target.at(4) = P3D_HUGE;
//  target.at(5) = P3D_HUGE;

  //double* q = manipConf.getFreeHoldingConf(NULL, armId, grasp, armData.getCcCntrt()->Tatt, confCost, target, NULL);
  
  // 1 - Create Agent
    //HRI_AGENTS * agents = hri_create_agents();
  HRI_AGENTS * agents = m_Agents;
  
  // 2 - Select Task
    HRI_GIK_TASK_TYPE task = GIK_RATREACH; // Left Arm GIK

  // 3 - Select Agent
  HRI_AGENT* agent = hri_get_one_agent_of_type(agents, HRI_JIDOKUKA);
  
  p3d_vector3 Tcoord;
  Tcoord[0] = WSPoint[0];
  Tcoord[1] = WSPoint[1];
  Tcoord[2] = WSPoint[2];
  
  double distance_tolerance = 0.10;
  
//  showConfig_2(q);
  double* q;
  
  if ( true )
  {
    vector< pair<double,target_info> > costOfPoint;
    
    for(unsigned int i=0;i<(unsigned int)traj.getNbPaths();i++)
    {
      p3d_set_and_update_this_robot_conf(agent->robotPt,
                                         traj.getLocalPath(i)->getEnd()->getConfigStruct());

      q = p3d_get_robot_config(agent->robotPt);
      
      bool succeed = hri_agent_single_task_manip_move(agent, task, &Tcoord, distance_tolerance, &q);
      
      if(succeed)
      {
        confPtr_t q_target(new Configuration(_Robot,q));
        
        p3d_update_virtual_object_config_for_arm_ik_constraint(_Robot->getP3dRobotStruct(), 0, q);
        
        //ManipulationUtils::checkConfigForCartesianMode(q,NULL);

        if (p3d_is_collision_free(_Robot->getP3dRobotStruct(),q))
        {
          showConfig_2(q);
          
          LocalPath path(traj.getLocalPath(i)->getEnd(),q_target);
          
          if (path.isValid())
          {
            pair<double,target_info> costOfTarget;
            target_info targ;
            
            targ.id = i;
            targ.q = p3d_copy_config(agent->robotPt,q);
            
            costOfPoint.push_back( make_pair(path.cost(),targ));
          }
        }
      }
      else {
        p3d_destroy_config(_Robot->getP3dRobotStruct(),q);
      }
    }
    
    if (!costOfPoint.empty())
    {
      //sort(costOfPoint.begin(),costOfPoint.end());
      pair<double,target_info> costOfTargetTmp;
      costOfTargetTmp.first = P3D_HUGE;
      
      for (unsigned int i=0; i<costOfPoint.size(); i++)
      {
        if (costOfPoint[i].first < costOfTargetTmp.first )
        {
          costOfTargetTmp = costOfPoint[i];
        }
      }
      
      id = costOfTargetTmp.second.id;
      ChronoPrint("");
      ChronoOff();
      cout << " * End testing for new OTP (Succed) ***********" << endl;
      return costOfTargetTmp.second.q;
    }
    else {
      p3d_destroy_config(_Robot->getP3dRobotStruct(),q);
    }
  }
  ChronoPrint("");
  ChronoOff();
  cout << " * End testing for new OTP (Error) ***************" << endl;
  return NULL;
}*/

/*bool Workspace::baseInSight(confPtr_t q_base)
 {
 
 const double min_theta= -M_PI/4;
 const double max_theta= M_PI/4;
 
 // Build the 2d transformation matrix
 Transform2d t;
 
 Vector2d HumanPos;
 
 HumanPos[0] = WSPoint[0];
 HumanPos[1] = WSPoint[1];
 
 t.translation() = HumanPos;
 
 Vector2d minGazePoint = m_Visibility->get2dPointAlongGaze(HRICS_innerRadius);
 
 Rotation2Dd rot(min_theta);
 t.linear() = rot.toRotationMatrix();
 
 Vector2d point1 = t * minGazePoint;
 
 Rotation2Dd rot(max_theta);
 t.linear() = rot.toRotationMatrix();
 
 Vector2d point2 = t * minGazePoint;
 
 Vector2d maxGazePoint = m_Visibility->get2dPointAlongGaze(HRICS_innerRadius);
 
 if( (*q_base)[plantformIndexDof + 0] > ;
 (*q_base)[plantformIndexDof + 1]    = point[1];
 
 
 }*/



bool Workspace::chooseBestTransferPoint(Vector3d& transfPoint, bool move, unsigned int threshold)
{
    setAndUpdateHumanArms(*mHumans[0]->getInitPos());
    _Robot->setAndUpdate(*_Robot->getInitPos());

    p3d_col_activate_rob_rob( _Robot->getP3dRobotStruct(), mHumans[0]->getP3dRobotStruct());

    confPtr_t q_base = _Robot->getCurrentPos();
    confPtr_t q_cur_robot  = _Robot->getCurrentPos();

    if ( m_ReachableSpace == NULL )
    {
        cout << "No ReachableSpace cost space initialized" << endl;
        return false;
    }

    vector< pair<double,Vector3d > > ReachablePoints = m_ReachableSpace->getReachableWSPoint();
    vector< pair<double,Vector3d > > costs;

    if (ReachablePoints.empty())
    {
        cout << "No OTP found" << endl;
        return false;
    }

    for (unsigned int i=0; i<ReachablePoints.size(); i++)
    {
        double KDist = ENV.getDouble(Env::Kdistance);
        double KVisi = ENV.getDouble(Env::Kvisibility);
        double KReac = ENV.getDouble(Env::Kreachable);

        Vector3d point   = ReachablePoints[i].second;

        double CostDist = m_DistanceSpace->getWorkspaceCost(point);
        double CostReach = ReachablePoints[i].first;
        double CostVisib = m_VisibilitySpace->getWorkspaceCost(point);

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

    sort(ReachablePoints.begin(),ReachablePoints.end(),NaturalPointsCompObject);
    sort(costs.begin(),costs.end(),NaturalPointsCompObject);


    for (unsigned int i=0; i<ReachablePoints.size(); i++)
    {
        transfPoint = ReachablePoints[i].second;

        if (m_ReachableSpace->computeIsReachableOnly(transfPoint,m_ReachableSpace->getGrid()->isReachableWithLA(transfPoint)) && i > threshold)
        {
            current_cost.first = ReachablePoints[i].first;
            current_cost.second = costs[i].second;
            break;
        }
    }

    confPtr_t q_cur_human = mHumans[0]->getCurrentPos();

    Vector3d WSPoint;

    WSPoint[0] = (*q_cur_human)[6];
    WSPoint[1] = (*q_cur_human)[7];
    WSPoint[2] = (*q_cur_human)[8];


    q_base->setAsNotTested();

    if (PointsToDraw == NULL){
        PointsToDraw = new PointCloud();
    }
    PointsToDraw->clear();

    initPR2RepoConf();

    if( sampleRobotBase(q_base,WSPoint) )
    {
        p3d_col_deactivate_rob_rob(_Robot->getP3dRobotStruct(), mHumans[0]->getP3dRobotStruct());
        setAndUpdateHumanArms(*mHumans[0]->getInitPos());
        if (!move)
        {
            _Robot->setAndUpdate(*q_cur_robot);
        }
        return true;
    }

    _Robot->setAndUpdate(*q_cur_robot);
    p3d_col_deactivate_rob_rob(_Robot->getP3dRobotStruct(), mHumans[0]->getP3dRobotStruct());
    setAndUpdateHumanArms(*mHumans[0]->getInitPos());
    cout << "No Point found" << endl;
    return false;
}

//changing this function
//bool Workspace::chooseBestTransferPoint(Vector3d& transfPoint, bool move)
//{
//	mHumans[0]->setAndUpdateHumanArms(*mHumans[0]->getInitPos());
//	_Robot->setAndUpdate(*_Robot->getInitPos());

//	p3d_col_activate_rob_rob( _Robot->getP3dRobotStruct(), mHumans[0]->getP3dRobotStruct());


//	if ( m_ReachableSpace == NULL )
//	{
//		cout << "No ReachableSpace cost space initialized" << endl;
//		return false;
//	}

//	vector< pair<double,Vector3d > > ReachablePoints = m_ReachableSpace->getReachableWSPoint();
//	vector< pair<double,Vector3d > > costs;

//	for (unsigned int i=0; i<ReachablePoints.size(); i++)
//	{
//		double KDist = ENV.getDouble(Env::Kdistance);
//		double KVisi = ENV.getDouble(Env::Kvisibility);
//		double KReac = ENV.getDouble(Env::Kreachable);

//		Vector3d point   = ReachablePoints[i].second;

//        double CostDist = m_DistanceSpace->getWorkspaceCost(point);
//        double CostReach = ReachablePoints[i].first;
//        double CostVisib = m_VisibilitySpace->getCost(point);

//        Vector3d localCosts;
//        localCosts[0] = CostDist;
//        localCosts[1] = CostReach;
//        localCosts[2] = CostVisib;


//        ReachablePoints[i].first = (KDist*CostDist + KVisi*CostVisib + KReac*CostReach )/ 3;

//        pair<double,Vector3d > p;
//        p.first = ReachablePoints[i].first;
//        p.second = localCosts;

//        costs.push_back( p );

//    }

//	sort(ReachablePoints.begin(),ReachablePoints.end(),NaturalPointsCompObject);
//	sort(costs.begin(),costs.end(),NaturalPointsCompObject);

//	vector<Vector3d> points;

//	for (unsigned int i=0; i<ReachablePoints.size(); i++)
//	{
//		points.push_back(ReachablePoints[i].second);
////                cout << "Cost = " << ReachablePoints[i].first << endl;
//	}

//	if( points.empty() )
//	{
//		cout << "No WSPoint to compute transfer point" << endl;
//		return false;
//	}

//	confPtr_t q_base = _Robot->getCurrentPos();

//	if ( /*baseInSight(q_base)*/ false )
//	{
//		if( transPFromBaseConf(q_base,points) )
//		{
//			_Robot->setAndUpdate(*q_base);
//			//cout << "0 : from base config" << endl;
//			return true;
//		}
//	}

//	confPtr_t q_cur_robot  = _Robot->getCurrentPos();



//	for (unsigned int i=0; i<ReachablePoints.size(); i++)
//	{
//		transfPoint = ReachablePoints[i].second;

//		if (m_ReachableSpace->computeIsReachableOnly(transfPoint,m_ReachableSpace->getGrid()->isReachableWithLA(transfPoint)))
//		{
//			current_cost.first = ReachablePoints[i].first;
//			current_cost.second = costs[i].second;
//			break;
//		}
//	}

////	Vector3d WSPoint;
////	WSPoint = transfPoint;

//	confPtr_t q_cur_human = mHumans[0]->getCurrentPos();

//	Vector3d WSPoint;

//	WSPoint[0] = (*q_cur_human)[6];
//	WSPoint[1] = (*q_cur_human)[7];
//	WSPoint[2] = (*q_cur_human)[8];


//	q_base->setAsNotTested();

//	if (PointsToDraw == NULL){
//			PointsToDraw = new PointCloud();
//	}
//	PointsToDraw->clear();

//	initPR2RepoConf();


//	//find a configuration for the whole robot (mobile base + arm):
//	for(unsigned int i=0; i<100; i++)
//	{
//		if( sampleRobotBase(q_base,WSPoint) )
//		{
//			cout << "Valid Base config at " << i << endl;
//			if ( transPFromBaseConf(q_base,points) )
//			{
//				cout << "Configuration at iteration " << i << " found!!!" << endl;

//                p3d_col_deactivate_rob_rob(_Robot->getP3dRobotStruct(), mHumans[0]->getP3dRobotStruct());
//                mHumans[0]->setAndUpdateHumanArms(*mHumans[0]->getInitPos());
//                if (!move)
//                {
//                    _Robot->setAndUpdate(*q_cur_robot);
//                }
//                return true;
//            }
//        }

//		_Robot->setAndUpdate(*q_cur_robot);

//	}

//	_Robot->setAndUpdate(*q_cur_robot);
//	p3d_col_deactivate_rob_rob(_Robot->getP3dRobotStruct(), mHumans[0]->getP3dRobotStruct());
//	mHumans[0]->setAndUpdateHumanArms(*mHumans[0]->getInitPos());
//	cout << "No Point found" << endl;
//	return false;
//}

bool Workspace::computeBestTransferPoint(Vector3d& transfPoint)
{	
    setAndUpdateHumanArms(*mHumans[0]->getInitPos());

    if ( m_ReachableSpace == NULL )
    {
        cout << "No ReachableSpace cost space initialized" << endl;
        return false;
    }

    vector< pair<double,Vector3d > > ReachablePoints = m_ReachableSpace->getReachableWSPoint();
    vector< pair<double,Vector3d > > costs;

    for (unsigned int i=0; i<ReachablePoints.size(); i++)
    {
        double KDist = ENV.getDouble(Env::Kdistance);
        double KVisi = ENV.getDouble(Env::Kvisibility);
        double KReac = ENV.getDouble(Env::Kreachable);

        Vector3d point   = ReachablePoints[i].second;

        double CostDist = m_DistanceSpace->getWorkspaceCost(point);
        double CostReach = ReachablePoints[i].first;
        double CostVisib = m_VisibilitySpace->getWorkspaceCost(point);

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

    sort(ReachablePoints.begin(),ReachablePoints.end(),NaturalPointsCompObject);
    sort(costs.begin(),costs.end(),NaturalPointsCompObject);

    if( ReachablePoints.empty() )
    {
        cout << "No WSPoint to compute transfer point" << endl;
        return false;
    }

    transfPoint = ReachablePoints[0].second;
    current_cost.first = ReachablePoints[0].first;
    current_cost.second = costs[0].second;

    return true;
}

bool Workspace::computeBestFeasableTransferPoint(Vector3d& transfPoint)
{
    setAndUpdateHumanArms(*mHumans[0]->getCurrentPos());

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
        double CostVisib = m_VisibilitySpace->getWorkspaceCost(point);

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

    sort(ReachablePoints.begin(),ReachablePoints.end(),NaturalPointsCompObject);
    sort(costs.begin(),costs.end(),NaturalPointsCompObject);

    for (unsigned int i=0; i<ReachablePoints.size(); i++)
    {
        transfPoint = ReachablePoints[i].second;

        if (m_ReachableSpace->computeIsReachableOnly(transfPoint,m_ReachableSpace->getGrid()->isReachableWithLA(transfPoint)))
        {
            current_cost.first = ReachablePoints[i].first;
            current_cost.second = costs[i].second;
            return true;
        }
    }

    setAndUpdateHumanArms(*mHumans[0]->getInitPos());

    return false;
}



bool Workspace::ComputeTheObjectTransfertPoint(bool Move, int type, Vector3d& WSPoint, int threshold)
{
    //   ENV.setDouble( Env::Kdistance,   5 );
    //   ENV.setDouble( Env::Kvisibility, 15 );
    //   ENV.setDouble( Env::Kreachable,  65 );

    bool hasComputed = false;

    if (ENV.getBool(Env::isCostSpace)){

        //                cout << "OtpWidget::computeTheOtp" << endl;
        ENV.setBool(Env::HRIComputeOTP,true);

        HRICS::Natural* reachSpace = HRICS_MotionPL->getReachability();
        reachSpace->setRobotToConfortPosture();

        if ( ENV.getBool(Env::HRIComputeOTP) ){

            Move3D::BaseGrid* tmp_grid;
            tmp_grid = API_activeGrid;
            API_activeGrid = reachSpace->getGrid();

            if (type == 0){
                hasComputed = computeBestTransferPoint(WSPoint);
            }else if(type==1){
                hasComputed = computeBestFeasableTransferPoint(WSPoint);
            }else if(type==2){
                hasComputed = chooseBestTransferPoint(WSPoint, Move, threshold);
            }


            if( hasComputed ){

                Vector3d inGridPoint( reachSpace->getGrid()->getTranformedToRobotFrame(WSPoint) );
                bool leftHand = dynamic_cast<NaturalCell*>(reachSpace->getGrid()->getCell(inGridPoint))->isReachableWithLA();
                if (dynamic_cast<NaturalCell*>(reachSpace->getGrid()->getCell(inGridPoint))->isReachableWithRA())
                {
                    leftHand = false;
                }

                if (Move){
                    reachSpace->computeIsReachableAndMove(WSPoint, leftHand);

                }else{
                    reachSpace->computeIsReachableOnly(WSPoint, reachSpace->getGrid()->isReachableWithLA(WSPoint));
                }
                current_WSPoint = WSPoint;
                //        computePR2GIK(Move);
                //        cout << WSPoint << endl;
            }else{
                current_WSPoint << 0.0 ,0.0, 0.0;
            }
            API_activeGrid = tmp_grid;
        }

        ENV.setBool(Env::HRIComputeOTP,false);

        //                cout << "The OTP has been computed succesfully"<< endl ;

    }else{
        cout << "Cost Space not initialized" << endl;
    }
    return hasComputed;
}

Eigen::Vector3d Workspace::computeOTPFromHandPose( bool rightHand )
{
    Joint* j;

    if (rightHand)
    {
        j = mHumans[0]->getJoint("rPalm");
    }
    else {
        j = mHumans[0]->getJoint("lPalm");
    }

    Transform3d t( j->getMatrixPos() );

    Vector3d offSet;
    offSet(0) = 0.00;
    offSet(1) = 0.20;
    offSet(2) = 0.00;

    return t*offSet;
}

void Workspace::initPR2RepoConf()
{
    confPtr_t q_cur = _Robot->getCurrentPos();

    int IndexObjectDof = m_ReachableSpace->getRobot()->getJoint("Pelvis")->getIndexOfFirstDof();

    configPt q;
    q = p3d_alloc_config(_Robot->getP3dRobotStruct());
    q[0] = 0;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
    q[4] = 0;
    q[5] = 0;
    q[6] = (*q_cur)[IndexObjectDof+0];
    q[7] = (*q_cur)[IndexObjectDof+1];
    q[8] = (*q_cur)[IndexObjectDof+2];
    q[9] = (*q_cur)[IndexObjectDof+3];
    q[10] = (*q_cur)[IndexObjectDof+4];
    q[11] = (*q_cur)[IndexObjectDof+5];
    q[12] = 0;
    q[13] = 0;
    q[14] = 0;
    q[15] = 0;
    q[16] = 0;
    q[17] = 54.00*M_PI/180;
    q[18] = -85.00*M_PI/180;
    q[19] = -111.00*M_PI/180;
    q[20] = -66.00*M_PI/180;
    q[21] = -116.00*M_PI/180;
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
    q[37] = 0;
    q[38] = 0;
    q[39] = 0;
    q[40] = 0;
    q[41] = 0;
    q[42] = 0;


    confPtr_t m_q = confPtr_t(
                new Configuration(_Robot,p3d_copy_config(_Robot->getP3dRobotStruct(),q)));
    _Robot->setInitPos(*m_q);
    _Robot->setAndUpdate( *m_q );


}


void Workspace::initPR2GiveConf()
{
    cout << "Workspace::initPR2GiveConf()" << endl;

    confPtr_t q_cur = _Robot->getCurrentPos();

    configPt q;
    q = p3d_alloc_config(_Robot->getP3dRobotStruct());


    for (unsigned int i = 0; i < 48; i++)
    {
        q[i] = (*q_cur)[i];
    }

    q[13] = 0;
    q[14] = 0;
    q[15] = 0;
    q[16] = 0;
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



    confPtr_t m_q = confPtr_t(
                new Configuration(_Robot,p3d_copy_config(_Robot->getP3dRobotStruct(),q)));
    _Robot->setAndUpdate( *m_q );
    _Robot->setInitPos(*m_q );
}


void Workspace::initPR2AndHumanTest()
{
    cout << "Workspace::initPR2AndHumanTest()" << endl;


    confPtr_t q_cur_human = m_ReachableSpace->getRobot()->getCurrentPos();
    int firstIndexOfHumanDof = m_ReachableSpace->getRobot()->getJoint("Pelvis")->getIndexOfFirstDof();

    configPt q_h;
    q_h = p3d_alloc_config(m_ReachableSpace->getRobot()->getP3dRobotStruct());
    for (int i = 0; i < m_ReachableSpace->getRobot()->getP3dRobotStruct()->nb_dof; i++)
    {
        q_h[i] = (*q_cur_human)[i];
    }


    q_h[firstIndexOfHumanDof + 0] = (*q_cur_human)[firstIndexOfHumanDof+0];
    q_h[firstIndexOfHumanDof + 1] = (*q_cur_human)[firstIndexOfHumanDof+1];
    q_h[firstIndexOfHumanDof + 5] = 0;

    confPtr_t m_q_human = confPtr_t(
                new Configuration(m_ReachableSpace->getRobot(),p3d_copy_config(m_ReachableSpace->getRobot()->getP3dRobotStruct(),q_h)));

    m_ReachableSpace->getRobot()->setInitPos(*m_q_human);
    m_ReachableSpace->getRobot()->setAndUpdate( *m_q_human );


    confPtr_t q_cur = _Robot->getCurrentPos();
    configPt q;
    q = p3d_alloc_config(_Robot->getP3dRobotStruct());

    for (int i = 0; i < _Robot->getP3dRobotStruct()->nb_dof; i++)
    {
        q[i] = (*q_cur)[i];
    }

    int firstIndexOfRobotDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;

    q[firstIndexOfRobotDof + 0] = (*q_cur_human)[firstIndexOfHumanDof+0] + 1.5;
    q[firstIndexOfRobotDof + 1] = (*q_cur_human)[firstIndexOfHumanDof+1];

    //it should be 180 or -180 but it won't work unless that.
    q[firstIndexOfRobotDof + 5] = 179.0*M_PI;

    confPtr_t m_q = confPtr_t(
                new Configuration(_Robot,p3d_copy_config(_Robot->getP3dRobotStruct(),q)));

    _Robot->setInitPos(*m_q);
    _Robot->setAndUpdate( *m_q );

    initPR2GiveConf();

}


void Workspace::computePR2GIK(bool move)
{
    //    initPR2GiveConf();
    cout << "Workspace::computePR2GIK()" << endl;

    int armId = 0;
    ArmManipulationData& armData = (*_Robot->getP3dRobotStruct()->armManipulationData)[armId];
    ManipulationConfigs manipConf(_Robot->getP3dRobotStruct());
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
        confPtr_t m_q = confPtr_t(
                    new Configuration(_Robot,p3d_copy_config(_Robot->getP3dRobotStruct(),q)));
        _Robot->setAndUpdate( *m_q );
    }
    else
    {
        initPR2GiveConf();
    }




    /*confPtr_t q_cur = _Robot->getCurrentPos();
    configPt q;
    q = p3d_alloc_config(_Robot->getP3dRobotStruct());

    for (unsigned int i = 0; i < _Robot->getP3dRobotStruct()->nb_dof ; i++)
    {
        q[i] = (*q_cur)[i];

    }
    int virtualObjectDOF = _Robot->getJoint("virtual_object_right")->getIndexOfFirstDof();
    q[virtualObjectDOF + 0] = current_WSPoint[0];
    q[virtualObjectDOF + 1] = current_WSPoint[1];
    q[virtualObjectDOF + 2] = current_WSPoint[2];

    double dist = 99.0;
    double distThreshold = 0.15;
    unsigned int i = 0;
    unsigned int loopThreshold = 50;

    confPtr_t m_q = confPtr_t(
                                          new Configuration(_Robot,p3d_copy_config(_Robot->getP3dRobotStruct(),q)));
    _Robot->setAndUpdate( *m_q );

    confPtr_t m_q_tmp;

    while (dist > distThreshold && i < loopThreshold)
    {
        _Robot->setAndUpdate( *m_q );
        q[virtualObjectDOF + 3] = p3d_random(-M_PI,M_PI);
        q[virtualObjectDOF + 4] = p3d_random(-M_PI,M_PI);
        q[virtualObjectDOF + 5] = p3d_random(-M_PI,M_PI);
        Joint* j14 = _Robot->getJoint("fingerJointGripper_0");

        m_q_tmp = confPtr_t(new Configuration(_Robot,p3d_copy_config(_Robot->getP3dRobotStruct(),q)));
        _Robot->setAndUpdate( *m_q_tmp );

        dist = std::sqrt( pow(current_WSPoint[0] - j14->getVectorPos()[0], 2) + pow(current_WSPoint[1] - j14->getVectorPos()[1], 2) + pow(current_WSPoint[2] - j14->getVectorPos()[2], 2));

        if (testCol(m_q_tmp))
        {
            dist = 99.0;
        }
        i++;
    }
    if (dist > distThreshold)
    {
        cout << "Transfer configuration : Not found!!!"<< endl;
        initPR2GiveConf();
    }
    else
    {
        // Set the robot in motion planning configuration
        _Robot->setGoTo(*m_q_tmp);
        _Robot->setInitialPosition(*q_cur);
        cout << "Transfer configuration : have been set for motion planning!!!"<< endl;
    }*/
}


void Workspace::ChangeRobotPos(double value)
{

    int firstIndexOfHumanDof = m_ReachableSpace->getRobot()->getJoint("Pelvis")->getIndexOfFirstDof();
    confPtr_t q_cur_human = m_ReachableSpace->getRobot()->getCurrentPos();

    int firstIndexOfDof = dynamic_cast<p3d_jnt*>(_Robot->getP3dRobotStruct()->baseJnt)->user_dof_equiv_nbr;
    cout << "Workspace::ChangeRobotPos()" << endl;
    cout << value << endl;

    confPtr_t q_cur = _Robot->getCurrentPos();
    configPt q;
    q = p3d_alloc_config(_Robot->getP3dRobotStruct());

    for (int i = 0; i < _Robot->getP3dRobotStruct()->nb_dof; i++)
    {
        q[i] = (*q_cur)[i];
    }


    q[firstIndexOfDof] = (*q_cur_human)[firstIndexOfHumanDof + 0] + value;

    confPtr_t m_q = confPtr_t(
                new Configuration(_Robot,p3d_copy_config(_Robot->getP3dRobotStruct(),q)));
    //    _Robot->setInitialPosition(*m_q);
    _Robot->setAndUpdate( *m_q );

    computePR2GIK(true);
}

/**
 * set and update Human Arms
 */
bool Workspace::setAndUpdateHumanArms( Move3D::Configuration& q )
{
    Robot* robot = q.getRobot();

    for(int i=2; i<=21; i++) // Just Arms
    {
        p3d_jnt* jntPt = static_cast<p3d_rob*>( robot->getP3dRobotStruct() )->joints[i];
        for(int j=0; j<jntPt->dof_equiv_nbr; j++)
        {
            p3d_jnt_set_dof(jntPt, j, q[jntPt->index_dof+j]);
        }
    }

    return p3d_update_this_robot_pos(static_cast<p3d_rob*>( robot->getP3dRobotStruct() ));
}

//void Workspace::getCostForPr2FromAgentGrids()
//{
//  
//}

/*
 *  HRICS_CostSpace.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 04/12/09.
 *  Copyright 2009 LAAS/CNRS. All rights reserved.
 *
 */
#ifdef QT_LIBRARY
#include <QMap>
#endif

#include "HRICS_costspace.hpp"
#include "Grid/HRICS_Grid.hpp"
#include "Grid/HRICS_GridState.hpp"
#include "RRT/HRICS_rrt.hpp"
#include "RRT/HRICS_rrtExpansion.hpp"
#include "../Diffusion/RRT-Variants/Transition-RRT.hpp"
//#include "../../qtWindow/cppToQt.hpp"
#include "API/Trajectory/smoothing.hpp"

//#include <Eigen/Array>

#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Planner-pkg.h"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

#ifdef HRI_PLANNER
HRICS::HriSpaceCost* hriSpace = NULL;
#endif

HRICS::Distance*		HRICS_activeDist = NULL;
HRICS::Visibility*	HRICS_activeVisi = NULL;
HRICS::Natural*			HRICS_activeNatu = NULL;
HRICS::Natural*			HRICS_activeReac = NULL;

HRICS::HumanAwareMotionPlanner*		HRICS_MotionPL = NULL;

API::ThreeDCell*	BiasedCell3D = NULL;
API::TwoDCell*		BiasedCell2D = NULL;

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
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
    if((*_Robot->getRobotStruct()->armManipulationData).size() >= 1 )
    {
        p3d_jnt* FF_Joint = (*_Robot->getRobotStruct()->armManipulationData)[0].getManipulationJnt();
        ENV.setInt(Env::akinJntId,FF_Joint->num);
        mIndexObjectDof = FF_Joint->index_dof;
    }
    else
    {
            cout << "Error : Manipulation Data uninitialized" << endl;
    }
  }
#else
        cout << "Warning: Lihght Planner not compiled" << endl;
#endif
	
#ifdef P3D_PLANNER
	p3d_del_graph(XYZ_GRAPH);
#endif
	
	XYZ_GRAPH = NULL;
	
	if(_Robot)
	{
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
        //#endif\


}

void Workspace::initDistance()
{
	m_DistanceSpace = new Distance(_Robot,mHumans);
	
	if (_Robot) 
	{
		cout << "Robot " << _Robot->getName() << endl;
		cout << "Robot get struct " << _Robot->getRobotStruct() << endl;
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

Eigen::Vector3d Workspace::getVisball()
{
  Eigen::Vector3d pos;
  Robot* visball = global_Project->getActiveScene()->getRobotByName("VISBALL");
  
  if (visball) {
    shared_ptr<Configuration> q = visball->getCurrentPos();
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
	
	shared_ptr<Configuration> config = _Robot->getInitialPosition();
	
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
	
	config = _Robot->getGoTo();
	
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
	
	shared_ptr<Configuration> config = _Robot->getCurrentPos();
	
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
			m3DPath.clear();
			m3DCellPath.clear();
			mPathExist = false;
			return;
		}
		
		for (unsigned int i=0;i<path.size();i++)
		{
			API::ThreeDCell* cell = dynamic_cast<State*>(path[i])->getCell();
			m3DPath.push_back( cell->getCenter() );
			m3DCellPath.push_back( cell );
		}
	}
	else
	{
		API::AStar* search = new API::AStar(goal);
		vector<API::State*> path = search->solve(start);
		
		if(path.size() == 0 )
		{
			m3DPath.clear();
			m3DCellPath.clear();
			mPathExist = false;
			return;
		}
		
		for (int i=path.size()-1;i>=0;i--)
		{
			API::ThreeDCell* cell = dynamic_cast<State*>(path[i])->getCell();
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
	if( mPathExist)
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
	
	shared_ptr<Configuration> config = _Robot->getCurrentPos();
	
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
	
	shared_ptr<Configuration> config = _Robot->getCurrentPos();
	
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
		activateCcCntrts(_Robot->getRobotStruct(),-1,true);
	}
	else
	{
		deactivateCcCntrts(_Robot->getRobotStruct(),-1);//true);
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
	p3d_col_deactivate_rob(_Robot->getRobotStruct());
	p3d_col_deactivate_rob_env(_Robot->getRobotStruct());
	p3d_col_deactivate_rob_rob(_Robot->getRobotStruct(),mHumans[0]->getRobotStruct());
	
	p3d_col_activate_obj_env(_Robot->getJoint(1)->getJointStruct()->o);
	p3d_col_activate_obj_rob(_Robot->getJoint(1)->getJointStruct()->o,mHumans[0]->getRobotStruct());
}

void Workspace::deactivateOnlyBaseCollision()
{
	p3d_col_deactivate_rob(_Robot->getRobotStruct());
	p3d_col_deactivate_rob_env(_Robot->getRobotStruct());
	p3d_col_deactivate_rob_rob(_Robot->getRobotStruct(),mHumans[0]->getRobotStruct());
	
	p3d_col_activate_rob(_Robot->getRobotStruct());
	p3d_col_activate_rob_env(_Robot->getRobotStruct());
	p3d_col_activate_rob_rob(_Robot->getRobotStruct(),mHumans[0]->getRobotStruct());
}

bool Workspace::transPFromBaseConf(shared_ptr<Configuration> q_base, vector< Vector3d > points )
{
	shared_ptr<Configuration>  q_actual = _Robot->getCurrentPos(); //pour memoriser la configuration courante du robot
	
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

const double HRICS_innerRadius = 1.2;
const double HRICS_outerRadius = 1.5;

bool Workspace::sampleRobotBase(shared_ptr<Configuration> q_base, const Vector3d& WSPoint)
{
	shared_ptr<Configuration> q_cur = _Robot->getCurrentPos(); //store the current configuration
	
	const unsigned int iterMax = 20;
	
	activateOnlyBaseCollision();
	
	for (unsigned int i=0; i<iterMax; i++) 
	{
		double radius= p3d_random(HRICS_innerRadius,HRICS_outerRadius);
		
		Vector2d gazePoint = m_VisibilitySpace->get2dPointAlongGaze(radius);
		
		// Build the 2d transformation matrix
		// That rotates a point around the human gaze
		Transform2d t;
		
                Vector2d HumanPos;
		
		HumanPos[0] = WSPoint[0];
		HumanPos[1] = WSPoint[1];
		

		t.translation() = HumanPos;
		
                Rotation2Dd rot(p3d_random(-M_PI/4, M_PI/4));
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

double* Workspace::testTransferPointToTrajectory( const Vector3d& WSPoint, API::Trajectory& traj , unsigned int& id)
{
  cout << " * Start testing for new OTP ***************" << endl;
  ChronoOn();
  
  int armId = 0;
  ArmManipulationData& armData = (*_Robot->getRobotStruct()->armManipulationData)[armId];
  ManipulationConfigs manipConf(_Robot->getRobotStruct());
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
  
  for(unsigned int i=0;i<(unsigned int)traj.getNbPaths();i++)
  {
    shared_ptr<Configuration> q_target(new Configuration(_Robot,q));
    
    LocalPath path(traj.getLocalPathPtrAt(i)->getEnd(),q_target);
    
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
    p3d_destroy_config(_Robot->getRobotStruct(),q);
  }
  
  ChronoPrint("");
  ChronoOff();
  cout << " * End testing for new OTP (Error) ***************" << endl;
  return NULL;
}

/**
double* Workspace::testTransferPointToTrajectory( const Vector3d& WSPoint, API::Trajectory& traj , unsigned int& id)
{
  cout << " * Start testing for new OTP ***************" << endl;
  ChronoOn();
  
//  int armId = 0;
//  ArmManipulationData& armData = (*_Robot->getRobotStruct()->armManipulationData)[armId];
//  ManipulationConfigs manipConf(_Robot->getRobotStruct());
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
                                         traj.getLocalPathPtrAt(i)->getEnd()->getConfigStruct());
      
      q = p3d_get_robot_config(agent->robotPt);
      
      bool succeed = hri_agent_single_task_manip_move(agent, task, &Tcoord, distance_tolerance, &q);
      
      if(succeed)
      {
        shared_ptr<Configuration> q_target(new Configuration(_Robot,q));
        
        p3d_update_virtual_object_config_for_arm_ik_constraint(_Robot->getRobotStruct(), 0, q);
        
        //ManipulationUtils::checkConfigForCartesianMode(q,NULL);
    
        if (p3d_is_collision_free(_Robot->getRobotStruct(),q)) 
        {
          showConfig_2(q);
          
          LocalPath path(traj.getLocalPathPtrAt(i)->getEnd(),q_target);
          
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
        p3d_destroy_config(_Robot->getRobotStruct(),q);
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
      p3d_destroy_config(_Robot->getRobotStruct(),q);
    }
  }
  ChronoPrint("");
  ChronoOff();
  cout << " * End testing for new OTP (Error) ***************" << endl;
  return NULL;
}*/
      
/*bool Workspace::baseInSight(shared_ptr<Configuration> q_base)
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

bool Workspace::chooseBestTransferPoint(Vector3d& transfPoint)
{	


        p3d_col_activate_rob_rob( _Robot->getRobotStruct(), mHumans[0]->getRobotStruct());
	
	mHumans[0]->setAndUpdateHumanArms(*mHumans[0]->getInitialPosition());
	
	if ( m_ReachableSpace == NULL )
	{
		cout << "No ReachableSpace cost space initialized" << endl;
		return false;
	}
	
	vector< pair<double,Vector3d > > ReachablePoints = m_ReachableSpace->getReachableWSPoint();
	
	for (unsigned int i=0; i<ReachablePoints.size(); i++)
        {
                double KDist = ENV.getDouble(Env::Kdistance);
                double KVisi = ENV.getDouble(Env::Kvisibility);
                double KReac = ENV.getDouble(Env::Kreachable);

                Vector3d point   = ReachablePoints[i].second;

                double CostDist = m_DistanceSpace->getWorkspaceCost(point);
                double CostReach = ReachablePoints[i].first;
                double CostVisib = m_VisibilitySpace->getCost(point);


                ReachablePoints[i].first = (KDist*CostDist + KVisi*CostVisib + KReac*CostReach )/ 3;
        }
	
	sort(ReachablePoints.begin(),ReachablePoints.end(),NaturalPointsCompObject);
	




	vector<Vector3d> points;
	
	for (unsigned int i=0; i<ReachablePoints.size(); i++)
	{
		points.push_back(ReachablePoints[i].second);
//                cout << "Cost = " << ReachablePoints[i].first << endl;
	}
	
	if( points.empty() )
	{
		cout << "No WSPoint to compute transfer point" << endl;
		return false;
	}
	
	shared_ptr<Configuration> q_base = _Robot->getCurrentPos();
	
	if ( /*baseInSight(q_base)*/ false ) 
	{
		if( transPFromBaseConf(q_base,points) )
		{
			_Robot->setAndUpdate(*q_base);
			//cout << "0 : from base config" << endl;
			return true;
		}
	}
	
	shared_ptr<Configuration> q_cur_robot  = _Robot->getCurrentPos();
	shared_ptr<Configuration> q_cur_human = mHumans[0]->getCurrentPos();
	
	Vector3d WSPoint;
	
        return m_ReachableSpace->computeIsReachableForRobot(_Robot, WSPoint, true);

//        // Human pos (x,y,z)
//	WSPoint[0] = (*q_cur_human)[6];
//	WSPoint[1] = (*q_cur_human)[7];
//	WSPoint[2] = (*q_cur_human)[8];
	
//	q_base->setAsNotTested();
	
//        if (PointsToDraw == NULL){
//                PointsToDraw = new ThreeDPoints();
//        }
//        PointsToDraw->clear();
	
//	//find a configuration for the whole robot (mobile base + arm):
//	for(unsigned int i=0; i<100; i++)
//	{
//		if( sampleRobotBase(q_base,WSPoint) )
//		{
//			cout << "Valid Base config at " << i << endl;
//			if ( transPFromBaseConf(q_base,points) )
//			{
//				cout << "Configuration at iteration " << i << " found!!!" << endl;
				
//                                transfPoint[0] = (*q_base)[mIndexObjectDof+0];
//                                transfPoint[1] = (*q_base)[mIndexObjectDof+1];
//                                transfPoint[2] = (*q_base)[mIndexObjectDof+2];
				
////				_Robot->setAndUpdate(*q_base);
//                                p3d_col_deactivate_rob_rob(_Robot->getRobotStruct(), mHumans[0]->getRobotStruct());
//                                _Robot->setAndUpdate(*q_cur_robot);
//				return true;
//			}
//		}
		
//		_Robot->setAndUpdate(*q_cur_robot);
		
//	}
	
//	_Robot->setAndUpdate(*q_cur_robot);
//	p3d_col_deactivate_rob_rob(_Robot->getRobotStruct(), mHumans[0]->getRobotStruct());
//	mHumans[0]->setAndUpdateHumanArms(*mHumans[0]->getInitialPosition());
//	cout << "No Point found" << endl;
//	return false;
}

bool Workspace::computeBestTransferPoint(Vector3d& transfPoint)
{	
	mHumans[0]->setAndUpdateHumanArms(*mHumans[0]->getInitialPosition());
	
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
	mHumans[0]->setAndUpdateHumanArms(*mHumans[0]->getInitialPosition());
	
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
	
	mHumans[0]->setAndUpdateHumanArms(*mHumans[0]->getInitialPosition());
	

	return false;
}



bool Workspace::ComputeTheObjectTransfertPoint(bool Move, int type, Vector3d& WSPoint)
{
   ENV.setDouble( Env::Kdistance,   5 );
   ENV.setDouble( Env::Kvisibility, 15 );
   ENV.setDouble( Env::Kreachable,  65 );

  bool hasComputed = false;
  
  if (ENV.getBool(Env::isCostSpace)){
    
    //                cout << "OtpWidget::computeTheOtp" << endl;
    ENV.setBool(Env::HRIComputeOTP,true);
    
    if ( ENV.getBool(Env::HRIComputeOTP) ){
      
      if (type == 0){
        hasComputed = computeBestTransferPoint(WSPoint);
      }else if(type==1){
        hasComputed = computeBestFeasableTransferPoint(WSPoint);
      }else if(type==2){
        hasComputed = chooseBestTransferPoint(WSPoint);
        
      }
      
      if( hasComputed ){
        HRICS::Natural* reachSpace = HRICS_MotionPL->getReachability();
        if (Move){
          reachSpace->computeIsReachableAndMove(WSPoint, reachSpace->getGrid()->isReachableWithLA(WSPoint));

        }else{
          reachSpace->computeIsReachableOnly(WSPoint, reachSpace->getGrid()->isReachableWithLA(WSPoint));
        }
        current_WSPoint = WSPoint;
//        cout << WSPoint << endl;
      }else{
        current_WSPoint << 0.0 ,0.0, 0.0;
        }
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


/*
 *  HRICS_costspace.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/06/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "HRICS_costspace.hpp"
#include "API/planningAPI.hpp"

#include "P3d-pkg.h"

const bool DebugCostFunctions=false;

using namespace std;
using namespace tr1;
using namespace HRICS;

// Cost function type
enum CostSpaceFunction 
{
	TaskSpace,
	WorkSpace,
	ConfSpace,
	NatuSpace
} myCostFunction;

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Main function for the HRI cost space
double HRICS_getConfigCost(Configuration& Conf)
{	
	Robot* rob = Conf.getRobot();
	shared_ptr<Configuration> q_Saved = rob->getCurrentPos();
  
	rob->setAndUpdate(Conf);
	
	if ( ENV.getBool(Env::HRIPlannerTS) )
	{
		myCostFunction = TaskSpace;
	}
	else if ( ENV.getBool(Env::HRIPlannerWS) ) 
	{
		myCostFunction = WorkSpace;
	}
	else if ( ENV.getBool(Env::HRIPlannerCS) )
	{
		myCostFunction = ConfSpace;
	}
	else if ( HRICS_activeNatu != NULL )
	{
		myCostFunction = NatuSpace;
	}
	
	double Cost=0.0;
	
	switch( myCostFunction )
	{
		case TaskSpace: // Akin's functions -----------------------------
#ifdef HRI_PLANNER
			Cost = hriSpace->switchCost();
#else
			cout << "Warning :: p3d_GetConfigCost ::  HRI_PLANNER not compiled nor linked\n" << endl;
#endif
			break;
			
		case WorkSpace: // Workspace Class ------------------------------
			
			Cost = 0.0;
			
			if( ENV.getBool(Env::HRIPathDistance) && dynamic_cast<Workspace*>(HRICS_MotionPL)->get3DPath().size() > 0 )
			{
				Cost = dynamic_cast<Workspace*>(HRICS_MotionPL)->distanceToEntirePath();
			}
			else
			{
				if( ( ENV.getInt(Env::hriCostType) == HRICS_Distance || ENV.getInt(Env::hriCostType) == HRICS_Combine )
					&& ( ENV.getDouble(Env::Kdistance) != 0.0 ) )
				{
					Cost += ENV.getDouble(Env::Kdistance)*(HRICS_MotionPL->getDistance()->getDistToZones()[0]);
					
					if ( DebugCostFunctions )
					{
						cout << "Distance Cost = " << Cost << endl;
					}
				}
				
				if( ( ENV.getInt(Env::hriCostType) == HRICS_Visibility || ENV.getInt(Env::hriCostType) == HRICS_Combine )
				   && ( ENV.getDouble(Env::Kvisibility) != 0.0 ) )
				{
					int object = dynamic_cast<Workspace*>(HRICS_MotionPL)->getIndexObjectDof();
					
					Eigen::Vector3d WSPoint;
					
					WSPoint[0] = Conf[object+0];
					WSPoint[1] = Conf[object+1];
					WSPoint[2] = Conf[object+2];
					
					double VisibCost = ENV.getDouble(Env::Kvisibility)*(HRICS_MotionPL->getVisibility()->getCost(WSPoint));
					Cost += VisibCost;
					
					if ( DebugCostFunctions )
					{
						cout << "Visibility Cost = " << VisibCost << endl;
					}
				}
				
				/*if( ENV.getInt(Env::hriCostType) == HRICS_Naturality || ENV.getInt(Env::hriCostType) == HRICS_Combine )
				 {
				 double NaturCost = ENV.getDouble(Env::Knatural)*(HRICS_MotionPL->getNaturality()->getConfigCost());
				 Cost += NaturCost;
				 cout << "Naturality Cost = " << NaturCost << endl;
				 }*/
				
				if( ( ENV.getInt(Env::hriCostType) == HRICS_Reachability || ENV.getInt(Env::hriCostType) == HRICS_Combine )
				   && ( ENV.getDouble(Env::Kreachable) != 0.0 ) )
				{
					int object = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getIndexObjectDof();
					
					Eigen::Vector3d WSPoint;
					
					WSPoint[0] = Conf[object+0];
					WSPoint[1] = Conf[object+1];
					WSPoint[2] = Conf[object+2];
					
					double ReachCost = ENV.getDouble(Env::Kreachable)*(HRICS_MotionPL->getReachability()->getCostInGrid(WSPoint));
					Cost += ReachCost;
					
					if (ReachCost<0) {
						p3d_set_robot_display_mode(rob->getRobotStruct(),P3D_ROB_BLUE_DISPLAY);
					}
					else {
						p3d_set_robot_display_mode(rob->getRobotStruct(),P3D_ROB_DEFAULT_DISPLAY);
					}
					
					if ( DebugCostFunctions )
					{
						cout << "Reach Cost = " << ReachCost << endl;
					}
				}
			}
			break;
			
			
		case ConfSpace: // ConfigSpace Class ------------------------------
			Cost = dynamic_cast<ConfigSpace*>(HRICS_MotionPL)->getConfigCost();
			break;
			
			
		case NatuSpace: // Natural Class ----------------------------------
			Cost = HRICS_activeNatu->getConfigCost();
			break;
			
		default:
			cout << "Warning :: p3d_GetConfigCost :: No Cost While costspace and HRI are on!!!" << endl;
			break;
	}
	
	//                Cost = hri_zones.getHriDistCost(robotPt, true);
  
	rob->setAndUpdate(*q_Saved);
	
	if ( DebugCostFunctions )
	{
		cout << "Cost = " << Cost << endl;
	}
             
	return Cost;
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
void HRICS_loadGrid(std::string docname)
{
	ENV.setBool(Env::drawGrid,false);
	
	HRICS_activeNatu  = new HRICS::Natural;
	
	HRICS::NaturalGrid* myGrid = new HRICS::NaturalGrid;
	myGrid->setNaturalCostSpace(HRICS_activeNatu);
	
	bool reading_OK=false;
	
	for (int i=0; (i<5)&&(!reading_OK) ; i++) 
	{
		cout << "Reading grid at : " << docname << endl;
		reading_OK = myGrid->loadFromXmlFile(docname);
	}
  
	API_activeGrid = myGrid;
	
	if( HRICS_MotionPL != NULL )
	{
		if( HRICS_activeNatu->IsHuman() )
		{
			cout << "Set Reachability space" << endl;
			HRICS_MotionPL->setReachability(HRICS_activeNatu);
		}
		else 
		{
			cout << "Set Natural space" << endl;
			HRICS_MotionPL->setNatural(HRICS_activeNatu);
		}
	}
	
	ENV.setBool(Env::drawGrid,true);
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
void HRICS_init()
{
  Robot* Human = global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");
  
  shared_ptr<Configuration> q = Human->getCurrentPos();
  
	HRICS_MotionPL = new HRICS::Workspace;
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initGrid();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initDistance();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initVisibility();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initNatural();
	
	ENV.setBool(Env::HRIPlannerWS,true);
	ENV.setDouble(Env::zone_size,0.5);
	HRICS_activeDist = HRICS_MotionPL->getDistance();
	
	API_activeGrid = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid();
	
	ENV.setBool(Env::enableHri,true);
	ENV.setBool(Env::isCostSpace,true);
	
	if( ENV.getBool(Env::HRIAutoLoadGrid) )
	{
		string fileName("/Users/jmainpri/workspace/Move3D-core/statFiles/Cost3DGrids/Cost3DGrid.grid");
		
		// Reads the grid from XML and sets it ti the HRICS_MotionPL
		HRICS_loadGrid(fileName);
		HRICS_activeNatu->setGrid(dynamic_cast<HRICS::NaturalGrid*>(API_activeGrid));
		ENV.setBool(Env::drawGrid,false);
	}
	
	ENV.setInt(Env::hriCostType,HRICS_Combine);
	
	cout << "new HRI Workspace" << endl;
  
  //Human->setAndUpdate( *q );
}

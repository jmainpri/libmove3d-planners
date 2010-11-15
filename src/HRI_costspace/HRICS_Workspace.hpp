#ifndef HRICS_PLANNER_H
#define HRICS_PLANNER_H

/*
 *  HRICS_CostSpace.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 04/12/09.
 *  Copyright 2009 mainprice@laas.fr All rights reserved.
 *
 */

#if defined( CXX_PLANNER )
#include "API/planningAPI.hpp"
#include "planner.hpp"
#include "Diffusion/RRT.hpp"
#endif

#if defined( MOVE3D_CORE )
#include "API/planningAPI.hpp"
#include "planner/planner.hpp"
#include "planner/Diffusion/RRT.hpp"
#endif

#include "HRICS_Distance.hpp"
#include "HRICS_Visibility.hpp"
#include "HRICS_Natural.hpp"
#include "Grid/HRICS_Grid.hpp"
#include "Grid/HRICS_GridState.hpp"

/**
 @defgroup HRICS Hri Cost space
 */

/**
 @ingroup HRICS
 */
namespace HRICS
{
	
	/*! 
	 * Base class for the HRICS motion planners
	 */
	class HumanAwareMotionPlanner : public Planner 
	{
		
	public:
		HumanAwareMotionPlanner() : 
		Planner(),
		m_DistanceSpace(NULL),
		m_VisibilitySpace(NULL),
		m_ReachableSpace(NULL),
		m_NaturalSpace(NULL) { }
		
		HumanAwareMotionPlanner(Robot* rob, Graph* graph) : 
		Planner(rob,graph),
		m_DistanceSpace(NULL),
		m_VisibilitySpace(NULL),
		m_ReachableSpace(NULL),
		m_NaturalSpace(NULL) { }
		
		~HumanAwareMotionPlanner() 
		{
			delete m_DistanceSpace;
			delete m_VisibilitySpace;
		}
		
		Distance*	getDistance() { return m_DistanceSpace; }
		Visibility* getVisibility() { return m_VisibilitySpace; }
		Natural*	getNaturality() { return m_NaturalSpace; }
		Natural*	getReachability() { return m_ReachableSpace; }
		
		
		void setDistance(Distance* dist) 
		{ 
			if( m_DistanceSpace )
			{ 
				delete m_DistanceSpace;
			}
			m_DistanceSpace = dist;
		}
		
		void setVisibility(Visibility* visib) 
		{ 
			if( m_VisibilitySpace )
			{ 
				delete m_VisibilitySpace;
			}
			m_VisibilitySpace = visib;
		}
		
		void setNatural(Natural* nat) 
		{ 
			if( m_NaturalSpace )
			{ 
				delete m_NaturalSpace;
			}
			m_NaturalSpace = nat;
		}
		
		void setReachability(Natural* nat) 
		{ 
			if( m_ReachableSpace )
			{ 
				delete m_ReachableSpace;
			}
			m_ReachableSpace = nat;
		}
		
		unsigned int run() { return 0; }
		
	protected:
		/**
		 * Distance and Visibility 
		 * cost spaces
		 */
		Distance*			m_DistanceSpace;
		Visibility*		m_VisibilitySpace;
		Natural*			m_ReachableSpace;
		Natural*			m_NaturalSpace;
		
	};
	
	/*! 
	 * Workspace Motion Planners
	 */
	class Workspace : public HumanAwareMotionPlanner
	{
		
    public :
		
		/**
		 * Constructors & Destructors
		 */
		Workspace();
		Workspace(Robot* rob, Graph* graph);
		~Workspace();
		
		/**
		 * Init Associated Objects
		 */
		void initGrid();
		void deleteGrid();
		void initDistance();
		void initVisibility();
		void initReachable();
		void initNatural();
		
		/**
		 * Get Robot and Human
		 */
		Robot* getHuman(){ return mHumans[0]; }
		
		/**
		 * Computes A* in Grid
		 */
		bool computeAStarIn3DGrid();
		double pathCost();
		void draw3dPath();
		
		/**
		 * Distance to 3D path
		 */
		double distanceToEntirePath();
		double distanceToCellPath();
		
		/**
		 * Getters
		 */
		Grid* getGrid() { return m3DGrid; }
		std::vector<Eigen::Vector3d> get3DPath() { return m3DPath; }
		std::vector<API::ThreeDCell*> getCellPath() { return m3DCellPath; }
		int getIndexObjectDof() 
		{ 
			if(mIndexObjectDof==0)
			{	
				std::cout << "Workspace::Warning::mIndexObjectDof == 0" << std::endl;
			}
			return mIndexObjectDof; 
		}
		
		/**
		 * Run RRT
		 */
		bool initHriRRT();
		
		/**
		 * Choose best transfer point from
		 * the natural costspace associated to the human
		 */
		void deactivateOnlyBaseCollision();
		void activateOnlyBaseCollision();
		bool sampleRobotBase(std::tr1::shared_ptr<Configuration> q_base, const Eigen::Vector3d& WSPoint);
		bool transPFromBaseConf(std::tr1::shared_ptr<Configuration> q_base, std::vector< Eigen::Vector3d > points );
		bool baseInSight(std::tr1::shared_ptr<Configuration> q_base);
		
		bool chooseBestTransferPoint(Eigen::Vector3d& transfPoint);
		bool computeBestTransferPoint(Eigen::Vector3d& transfPoint);
		bool computeBestFeasableTransferPoint(Eigen::Vector3d& transfPoint);
		
	private:
		
		void solveAStar(State* start,State* goal);
		
		/** 
		 * Members
		 */
		
		/**
		 * Humans in the scene
		 */
		std::vector<Robot*>     mHumans;
		
		int mIndexObjectDof;
		
		/**
		 * 3d grid to compute a Workspace path
		 */
		Grid*                   m3DGrid;
		
		
		/**
		 * 3d path internals
		 */
		bool mPathExist;
		std::vector<Eigen::Vector3d>   m3DPath;
		std::vector<API::ThreeDCell*> m3DCellPath;
	};
}

#endif

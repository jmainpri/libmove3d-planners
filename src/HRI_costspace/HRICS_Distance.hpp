#ifndef HRIDISTANCE_H
#define HRIDISTANCE_H

/*
 *  Distance.h
 *  
 *
 *  Created by Jim Mainprice on 05/12/09.
 *  Copyright 2009 mainprice@gmail.com All rights reserved.
 *
 */
#if defined( CXX_PLANNER ) ||  defined( OOMOVE3D_CORE )
#include "API/planningAPI.hpp"
#endif

/**
 @ingroup HRICS
 */
namespace HRICS
{
	class Distance 
	{
		//        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
	public:
		Distance();
		Distance(Robot* rob, std::vector<Robot*> humans);
		
		~Distance();
		
		void drawZones();
		void parseHumans();
		double getDistance();
		void offSetPrim(p3d_poly* poly,double offset);
		
		void activateSafetyZonesMode();
		void activateNormalMode();
		
		double computeCost(double distance);
		double getWorkspaceCost(const Eigen::Vector3d& WSPoint);
		
		std::vector<double> getDistToZones();
		std::vector<double> getVectorJim() {return vect_jim; }
		
		void setVector( std::vector<double> toDrawVector ) { vect_jim = toDrawVector; }
		
		double computeBBDist(p3d_vector3 robot, p3d_vector3 human);
		
		bool computeCylinderZone(Eigen::Vector3d &p1, Eigen::Vector3d& p2);
		
		double pointToLineSegmentDistance(const Eigen::Vector3d& p, 
																			const Eigen::Vector3d& p1, 
																			const Eigen::Vector3d& p2, 
																			Eigen::Vector3d& closestPoint);
		
		double computeBoundingBalls(const Eigen::Vector3d& WSPoint, p3d_vector3 robot, p3d_vector3 human);
		
		Eigen::Vector3d getColsestPointToHuman() { return mClosestPointToHuman; }
		
		void setSafeRadius(double radius) { _SafeRadius = radius; }
		
		
	private:
		Robot* m_Robot;
		std::vector<Robot*> _Humans;
		std::vector< std::vector<int> > _SafetyZonesBodyId;
		std::vector<double> _PenetrationDist;
		std::vector<double> vect_jim;
		double _SafeOffset;
		double _SafeRadius;
		
		enum Computation 
		{
			Balls,
			Boxes,
			Full,
		} m_Method;
		
		Eigen::Vector3d mClosestPointToHuman;
	};
}

#endif

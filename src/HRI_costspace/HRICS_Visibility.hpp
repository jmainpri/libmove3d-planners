#ifndef HRICS_VISIBILITY_H
#define HRICS_VISIBILITY_H

/*
 *  HRICS_Visbility.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 30/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

/**
 @ingroup HRICS
 */
namespace HRICS
{
    class Visibility 
	{
	public:
		/**
		 * Sets the Pointer to Human
		 */
		Visibility(Robot* R);
		
		/**
		 * Gets the Visibiliy cost (distance to gaze direction) of a 
		 * Workspace point
		 */
		double getCost(const Eigen::Vector3d& WSPoint);
		
		/**
		 * Copy paste from Akins
		 */
		double akinVisibilityCost(const Eigen::Vector3d& WSPoint);
		
		/**
		 * Returns the gaze associated to the visibility
		 * cost
		 */
		std::vector<double> getGaze();
		
		/**
		 * Distance on gaze
		 */
		Eigen::Vector2d get2dPointAlongGaze(double dist);
		
	private:
		/**
		 * Human Kinematics
		 */
		Robot* m_Human;
		
		/**
		 * Previous cost
		 */
		double m_Cost;
		
		/**
		 * Returns the gaze of the
		 * actual human position
		 */
		std::vector<double> m_VectGaze;
		
		/**
		 * Flag to compute 
		 * the gaze vector
		 */
		bool m_DrawGaze;
	};
}

#endif

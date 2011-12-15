/*
 * costOptimization.hpp
 *
 *  Created on: Jun 25, 2009
 *      Author: jmainpri
 */

#ifndef COST_OPTIMIZATION_HPP_
#define COST_OPTIMIZATION_HPP_

#include "API/ConfigSpace/configuration.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"

/**
 * @ingroup Trajectory
 * @brief Genera Cost Optimization of a trajectory
 */
namespace API 
{
	class CostOptimization : public Smoothing 
	{
	public:
		
		/**
		 * Constructors and Destructors of the class
		 */
		CostOptimization();
		CostOptimization(const Trajectory& T);
		CostOptimization(Robot* R,p3d_traj* t);
		
		~CostOptimization();
    
    /**
		 * Prints debug information
		 */
		void printDebugInfo();
		
		
		/**
		 * Returns true if the new trajectory is in collision
		 */
		bool deformInCollision() { return m_inCollision; }
		
		/**
		 * Set the cheat 
		 */
		void setCheat() { m_cheat = true; }
    
    /**
		 * Get the minimal cost
		 */
		double getMinCost() { return m_mincost; }
    
    /**
		 * One loop of the deformation strategy
		 * @param step is the distance between 2 configurations
		 */
		bool oneLoopDeform();
		
		/**
		 * One loop of the deformation strategy with recomputing 
		 * of the trajectory portion cost as it might change
		 * @param step is the distance between 2 configurations
		 */
		bool oneLoopDeformRecompute();
		
		/**
		 * 
		 */
		void runDeformation( int nbIteration , int idRun=0 );
    

  protected:
		
		/**
		 * Cheat for Justin
		 */
		std::tr1::shared_ptr<Configuration> cheat();
		
		/**
		 * Stops at the last descending configuration
		 * on the cost map
		 */
		double getLastDescendingConfParam(LocalPath& directionPath);
		
		/**
		 * Expand the configuration to a ne
		 */
		std::tr1::shared_ptr<Configuration> perturbCurrent(std::tr1::shared_ptr<Configuration> qCurrPt,
                                                           std::tr1::shared_ptr<Configuration> qRandPt);

		
		/**
		 * Create new trajectories to show in debug mode
		 * also calls the g3d_draw function to plot in the OpenGl display
		 */
		void debugShowTraj(double lPrev,double lNext,std::tr1::shared_ptr<Configuration> qNew , int color);
    
    /**
		 * Returns 3 random configurations along the trajtectory
		 * @param the step between the 3 configuration
		 * @return vector of configuration
		 * @return 
		 */
		std::vector< std::tr1::shared_ptr<Configuration> > get3RandSuccesConfAlongTraj(
                                                                                   double& prevDistPt,
                                                                                   double& randDistPt,
                                                                                   double& nextDistPt,
                                                                                   double step);
		
		/**
		 * Returns the 3 configurations that are the closest to the input configuration
		 * @param 
		 */
		std::vector< std::tr1::shared_ptr<Configuration> > getClosestConfOnTraj(
                                                                            double& prevDistPt,
                                                                            double& randDistPt,
                                                                            double& nextDistPt,
                                                                            std::tr1::shared_ptr<Configuration> ptrConf,
                                                                            double step);
		
		
	private:
		
		bool									m_cheat;
		double								m_mincost;
		unsigned int					m_nbErrors;
		std::vector<double>		m_Errors;
		bool									m_DeformBiased;
		bool									m_inCollision;
		bool									m_descent;
		
		//	bool oneLoopShortCut(double step);
	};
}

#endif /* COST_OPTIMIZATION_HPP_ */

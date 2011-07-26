/*
 * smoothing.hpp
 *
 *  Created on: Jun 26, 2009
 *      Author: jmainpri
 */

#ifndef BASEOPTIMIZATION_HPP_
#define BASEOPTIMIZATION_HPP_

#undef Trajectory

#include "API/Trajectory/trajectory.hpp"

/**
 * @ingroup Trajectory
 * @brief Basic optimization of a trajectory
 */
namespace API 
{
	class Smoothing : public Trajectory 
	{
	public:
		/**
		 * Class constructors and destructors
		 */
		Smoothing();
		Smoothing(const Trajectory& T);
		Smoothing(Robot* R,traj* t);
		
		~Smoothing();
		
		/**
		 * stops the trajectory optimization
		 */
		bool checkStopConditions( unsigned int iter );
			
		/**
		 * gets randomly two random configurations
		 */
		std::vector< std::tr1::shared_ptr<Configuration> > get2RandomConf( double step,
																																			double& secondDist,
																																			double& firstDist);
		
		/**
		 * gets randomly n configurations on the traj between firstDist and secondDist
		 */
		std::vector< std::tr1::shared_ptr<Configuration> > getConfAtStepAlongTraj( double step , 
																																							double firstDist, 
																																							double secondDist );
		/**
		 * One loop of the random shortcut
		 */
		bool oneLoopShortCut();
		
		/**
		 * One loop of the random shortcut 
		 * with recomputation of the trajectory cost
		 */
		bool oneLoopShortCutRecompute();
		
		/**
		 * PatialShortCut : intependently shortcut each DoFs
		 */
		bool partialShortcut();
		
		/**
		 * Compute the subportion of with entire outer localpaths
		 */
		bool isLowerCostLargePortion( double lFirst , double lSecond , std::vector<LocalPath*>& paths);
		
		/**
		 * Interpolates a Configuration
		 */
		double interpolateOneDoF( unsigned int ithActiveDoF , double init , double goal , double alpha );
		
		/**
		 * Change the Ith Active Dof on Conf
		 */
		void changeIthActiveDofValueOnConf( Configuration& q,
																				unsigned int ithActiveDoF, 
																				double value );
		
		/**
		 * Go through all nodes in a deterministic manner
		 */
		void removeRedundantNodes();
		
		/**
		 * Get the time spent in optimization
		 */
		double getTime() { return m_time; }
		
		/**
		 * Set Context name
		 */
		void setContextName(std::string name) { m_ContextName = name; } 
		
		/**
		 * Show the trajectory while being deformed
		 */
		void debugShowTraj(double lPrev,double lNext);
		
		/**
		 * Set the sorted indexes by cost
		 */
		void setSortedIndex();
		
		/**
		 * Get a parameter on the trajectory
		 * which is biased to the high cost parts of the trajectory
		 */
		double getBiasedParamOnTraj();
		
		/**
		 * Compute the resolution of a path
		 *  @param resolution
		 */
		double closestResolutionToStep(double length,double step);
		
		/**
		 * Compute the gain of the last n succueded iterations
		 * @param last n taken into account iterations
		 */
		double gainOfLastIterations( unsigned int n );
		
		/**
		 * Save the optimization to a file
		 */
		void saveOptimToFile( std::string str );
		
		/**
		 * Runs the shortcut method for a certain number of iterations
		 * @param iterations
		 */
		void runShortCut(int nbIteration, int idRun = 0);
		
	protected:
		
		std::string						m_ContextName;
		std::vector<double>		m_Selected;
		int										m_nbBiased;
		int										m_nbReallyBiased;
		
		double								m_time;
		
		double								m_step;
		
		std::vector<double>		m_OptimCost;
		std::vector<double>		m_GainCost;
		
		bool									m_IterationSucceded;
		std::vector<double>		m_GainOfIterations;
		unsigned int					m_MaxNumberOfIterations;
		
	private:
		std::vector<uint>			m_IdSorted;
		bool									m_ShortCutBiased;
	};
}

#endif /* SHORTCUT_HPP_ */

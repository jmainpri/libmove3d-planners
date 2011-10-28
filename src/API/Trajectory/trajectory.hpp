/*
 * trajectory.hpp
 *
 *  Created on: Jun 17, 2009
 *      Author: jmainpri
 */

#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/localpath.hpp"

#ifndef _TRAJ_H
struct traj;
#endif
/**
 * @ingroup CPP_API
 * @defgroup Trajectory
 */

/**
 * @ingroup Trajectory
 * @brief Trajectory witch is a vector of local paths
 */
namespace API 
{
	class Trajectory 
	{
	public:
		
		//---------------------------------------------------------
		// Constructors
		Trajectory();
		Trajectory(Robot* R);
		Trajectory(const Trajectory& T);
		Trajectory(Robot* R,traj* t);
		Trajectory(std::vector< std::tr1::shared_ptr<Configuration> >& C);
		~Trajectory();
		
		Trajectory& operator= (const Trajectory& f);
		
		//---------------------------------------------------------
		// Operations
		void copyPaths( std::vector<LocalPath*>& vect );
		
		std::vector< std::tr1::shared_ptr<Configuration> > getTowConfigurationAtParam(
                                                                                      double param1, double param2 , uint& lp1, uint& lp2);
		
		std::vector<LocalPath*> extractSubPortion(
                                                  double param1,
                                                  double param2,
                                                  unsigned int& first,
                                                  unsigned int& last);
		
		Trajectory extractSubTrajectory(unsigned int id_start, unsigned int id_end);
        
		Trajectory extractSubTrajectory(
                                        double param1,
                                        double param2);
        
		bool concat(const Trajectory& traj);
		
		bool replacePortion(
                            unsigned int id1,
                            unsigned int id2,
                            std::vector<LocalPath*> paths, bool freeMemory = true );
		
		bool replacePortion(
                            double param1,
                            double param2,
                            std::vector<LocalPath*> paths , bool freeMemory = true );
		
		void cutTrajInSmallLP(unsigned int nLP);
		uint cutPortionInSmallLP(std::vector<LocalPath*>& portion, uint nLP);
		
		void push_back(std::tr1::shared_ptr<Configuration> q);
		
		//---------------------------------------------------------
		// Cost
		std::vector< std::pair<double,double > > getCostProfile();
		double computeSubPortionIntergralCost(std::vector<LocalPath*>& portion);
		double computeSubPortionCost(std::vector<LocalPath*>& portion);
		double ReComputeSubPortionCost(std::vector<LocalPath*>& portion);
		double computeSubPortionCostVisib( std::vector<LocalPath*>& portion );
		double costOfPortion(double param1,double param2);
		double extractCostPortion(double param1, double param2);
		double cost();
		double costNoRecompute();
		double costDeltaAlongTraj();
		std::vector<double> getCostAlongTrajectory(int nbSample);
		void resetCostComputed();
		
		
		//---------------------------------------------------------
		// Basic
    bool isEmpty();
    
		std::tr1::shared_ptr<Configuration> configAtParam(double param);
		
		std::vector< std::tr1::shared_ptr<Configuration> > getNConfAtParam(double delta);
		std::vector< std::tr1::shared_ptr<Configuration> > getVectorOfConfiguration();
		
		uint					getIdOfPathAt(double param);
		LocalPath* 		getLocalPathPtrAt(unsigned int id) const;
		int						getNbOfPaths() const;
    int						getNbOfViaPoints() const;
		
		bool isValid();
		
		void 	updateRange();
		double computeSubPortionRange(std::vector<LocalPath*> portion);
		
		bool 	replaceP3dTraj();
		p3d_traj* 	replaceP3dTraj(p3d_traj* trajPt);

		p3d_traj* 	replaceHumanP3dTraj(Robot*rob, p3d_traj* trajPt);
		
		void draw(int nbKeyFrame);
		void print();
		
		int meanCollTest();
		
		//---------------------------------------------------------
		// B-Spline
		bool makeBSplineTrajectory();
		
		
		//---------------------------------------------------------
		// Getters & Setters
		
		void setColor(int col) {mColor=col;}
		
		unsigned int getHighestCostId() const
		{
			return HighestCostId;
		}
		
		Robot* getRobot() const
		{
			return m_Robot;
		}
		
		double getRangeMax() const
		{
			return range_param;
		}
		
		std::tr1::shared_ptr<Configuration> getBegin() const
		{
			return m_Source;
		}
		
		std::tr1::shared_ptr<Configuration> getEnd() const
		{
			return m_Target;
		}
		
		std::vector<LocalPath*> getCourbe() const
		{	
			return m_Courbe;
		}
		
		//---------------------------------------------------------
		// Members
	protected:
		/* Robot */
		Robot*										m_Robot;
		
		unsigned int							HighestCostId;
		bool											isHighestCostIdSet;
		
	private:
		std::vector<LocalPath*> 	m_Courbe;
		
		/* name of trajectory */
		std::string name;
		
		/* Name of the file */
		std::string file;
		
		/* Number of localpath */
		uint		nloc;
		
		int mColor;
		
		/* Maximum range of parameter along the trajectory (length)*/
		double    	range_param;
		
		/* Start and Goal (should never change) */
		std::tr1::shared_ptr<Configuration> m_Source;
		std::tr1::shared_ptr<Configuration> m_Target;
	};
}

//#if defined( QT_LIBRARY ) 
#include <vector>
namespace API { class Trajectory; }
extern std::vector<API::Trajectory> trajToDraw;
void draw_traj_debug();
//#endif

#endif /* TRAJECTORY_HPP_ */

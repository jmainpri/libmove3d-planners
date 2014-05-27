/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
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

#include <iostream>

#ifndef _TRAJ_H
struct traj;
#endif

/**
 * @ingroup Trajectory
 * @brief Trajectory witch is a vector of local paths
 */
namespace Move3D
{
/**
 * @ingroup CPP_API
 * @defgroup Trajectory
 */

struct TrajectoryStatistics
{
    double length;
    double min;
    double max;
    double sum;
    double average;
    double integral;
    double mecha_work;
    bool is_valid;

    void print()
    {
        std::cout << "--- stats on traj ---" << std::endl;
        std::cout << " length = " << length << std::endl;
        std::cout << " min = " << min << std::endl;
        std::cout << " max = " << max << std::endl;
        std::cout << " average = " << average << std::endl;
        std::cout << " integral = " << integral << std::endl;
        std::cout << " mecha_work = " << mecha_work << std::endl;
        std::cout << "---------------------" << std::endl;
    }
};

class Trajectory
{
public:
    //---------------------------------------------------------
    // Constructors
    Trajectory();
    Trajectory(Robot* R);
    Trajectory(Robot* R,traj* t);
    Trajectory(std::vector<confPtr_t>& C);
    Trajectory(const Trajectory& T);
    ~Trajectory();

    Trajectory& operator= (const Trajectory& t);

    bool operator == ( const Trajectory& t ) const;
    bool operator != ( const Trajectory& t ) const;
    
    //---------------------------------------------------------
    // Operations
    void copyPaths( std::vector<LocalPath*>& vect );

    std::vector< MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> > getTowConfigurationAtParam( double param1, double param2, uint& lp1, uint& lp2 ) const;

    std::pair<bool,std::vector<LocalPath*> > extractSubPortion(double param1,double param2,unsigned int& first,unsigned int& last, bool check_for_coll = true) const;
    Trajectory extractSubTrajectoryOfLocalPaths(unsigned int id_start, unsigned int id_end) const;
    Trajectory extractSubTrajectory(double param1,double param2, bool check_for_coll = true) const;
    Trajectory extractReverseTrajectory() const;

    bool concat(const Trajectory& traj);

    bool replacePortionOfLocalPaths( unsigned int id1, unsigned int id2, std::vector<LocalPath*> paths, bool freeMemory = true );
    bool replacePortion( double param1, double param2, std::vector<LocalPath*> paths, bool freeMemory = true );
    
    bool replaceBegin(double param, const std::vector<LocalPath*>& paths );
    bool replaceEnd(double param, const std::vector<LocalPath*>& paths );

    bool cutTrajInSmallLP(unsigned int nLP);
    bool cutTrajInSmallLPSimple(unsigned int nLP);
    uint cutPortionInSmallLP(std::vector<LocalPath*>& portion, uint nLP);

    void push_back(confPtr_t q);
    bool push_back(MOVE3D_PTR_NAMESPACE::shared_ptr<LocalPath> path);

    //---------------------------------------------------------
    // Cost

    bool operator < (const Trajectory& traj) const
    {
        return (cost() < traj.cost());
    }

    double collisionCost() const;
    
    std::vector< std::pair<double,double > > getCostProfile();
    double computeSubPortionIntergralCost(const std::vector<LocalPath*>& portion);
    double computeSubPortionCost(const std::vector<LocalPath*>& portion) const;
    std::pair<double,double> computeSubPortionMinAndMaxCost(std::vector<LocalPath*>& portion);
    double reComputeSubPortionCost(std::vector<LocalPath*>& portion, int& nb_cost_tests);
    double computeSubPortionCostVisib( std::vector<LocalPath*>& portion );
    double costOfPortion(double param1,double param2);
    double extractCostPortion(double param1, double param2);

    double cost() const;
    double costNoRecompute();
    double costRecomputed();
    double costStatistics( TrajectoryStatistics& stat );
    double costDeltaAlongTraj();
    double costNPoints(const int n_points);
    double costSum();
    
    std::vector<double> getCostAlongTrajectory(int nbSample);
    void resetCostComputed();

    //---------------------------------------------------------
    // Basic
    bool isEmpty();
    
    void clear();
    
    confPtr_t configAtParam(double param, unsigned int* id_localpath=NULL) const;

    std::vector<confPtr_t> getNConfAtParam(double delta) const;
    std::vector<confPtr_t> getVectorOfConfiguration() const;

    uint            getLocalPathId(double param) const;
    LocalPath*      getLocalPath(unsigned int id) const;

    confPtr_t operator[] ( const int &i ) const;
    int size() const { return m_Courbe.size(); }
    int	getNbOfPaths() const { return m_Courbe.size(); }
    int	getNbOfViaPoints() const;

    bool isValid() const;
    void resetIsValid();

    void 	updateRange();
    double  computeSubPortionRange(const std::vector<LocalPath*>& portion) const;

    bool    replaceP3dTraj() const;
    traj* 	replaceP3dTraj(traj* trajPt) const;
    traj* 	replaceHumanP3dTraj(Robot*rob, traj* trajPt);

    Eigen::MatrixXd getEigenMatrix(int startIndex=0, int endIndex=0) const;
    Eigen::MatrixXd getEigenMatrix(const std::vector<int>& incides) const;

    void printAllLocalpathCost();
    void draw(int nbKeyFrame = 0);
    void print() const;

    int meanCollTest();

    //---------------------------------------------------------
    // B-Spline
    bool makeBSplineTrajectory();


    //---------------------------------------------------------
    // Getters & Setters

    void setUseContinuousColors(bool use_continuous_color=true) { m_use_continuous_color=use_continuous_color; }
    void setColor(double col) { m_Color=col; }

    unsigned int getHighestCostId() const
    {
        return m_HighestCostId;
    }

    Robot* getRobot() const
    {
        return m_Robot;
    }
    
    confPtr_t getBegin() const
    {
        return m_Source;
    }

    confPtr_t getEnd() const
    {
        return m_Target;
    }

    const std::vector<LocalPath*>& getCourbe() const
    {
        return m_Courbe;
    }
    
    double getRangeMax() const {
        return computeSubPortionRange(m_Courbe);
    }

    long int Id() { return m_id; }

    //---------------------------------------------------------
    // Members
protected:
    /* Robot */
    Robot*						m_Robot;

    unsigned int				m_HighestCostId;
    bool						m_isHighestCostIdSet;

private:
    std::vector<LocalPath*> 	m_Courbe;

    /* name of trajectory */
    std::string                 m_name;

    /* trajectory id */
    long int                    m_id;

    /* Name of the file */
    std::string                 m_file;

    bool                        m_use_continuous_color;
    double                      m_Color;

    /* Start and Goal (should never change) */
    confPtr_t                   m_Source;
    confPtr_t                   m_Target;
};
}

//#if defined( QT_LIBRARY ) 
#include <vector>
namespace Move3D { class Trajectory; }
extern std::vector<Move3D::Trajectory> global_trajToDraw;
void draw_traj_debug();
//#endif

#endif /* TRAJECTORY_HPP_ */

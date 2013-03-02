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

struct TrajectoryStatistics 
{
    double length;
    double max;
    double sum;
    double average;
    double integral;
    double mecha_work;
    bool is_valid;
};

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

    std::vector< MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> > getTowConfigurationAtParam( double param1, double param2,
                                                                                               uint& lp1, uint& lp2 );

    std::pair<bool,std::vector<LocalPath*> > extractSubPortion(double param1,double param2,unsigned int& first,unsigned int& last, bool check_for_coll = true);
    Trajectory extractSubTrajectoryOfLocalPaths(unsigned int id_start, unsigned int id_end);
    Trajectory extractSubTrajectory(double param1,double param2, bool check_for_coll = true);

    bool concat(const Trajectory& traj);

    bool replacePortionOfLocalPaths(unsigned int id1,unsigned int id2,std::vector<LocalPath*> paths, bool freeMemory = true );
    bool replacePortion(double param1,double param2,std::vector<LocalPath*> paths , bool freeMemory = true );
    
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
    double computeSubPortionIntergralCost(std::vector<LocalPath*>& portion);
    double computeSubPortionCost(const std::vector<LocalPath*>& portion) const;
    double computeSubPortionMaxCost(std::vector<LocalPath*>& portion);
    double ReComputeSubPortionCost(std::vector<LocalPath*>& portion, int& nb_cost_tests);
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
    
    confPtr_t operator [] ( const int &i ) const;
    confPtr_t configAtParam(double param, unsigned int* id_localpath=NULL) const;

    std::vector<confPtr_t> getNConfAtParam(double delta);
    std::vector<confPtr_t> getVectorOfConfiguration();

    uint					getIdOfPathAt(double param);
    LocalPath* 		getLocalPathPtrAt(unsigned int id) const;
    int						getNbOfPaths() const;
    int						getNbOfViaPoints() const;

    bool isValid() const;
    void resetIsValid();

    void 	updateRange();
    double computeSubPortionRange(const std::vector<LocalPath*>& portion) const;

    bool        replaceP3dTraj() const;
    p3d_traj* 	replaceP3dTraj(p3d_traj* trajPt) const;
    p3d_traj* 	replaceHumanP3dTraj(Robot*rob, p3d_traj* trajPt);

    void printAllLocalpathCost();
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
    
    int size() const
    {
        return m_Courbe.size();
    }

    confPtr_t getBegin() const
    {
        return m_Source;
    }

    confPtr_t getEnd() const
    {
        return m_Target;
    }

    std::vector<LocalPath*> getCourbe() const
    {
        return m_Courbe;
    }
    
    double getRangeMax() const {
        return computeSubPortionRange(m_Courbe);
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

    int mColor;

    /* Start and Goal (should never change) */
    confPtr_t m_Source;
    confPtr_t m_Target;
};
}

//#if defined( QT_LIBRARY ) 
#include <vector>
namespace API { class Trajectory; }
extern std::vector<API::Trajectory> trajToDraw;
void draw_traj_debug();
//#endif

#endif /* TRAJECTORY_HPP_ */

#ifndef HRICS_LEGIBILITY_HPP
#define HRICS_LEGIBILITY_HPP


#include "API/ConfigSpace/configuration.hpp"
#include <Eigen/Core>
#include <vector>

/**
 @ingroup HRICS

 These classes implement the legibility criterion of
 Anca Dragan and Siddartha Srinivasa

 */
namespace HRICS
{

class Predictability
{
public:
    Predictability();

    //! Add a goal to the predictability cost function
    void addGoal( const Eigen::VectorXd& g );

    //! Clear the goal set
    void clearGoals();

    //! returns the cost of a given trajectory
    double costPredict( const Eigen::MatrixXd& t );

    // Get a discretized interpolated trajectory
    Eigen::MatrixXd getInterpolatedTrajectory( const Eigen::VectorXd& a, const Eigen::VectorXd& b, int nb_points );

    //! interpolate between configurations
    Eigen::VectorXd interpolate( const Eigen::VectorXd& a, const Eigen::VectorXd& b, double u ) const;

private:
    std::vector<Eigen::VectorXd> goals_;
};

class Legibility : public Predictability
{
public:
    Legibility();

    //! Set the current trajectory
    void setTrajectory( const Eigen::MatrixXd& t );

    //! Set the current trajectory length
    void setLength(double l);

    //! Get the cost of the ith configuration
    double cost( int i );

private:
    double length_;
    Eigen::MatrixXd traj_;
};

}

#endif // HRICS_LEGIBILITY_HPP

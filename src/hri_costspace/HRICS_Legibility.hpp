#ifndef HRICS_LEGIBILITY_HPP
#define HRICS_LEGIBILITY_HPP

#include "API/ConfigSpace/configuration.hpp"
#include "planner/TrajectoryOptim/Stomp/control_cost.hpp"
#include <Eigen/Core>
#include <vector>

/**
 @ingroup HRICS

 These classes implement the legibility criterion of
 Anca Dragan and Siddartha Srinivasa

 */
namespace HRICS
{

class Predictability : public ControlCost
{
public:
    Predictability();

    //! Add a goal to the predictability cost function
    void addGoal( const Eigen::VectorXd& g );

    //! Clear the goal set
    void clearGoals();

protected:
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
    double legibilityCost( int i );

private:
    double length_;
    Eigen::MatrixXd traj_;
    Eigen::MatrixXd straight_line_;
    Eigen::VectorXd q_source_;
    std::vector<double> c_g_;
};

}

#endif // HRICS_LEGIBILITY_HPP

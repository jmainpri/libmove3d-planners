#ifndef CONTROL_COST_HPP
#define CONTROL_COST_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>

#include <vector>

class ControlCost
{
public:
    ControlCost();

    //! returns the cost of a given trajectory
    double cost( const Eigen::MatrixXd& t );

    //! sets the start and end points in the trajectory
    void fillTrajectory( const Eigen::VectorXd& a, const Eigen::VectorXd& b, Eigen::MatrixXd& traj );

    //! resample the matrix rows
    Eigen::MatrixXd resample( const Eigen::MatrixXd& m, int nb_points ) const;

    // Get a discretized interpolated trajectory
    Eigen::MatrixXd getInterpolatedTrajectory( const Eigen::VectorXd& a, const Eigen::VectorXd& b, int nb_points );

    //! interpolate between configurations
    Eigen::VectorXd interpolate( const Eigen::VectorXd& a, const Eigen::VectorXd& b, double u ) const;

protected:
    enum cost_type { vel=0, acc=1, jerk=2 } type_;
    int diff_rule_length_;
    double scaling_;
};

#endif // CONTROL_COST_HPP

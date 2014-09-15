#ifndef SMOOTHNESS_HPP
#define SMOOTHNESS_HPP

#include "features.hpp"
#include "planner/TrajectoryOptim/Stomp/control_cost.hpp"

namespace Move3D
{

////////////////////////////////////////
class LengthFeature : public Move3D::Feature
{
public:
    LengthFeature();
    Move3D::FeatureVect getFeatureCount( const Move3D::Trajectory& t );
    Move3D::FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0) );
    double scaling_;
};

////////////////////////////////////////
class TrajectorySmoothness : public Feature
{
public:
    TrajectorySmoothness();

    //! Returns a smoothness cost for the trajectory
    FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
    FeatureVect getFeatureCount(const Move3D::Trajectory& t);

    void setWeights(const WeightVect &w);

    //! Prints the control cost along the trajectory
    void printControlCosts( const std::vector<Eigen::VectorXd>& control_cost  );

    //! Set Buffer
    void setBuffer(const Eigen::MatrixXd& buffer) { control_cost_.setBuffer(buffer); buffer_is_filled_=true; }
    void clearBuffer() { buffer_is_filled_=false; }

protected:
    ControlCost control_cost_;

private:
    double computeControlCost( const Eigen::MatrixXd& traj );
    bool buffer_is_filled_;

};

////////////////////////////////////////
class VelocitySmoothness : public TrajectorySmoothness
{
public:
    VelocitySmoothness();
    void setWeights(const WeightVect &w);
};

////////////////////////////////////////
class AccelerationSmoothness : public TrajectorySmoothness
{
public:
    AccelerationSmoothness();
    void setWeights(const WeightVect &w);
};

////////////////////////////////////////
class JerkSmoothness : public TrajectorySmoothness
{
public:
    JerkSmoothness();
    void setWeights(const WeightVect &w);
};

////////////////////////////////////////
class SmoothnessFeature : public Move3D::Feature
{
public:
    SmoothnessFeature();
    Move3D::FeatureVect getFeatureCount( const Move3D::Trajectory& t );
    Move3D::FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0) );
    void setActiveDoFs( const std::vector<int>& active_dofs );
    void setWeights(const WeightVect &w);

    //! Set Buffer
    void setBuffer(const Eigen::MatrixXd& buffer);
    void clearBuffer();

private:
    LengthFeature length_;
    VelocitySmoothness velocity_;
    AccelerationSmoothness acceleration_;
    JerkSmoothness jerk_;
};

}

#endif // SMOOTHNESS_HPP

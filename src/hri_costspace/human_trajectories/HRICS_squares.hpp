#ifndef HRICS_SQUARES_HPP
#define HRICS_SQUARES_HPP

#include "HRICS_planar_feature.hpp"

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

namespace HRICS
{

class Square
{
public:
    Square( const Eigen::VectorXd& center, const Eigen::VectorXd& size )
    {
        center_ = center;
        size_ = size;
    }

    virtual void draw() const;

    Eigen::VectorXd center_;
    Eigen::VectorXd size_;
};

class Squares : public PlanarFeature
{
public:
    Squares();
    ~Squares();

    virtual void initialize();
    void computeSize();

    FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_features = std::vector<int>(0) );
    double getFeaturesJacobianMagnitude( const Move3D::Configuration& q );

    void produceDerivativeFeatureCostMap(int ith);

    double distToSquare(  const Square& square, const Move3D::Configuration& q  );
    bool isInAASquare( const std::vector<Eigen::Vector2d>& corners, Eigen::Vector2d p );
    double pointToLineSegmentDistance(const Eigen::VectorXd& p, const Eigen::VectorXd& p1, const Eigen::VectorXd& p2, Eigen::VectorXd& closestPoint);

    void draw();

protected:
    std::vector<const Square*> boxes_;
};

}

// Global cost function
bool HRICS_init_square_cost();

#endif // HRICS_SQUARES_HPP

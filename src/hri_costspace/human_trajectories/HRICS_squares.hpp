#ifndef HRICS_SQUARES_HPP
#define HRICS_SQUARES_HPP

#include "HRICS_planar_feature.hpp"

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

    FeatureVect getFeatures(const Configuration& q );
//    FeatureVect getFeatureCount(const API::Trajectory& t);
    double getFeaturesJacobianMagnitude(const Configuration& q);

    double jacobianCost(const Configuration& q);

    double distToSquare(  const Square& square, const Configuration& q  );
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

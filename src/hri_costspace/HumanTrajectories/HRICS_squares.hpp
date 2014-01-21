#ifndef HRICS_SQUARES_HPP
#define HRICS_SQUARES_HPP

#include "HRICS_planarfeature.hpp"

namespace HRICS
{

class Square
{
public:
    Square( const Eigen::Vector2d& center, double x, double y )
    {
        center_ = center;
        x_ = x;
        y_ = y;
    }

    void draw();

    Eigen::Vector2d center_;
    double x_;
    double y_;
};

class Squares : public PlanarFeature
{
public:
    Squares();
    ~Squares();

    void initialize();
    void computeSize();

    FeatureVect getFeatures(const Configuration& q );
    FeatureVect getFeatureCount(const API::Trajectory& t);

    double distToSquare(  const Square& square, const Configuration& q  );
    bool isInAASquare( const std::vector<Eigen::Vector2d>& corners, Eigen::Vector2d p );
    double pointToLineSegmentDistance(const Eigen::Vector2d& p, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, Eigen::Vector2d& closestPoint);

    void draw();

private:
    std::vector<Square> squares_;
};

}

// Global cost function
void HRICS_init_square_cost();

#endif // HRICS_SQUARES_HPP

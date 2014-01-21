#ifndef HRICS_SQUARES_HPP
#define HRICS_SQUARES_HPP

#include "HRICS_planarfeature.hpp"

namespace HRICS
{

class Square
{
public:
    Square( Robot* center, double x, double y )
    {
        center_ = center;
        x_ = x;
        y_ = y;
    }
    Robot* center_;
    double x_;
    double y_;
};

class Squares : public PlanarFeature
{
public:
    Squares();

    void initialize();
    void computeSize();

    FeatureVect getFeatures(const Configuration& q );
    FeatureVect getFeatureCount(const API::Trajectory& t);

private:
    void isInSquare( const Square& square, const Configuration& q );
    std::vector<Square> squares_;
};

}

// Global cost function
void HRICS_init_square_cost();

#endif // HRICS_SQUARES_HPP

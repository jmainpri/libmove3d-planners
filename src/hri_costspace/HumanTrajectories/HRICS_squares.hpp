#ifndef HRICS_SQUARES_HPP
#define HRICS_SQUARES_HPP

#include "HRICS_planarfeature.hpp"

namespace HRICS
{

class Squares : public PlanarFeature
{
public:
    Squares();

    void initialize();

    FeatureVect getFeatures(const Configuration& q );
    FeatureVect getFeatureCount(const API::Trajectory& t);
};

}

// Global cost function
void HRICS_init_square_cost();

#endif // HRICS_SQUARES_HPP

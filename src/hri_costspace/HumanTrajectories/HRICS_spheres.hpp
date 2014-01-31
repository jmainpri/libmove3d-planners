#ifndef HRICS_SPHERES_HPP
#define HRICS_SPHERES_HPP

#include "HRICS_planarfeature.hpp"

namespace HRICS
{

class Spheres : public PlanarFeature
{
public:
    Spheres();

    void initialize();

    FeatureVect getFeatures(const Configuration& q );
//    FeatureVect getFeatureCount(const API::Trajectory& t);
};

}

// Global cost function
void HRICS_init_sphere_cost();

#endif // HRICS_SPHERES_HPP

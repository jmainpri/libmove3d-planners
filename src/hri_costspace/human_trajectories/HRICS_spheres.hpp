#ifndef HRICS_SPHERES_HPP
#define HRICS_SPHERES_HPP

#include "HRICS_planar_feature.hpp"

namespace HRICS
{

class Spheres : public PlanarFeature
{
public:
    Spheres();

    void initialize();

    FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_features = std::vector<int>(0) ) ;
};

}

// Global cost function
bool HRICS_init_sphere_cost();

#endif // HRICS_SPHERES_HPP

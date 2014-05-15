#ifndef HRICS_SPHERES_HPP
#define HRICS_SPHERES_HPP

#include "planar_feature.hpp"

namespace Move3D
{

class Spheres : public PlanarFeature
{
public:
    Spheres();

    void initialize();

    FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_features = std::vector<int>(0) ) ;

private :
    double max_value_;
    double radius_;
};

}

// Global cost function
bool HRICS_init_sphere_cost();

#endif // HRICS_SPHERES_HPP

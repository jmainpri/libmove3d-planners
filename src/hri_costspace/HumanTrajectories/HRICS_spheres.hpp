#ifndef HRICS_SPHERES_HPP
#define HRICS_SPHERES_HPP

#include "API/Device/robot.hpp"
#include "HRICS_features.hpp"

namespace HRICS
{

class Spheres : public Feature
{
public:
    Spheres();

    void initialize();
    void setWeights( const std::vector<double>& w ) { w_ = w; }
    double cost( Configuration& q );
    FeatureVect features( Configuration& q );

    FeatureVect getFeatureCount(const API::Trajectory& t);

private:
    std::vector<Robot*> centers_;
    std::vector<double> w_;
};

}

// Global cost function
void HRICS_init_sphere_cost();

extern HRICS::Spheres* global_SphereCostFct;
extern double HRICS_sphere_cost(Configuration& q);

#endif // HRICS_SPHERES_HPP

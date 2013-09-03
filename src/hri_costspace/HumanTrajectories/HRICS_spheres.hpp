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
    void setWeights( const WeightVect& w ) { w_ = w; }
    WeightVect getWeights() { return w_; }

    double cost( Configuration& q );
    FeatureVect features( Configuration& q );
    void produceCostMap();

    FeatureVect getFeatureCount(const API::Trajectory& t);

private:
    Robot* robot_;
    std::vector<Robot*> centers_;
    FeatureVect w_;
};

}

// Global cost function
void HRICS_init_sphere_cost();

extern HRICS::Spheres* global_SphereCostFct;
extern double HRICS_sphere_cost(Configuration& q);

#endif // HRICS_SPHERES_HPP

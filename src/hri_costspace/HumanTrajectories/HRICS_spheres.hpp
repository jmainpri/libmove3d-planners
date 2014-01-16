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
    void addCenters(int nb_centers);

    FeatureVect getFeatures(const Configuration& q );
    FeatureVect getFeatureCount(const API::Trajectory& t);
    // FeatureJacobian getFeaturesJacobian(const Configuration& q);

    void produceCostMap();
    void produceDerivativeFeatureCostMap();
    void placeCenterGrid(bool on_wall);
    void printWeights() const;

private:
    Robot* robot_;
    std::vector<Robot*> centers_;
};

}

// Global cost function
void HRICS_init_sphere_cost();

extern HRICS::Spheres* global_SphereCostFct;
extern double HRICS_sphere_cost(Configuration& q);

#endif // HRICS_SPHERES_HPP

#ifndef HRICS_SPHERES_HPP
#define HRICS_SPHERES_HPP

#include "API/Device/robot.hpp"

namespace HRICS
{

class Spheres
{
public:
    Spheres();

    void initialize();
    double cost( Configuration& q );

private:
    std::vector<Robot*> centers_;
};

}

// Global cost function
void HRICS_init_sphere_cost();
extern double HRICS_sphere_cost(Configuration& q);

#endif // HRICS_SPHERES_HPP

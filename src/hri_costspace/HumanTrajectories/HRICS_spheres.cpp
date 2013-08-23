#include "HRICS_spheres.hpp"

#include "API/project.hpp"
#include "planner/cost_space.hpp"

#include <boost/bind.hpp>

using namespace HRICS;
using std::cout;
using std::endl;

Spheres::Spheres()
{

}

void Spheres::initialize()
{
    Scene* sce = global_Project->getActiveScene();

    centers_.clear();
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_1") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_2") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_3") );
}

double Spheres::cost( Configuration& q )
{
    double cost = 0.0;

    Eigen::VectorXd x = q.getEigenVector(6,7);

    for( int i=0; i< int(centers_.size()); i++ )
    {
        Eigen::VectorXd mu = centers_[i]->getCurrentPos()->getEigenVector(6,7);
        cost += exp( -( x - mu ).norm() );
    }

    return cost;
}

static Spheres* cost_fct;

double HRICS_sphere_cost(Configuration& q)
{
    return cost_fct->cost( q );
}

void HRICS_init_sphere_cost()
{
    cout << "initializes sphere cost" << endl;

    cost_fct = new Spheres();
    cost_fct->initialize();

    cout << "add cost functions : " << global_costSpace << endl;

    global_costSpace->addCost("costSpheres", boost::bind(HRICS_sphere_cost, _1) );
    global_costSpace->setCost("costSpheres");
}

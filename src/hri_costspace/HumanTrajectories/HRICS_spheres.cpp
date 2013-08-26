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

    w_.clear();
    w_.resize( centers_.size(), 1.0 );
}

double Spheres::cost( Configuration& q )
{
    double cost = 0.0;

    FeatureVect phi = features( q );

    for( int i=0; i< int(phi.size()); i++ )
        cost += w_[i]*phi[i];

    return cost;
}

FeatureVect Spheres::features( Configuration& q )
{
    FeatureVect features(centers_.size());

    Eigen::VectorXd x = q.getEigenVector(6,7);

    for( int i=0; i< int(centers_.size()); i++ )
    {
        Eigen::VectorXd mu = centers_[i]->getCurrentPos()->getEigenVector(6,7);
        features[i] = exp( -( x - mu ).norm() );
    }

    return features;
}

FeatureVect Spheres::getFeatureCount( const API::Trajectory& t )
{
    FeatureVect vect( centers_.size() );

    for(int i=0;i<t.getNbOfViaPoints();i++)
    {
        confPtr_t q = t[i];
        vect += features( *q );
    }

    return vect;
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Spheres* global_SphereCostFct=NULL;

double HRICS_sphere_cost(Configuration& q)
{
    return global_SphereCostFct->cost( q );
}

void HRICS_init_sphere_cost()
{
    cout << "initializes sphere cost" << endl;

    global_SphereCostFct = new Spheres();
    global_SphereCostFct->initialize();

    cout << "add cost functions : " << global_costSpace << endl;

    global_costSpace->addCost("costSpheres", boost::bind(HRICS_sphere_cost, _1) );
    global_costSpace->setCost("costSpheres");
}

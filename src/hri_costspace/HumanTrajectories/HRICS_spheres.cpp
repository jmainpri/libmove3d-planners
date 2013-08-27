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
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_01") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_02") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_03") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_04") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_05") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_06") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_07") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_08") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_09") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_10") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_11") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_12") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_13") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_14") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_15") );
    centers_.push_back( sce->getRobotByName("GAUSSIAN_MU_16") );

    w_.clear();
    w_.resize( centers_.size(), 1.0 );
    w_[0] = 10;
    w_[1] = 2;
    w_[2] = 12;
    w_[3] = 4;
    w_[4] = 30;
    w_[5] = 6;
    w_[6] = 12;
    w_[7] = 8;
    w_[8] = 15;
    w_[9] = 10;
    w_[10] = 10;
    w_[11] = 12;
    w_[12] = 2;
    w_[13] = 14;
    w_[14] = 25;
    w_[15] = 16;
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

    const double factor_distance = 5;

    for( int i=0; i< int(centers_.size()); i++ )
    {
        Eigen::VectorXd mu = centers_[i]->getCurrentPos()->getEigenVector(6,7);
        features[i] = exp( -( x - mu ).norm()/factor_distance );
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

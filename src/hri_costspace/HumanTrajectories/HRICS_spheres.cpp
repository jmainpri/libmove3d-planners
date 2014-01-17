#include "HRICS_spheres.hpp"

#include "HRICS_GestParameters.hpp"
#include "HRICS_parameters.hpp"

#include "API/project.hpp"
#include "planner/cost_space.hpp"

#include <boost/bind.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace HRICS;
using std::cout;
using std::endl;

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Spheres* global_SphereCostFct=NULL;

void HRICS_init_sphere_cost()
{
    cout << "Initializing sphere cost" << endl;

    global_SphereCostFct = new Spheres();
    global_SphereCostFct->initialize();

    if( global_SphereCostFct->getNumberOfFeatures() > 0 )
    {
        global_PlanarCostFct = global_SphereCostFct;
        std::string cost_function("costSpheres");
        cout << "add cost functions : " << cost_function << endl;
        global_costSpace->addCost( cost_function, boost::bind( &Spheres::cost, global_SphereCostFct, _1) );
        global_costSpace->setCost( cost_function );
    }
    else{
        delete global_SphereCostFct;
        global_SphereCostFct = NULL;
    }
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Spheres::Spheres()
{

}

void Spheres::initialize()
{
    Scene* sce = global_Project->getActiveScene();

    robot_ = sce->getActiveRobot();
    if( robot_ == NULL ){
        return;
    }

    centers_.clear();

    // int nb_spheres = sce->getNumberOfRobots()-1;
    int nb_spheres = addCenters("GAUSSIAN");
    if( nb_spheres == 0 ){
        return;
    }

    w_.resize( centers_.size() );

    int i=0;

    if( nb_spheres == 64 )
    {
        placeCenterGrid( true );

        w_[i++] = 8; w_[i++] = 8;  w_[i++] = 8; w_[i++] = 8;  w_[i++] = 8; w_[i++] = 8;  w_[i++] = 8; w_[i++] = 8;
        w_[i++] = 8; w_[i++] = 1; w_[i++] = 1; w_[i++] = 1; w_[i++] = 2;  w_[i++] = 1; w_[i++] = 1; w_[i++] = 40;
        w_[i++] = 100; w_[i++] = 100; w_[i++] = 100; w_[i++] = 60; w_[i++] = 50; w_[i++] = 6;  w_[i++] = 1; w_[i++] = 40;
        w_[i++] = 100; w_[i++] = 100; w_[i++] = 50; w_[i++] = 30; w_[i++] = 1;  w_[i++] = 1; w_[i++] = 1; w_[i++] = 50;

        w_[i++] = 100; w_[i++] = 1; w_[i++] = 5; w_[i++] = 5; w_[i++] = 50; w_[i++] = 100; w_[i++] = 100; w_[i++] = 100;
        w_[i++] = 8; w_[i++] = 1;  w_[i++] = 30; w_[i++] = 99;  w_[i++] = 99; w_[i++] = 100;  w_[i++] = 100; w_[i++] = 100;
        w_[i++] =50; w_[i++] = 1; w_[i++] = 1; w_[i++] = 1; w_[i++] = 3; w_[i++] = 1; w_[i++] = 1; w_[i++] = 10;
        w_[i++] = 100; w_[i++] = 100;  w_[i++] = 12; w_[i++] = 10;  w_[i++] = 10; w_[i++] = 10;  w_[i++] = 12; w_[i++] = 10;
    }

    if( nb_spheres == 4 )
    {
        placeCenterGrid( false );

        w_[i++] = 10;
        w_[i++] = 5;
        w_[i++] = 10;
        w_[i++] = 5;
    }

    if( nb_spheres == 16 )
    {
        placeCenterGrid( true );
        w_[i++] = 100; w_[i++] = 100;  w_[i++] = 100; w_[i++] = 100;
        w_[i++] = 100; w_[i++] = 50;  w_[i++] = 8; w_[i++] = 100;
        w_[i++] = 100; w_[i++] = 30; w_[i++] = 50; w_[i++] = 100;
        w_[i++] = 100;  w_[i++] = 100; w_[i++] = 100; w_[i++] = 100;
    }

    double max = w_.maxCoeff();
    w_ /= max;

    std::vector<int> active_dofs;
    active_dofs.push_back( 6 );
    active_dofs.push_back( 7 );
    setActiveDofs( active_dofs );
}

FeatureVect Spheres::getFeatures( const Configuration& q )
{
    FeatureVect features(centers_.size());

    Eigen::VectorXd x = q.getEigenVector(6,7);

    // The factor distance when larger
    const double factor_distance = 10.0;
    const double factor_height = HriEnv->getDouble(HricsParam::ioc_spheres_power);

    for( int i=0; i< int(centers_.size()); i++ )
    {
        Eigen::VectorXd mu = centers_[i]->getCurrentPos()->getEigenVector(6,7);
        features[i] = pow(exp( -( x - mu ).norm()/factor_distance ),factor_height);
    }

    return features;
}

//FeatureJacobian Spheres::getFeaturesJacobian(const Configuration& q_0)
//{
//    const double eps = 1e-6;

//    Configuration q_1(*robot_->getCurrentPos());
//    Configuration q_2(*robot_->getCurrentPos());

//    q_1[6] = q_0[6] + eps;
//    q_1[7] = q_0[7];

//    q_2[6] = q_0[6];
//    q_2[7] = q_0[7] + eps;

//    FeatureVect f_0 = getFeatures( q_0 );
//    FeatureVect f_1 = getFeatures( q_1 );
//    FeatureVect f_2 = getFeatures( q_2 );

//    FeatureJacobian J = Eigen::MatrixXd::Zero(getNumberOfFeatures(),2);
//    J.col(0) = (f_1 - f_0) / eps;
//    J.col(1) = (f_2 - f_0) / eps;

//    return J;
//}

FeatureVect Spheres::getFeatureCount( const API::Trajectory& t )
{
    FeatureVect phi( Eigen::VectorXd::Zero(centers_.size()) );

    for(int i=1;i<t.getNbOfViaPoints();i++)
    {
        confPtr_t q_1 = t[i-1];
        confPtr_t q_2 = t[i];
        Eigen::VectorXd pos1 = q_1->getEigenVector(6,7);
        Eigen::VectorXd pos2 = q_2->getEigenVector(6,7);
        double dist = ( pos1 - pos2 ).norm();
        phi += getFeatures( *q_1 )*dist;
    }

    return phi;
}

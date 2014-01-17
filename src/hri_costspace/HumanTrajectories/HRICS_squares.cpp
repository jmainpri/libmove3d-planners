#include "HRICS_squares.hpp"

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

Squares* global_SquareCostFct=NULL;

void HRICS_init_square_cost()
{
    cout << "Initializing square cost" << endl;

    global_SquareCostFct = new Squares();
    global_SquareCostFct->initialize();

    if( global_SquareCostFct->getNumberOfFeatures() > 0 )
    {
        global_PlanarCostFct = global_SquareCostFct;

        std::string cost_function("costSquares");
        cout << "add cost functions : " << cost_function << endl;
        global_costSpace->addCost( cost_function, boost::bind( &Squares::cost, global_SquareCostFct, _1) );
        global_costSpace->setCost( cost_function );
    }
    else{
        delete global_SquareCostFct;
        global_SquareCostFct = NULL;
    }
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Squares::Squares()
{

}

void Squares::initialize()
{
    cout << "--------------------------------"  << endl;
    cout << "INIT SQUARES" << endl;

    Scene* sce = global_Project->getActiveScene();

    robot_ = sce->getActiveRobot();
    if( robot_ == NULL ){
        return;
    }

    centers_.clear();

    // int nb_spheres = sce->getNumberOfRobots()-1;
    int nb_squares = addCenters("SQUARE");
    if( nb_squares == 0 ){
        return;
    }

     w_.resize( centers_.size() );

    int i=0;

    cout << "nb_squares : " << nb_squares << endl;

    if( nb_squares == 16 )
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

FeatureVect Squares::getFeatures( const Configuration& q )
{
    FeatureVect features(centers_.size());

    robot_->setAndUpdate(q);

    // The factor distance when larger
    const double factor_distance = 10.0;
    const double factor_height = HriEnv->getDouble(HricsParam::ioc_spheres_power);

//    if( robot_->isInCollision() )
//    {
//        dist = 0;
//    }
//    else {
//        dist = robot_->distanceToEnviroment(); // TODO FIX FUNCTION
//    }

    for( int i=0; i< int(centers_.size()); i++ )
    {
        double dist = robot_->distanceToRobot( centers_[i] );
        // cout << dist << endl;
        features[i] = pow( exp( -dist/factor_distance ), factor_height );
    }

    // cout << "features.norm() : " << features.norm() << endl;

    return features;
}

FeatureVect Squares::getFeatureCount( const API::Trajectory& t )
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

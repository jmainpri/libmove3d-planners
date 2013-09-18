#include "HRICS_spheres.hpp"

#include "API/project.hpp"
#include "planner/cost_space.hpp"

#include <boost/bind.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace HRICS;
using std::cout;
using std::endl;

Spheres::Spheres()
{

}

void Spheres::initialize()
{
    Scene* sce = global_Project->getActiveScene();

    robot_ = sce->getActiveRobot();

    centers_.clear();
    addCenters( sce->getNumberOfRobots()-1 );
    placeCenterGrid();

    w_.resize( centers_.size() );

    int i=0;

    w_[i++] = 10; w_[i++] = 2;  w_[i++] = 12; w_[i++] = 4;  w_[i++] = 30; w_[i++] = 6;  w_[i++] = 12; w_[i++] = 8;
    w_[i++] = 15; w_[i++] = 10; w_[i++] = 10; w_[i++] = 12; w_[i++] = 2;  w_[i++] = 14; w_[i++] = 25; w_[i++] = 16;
    w_[i++] = 10; w_[i++] = 2;  w_[i++] = 12; w_[i++] = 4;  w_[i++] = 30; w_[i++] = 6;  w_[i++] = 12; w_[i++] = 8;
    w_[i++] = 15; w_[i++] = 10; w_[i++] = 10; w_[i++] = 12; w_[i++] = 2;  w_[i++] = 14; w_[i++] = 25; w_[i++] = 16;

    w_[i++] = 15; w_[i++] = 10; w_[i++] = 10; w_[i++] = 12; w_[i++] = 2;  w_[i++] = 14; w_[i++] = 25; w_[i++] = 16;
    w_[i++] = 10; w_[i++] = 2;  w_[i++] = 12; w_[i++] = 4;  w_[i++] = 30; w_[i++] = 6;  w_[i++] = 12; w_[i++] = 8;
    w_[i++] = 15; w_[i++] = 10; w_[i++] = 10; w_[i++] = 12; w_[i++] = 2;  w_[i++] = 14; w_[i++] = 25; w_[i++] = 16;
    w_[i++] = 10; w_[i++] = 2;  w_[i++] = 12; w_[i++] = 4;  w_[i++] = 30; w_[i++] = 6;  w_[i++] = 12; w_[i++] = 8;

    produceCostMap();
}

void Spheres::addCenters(int nb_centers)
{
    Scene* sce = global_Project->getActiveScene();
    std::stringstream ss;

    for( int i=1;i<=nb_centers;i++)
    {
        ss.str(""); // clear stream
        ss << "GAUSSIAN_MU_" << std::setw(2) << std::setfill( '0' ) << i ;
        centers_.push_back( sce->getRobotByName( ss.str() ) );
        cout << "Add robot : " << centers_.back()->getName() << endl;
    }
}

void Spheres::placeCenterGrid()
{
    double max_1, max_2;
    double min_1, min_2;
    robot_->getJoint(1)->getDofBounds( 0, min_1, max_1 );
    robot_->getJoint(1)->getDofBounds( 1, min_2, max_2 );

    int nb_cells = std::sqrt( double(centers_.size()) );

    for( int i=0; i<nb_cells; i++ )
    {
        for( int j=0; j<nb_cells; j++ )
        {
            int id = i*nb_cells+j;
            confPtr_t q = centers_[id]->getCurrentPos();
            (*q)[6] = min_1 + double(i)*(max_1-min_1)/double(nb_cells-1);
            (*q)[7] = min_2 + double(j)*(max_2-min_2)/double(nb_cells-1);
            centers_[id]->setAndUpdate( *q );
            centers_[id]->setInitPos( *q );
        }
    }
}

void Spheres::produceCostMap()
{
    double max_1, max_2;
    double min_1, min_2;
    robot_->getJoint(1)->getDofBounds( 0, min_1, max_1 );
    robot_->getJoint(1)->getDofBounds( 1, min_2, max_2 );

    int nb_cells = 100;
    Eigen::MatrixXd mat( nb_cells, nb_cells );

    for( int i=0; i<nb_cells; i++ )
    {
        for( int j=0; j<nb_cells; j++ )
        {
            confPtr_t q = robot_->getCurrentPos();
            (*q)[6] = min_1 + double(i)*(max_1-min_1)/double(nb_cells-1);
            (*q)[7] = min_2 + double(j)*(max_2-min_2)/double(nb_cells-1);
            mat(i,j) = cost(*q);
        }
    }

    std::string filename("matlab/cost_map.txt");
    cout << "Save cost map to : " << filename << endl;
    std::ofstream file( filename.c_str() );
    if (file.is_open())
        file << mat << '\n';
    file.close();
}

FeatureVect Spheres::getFeatures(const Configuration& q)
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
        vect += 0.001*getFeatures( *q );
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
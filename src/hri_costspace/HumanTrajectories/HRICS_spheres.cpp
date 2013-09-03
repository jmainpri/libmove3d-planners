#include "HRICS_spheres.hpp"

#include "API/project.hpp"
#include "planner/cost_space.hpp"

#include <boost/bind.hpp>
#include <fstream>

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

//    w_.clear();
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
    FeatureVect phi = features( q );
    double cost = w_.transpose()*phi;
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
        vect += 0.001*features( *q );
    }

    return vect;
}

void Spheres::produceCostMap()
{
    double max_1, max_2;
    double min_1, min_2;
    robot_->getJoint(1)->getDofBounds( 0, min_1 ,max_1 );
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

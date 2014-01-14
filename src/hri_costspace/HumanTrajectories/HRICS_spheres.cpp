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

    w_[i++] = 8; w_[i++] = 8;  w_[i++] = 8; w_[i++] = 8;  w_[i++] = 8; w_[i++] = 8;  w_[i++] = 8; w_[i++] = 8;
    w_[i++] = 8; w_[i++] = 1; w_[i++] = 1; w_[i++] = 1; w_[i++] = 2;  w_[i++] = 1; w_[i++] = 1; w_[i++] = 40;
    w_[i++] = 100; w_[i++] = 100; w_[i++] = 100; w_[i++] = 60; w_[i++] = 50; w_[i++] = 6;  w_[i++] = 1; w_[i++] = 40;
    w_[i++] = 100; w_[i++] = 100; w_[i++] = 50; w_[i++] = 30; w_[i++] = 1;  w_[i++] = 1; w_[i++] = 1; w_[i++] = 50;

    w_[i++] = 100; w_[i++] = 1; w_[i++] = 5; w_[i++] = 5; w_[i++] = 50; w_[i++] = 100; w_[i++] = 100; w_[i++] = 100;
    w_[i++] = 8; w_[i++] = 1;  w_[i++] = 30; w_[i++] = 99;  w_[i++] = 99; w_[i++] = 100;  w_[i++] = 100; w_[i++] = 100;
    w_[i++] =50; w_[i++] = 1; w_[i++] = 1; w_[i++] = 1; w_[i++] = 3; w_[i++] = 1; w_[i++] = 1; w_[i++] = 10;
    w_[i++] = 100; w_[i++] = 100;  w_[i++] = 12; w_[i++] = 10;  w_[i++] = 10; w_[i++] = 10;  w_[i++] = 12; w_[i++] = 10;

    double max = w_.maxCoeff();
    w_ /= max;

    std::vector<int> active_dofs;
    active_dofs.push_back( 6 );
    active_dofs.push_back( 7 );
    setActiveDofs( active_dofs );
}

void Spheres::printWeights() const
{
    cout << "weights : center" << endl;
    cout.precision(3);

    int n=8;
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<n;j++)
        {
            cout << "\t " << w_[n*i+j] << std::fixed;
        }
        cout << endl;
    }
}

void Spheres::addCenters(int nb_centers)
{
    Scene* sce = global_Project->getActiveScene();
    std::stringstream ss;

    double gray_scale = 0.0;
    double color_vect[4];

    for( int i=1; i<=nb_centers; i++)
    {
        ss.str(""); // clear stream
        ss << "GAUSSIAN_MU_" << std::setw(2) << std::setfill( '0' ) << i ;
        centers_.push_back( sce->getRobotByName( ss.str() ) );
        cout << "Add robot : " << centers_.back()->getName() << endl;

        p3d_obj* o = p3d_get_robot_body_by_name( centers_.back()->getRobotStruct(), "body" );
        cout << o->name << endl;

        gray_scale = 1-double(i)/double(nb_centers); // TODO fix
//        color_vect[0] = gray_scale;
//        color_vect[1] = gray_scale;
//        color_vect[2] = gray_scale;
//        color_vect[3] = 1.0;

        GroundColorMixGreenToRed( color_vect, gray_scale );
        color_vect[3] = 1.0;

        p3d_poly_set_color( o->pol[1], Any, color_vect );
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
            cout << "c (" << id << ") : " << (*q)[6] << " , " << (*q)[7] << endl;
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

    std::string filename("matlab/cost_map_64.txt");
    cout << "Save cost map to : " << filename << endl;
    std::ofstream file( filename.c_str() );
    if (file.is_open())
        file << mat << '\n';
    file.close();
}

void Spheres::produceDerivativeFeatureCostMap()
{
    double max_1, max_2;
    double min_1, min_2;
    robot_->getJoint(1)->getDofBounds( 0, min_1, max_1 );
    robot_->getJoint(1)->getDofBounds( 1, min_2, max_2 );

    int nb_cells = 100;
    Eigen::MatrixXd mat1( nb_cells, nb_cells );
    Eigen::MatrixXd mat2( nb_cells, nb_cells );

    for( int i=0; i<nb_cells; i++ )
    {
        for( int j=0; j<nb_cells; j++ )
        {
            confPtr_t q = robot_->getCurrentPos();
            (*q)[6] = min_1 + double(i)*(max_1-min_1)/double(nb_cells-1);
            (*q)[7] = min_2 + double(j)*(max_2-min_2)/double(nb_cells-1);
            FeatureJacobian J = getFeaturesJacobian(*q);
            // cout << "J : " << endl << J << endl;
            mat1(i,j) = J.col(0).maxCoeff();
            mat2(i,j) = J.col(1).maxCoeff();
        }
    }

    std::string filename1("matlab/cost_map_derv_0_64.txt");
    std::string filename2("matlab/cost_map_derv_1_64.txt");

    cout << "Save derivative cost map to : " << filename1 << endl;
    std::ofstream file1( filename1.c_str() );
    if (file1.is_open())
        file1 << mat1 << '\n';
    file1.close();

    cout << "Save derivative cost map to : " << filename2 << endl;
    std::ofstream file2( filename2.c_str() );
    if (file2.is_open())
        file2 << mat1 << '\n';
    file2.close();
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

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Spheres* global_SphereCostFct=NULL;

double HRICS_sphere_cost(Configuration& q)
{
    if( GestEnv->getBool(GestParam::print_debug) )
    {
        FeatureVect phi = global_SphereCostFct->getFeatures( q );
        WeightVect w = global_SphereCostFct->getWeights();
        cout << "phi : " << endl;
        cout << phi.transpose() << endl;
        cout << "w : " << endl;
        cout << w.transpose() << endl;
        cout << endl;
    }
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

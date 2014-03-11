#include "HRICS_squares.hpp"

#include "HRICS_GestParameters.hpp"
#include "HRICS_parameters.hpp"

#include "API/Graphic/drawModule.hpp"
#include "API/project.hpp"
#include "API/misc_functions.hpp"

#include "planner/cost_space.hpp"

#include <boost/bind.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "Graphic-pkg.h"

using namespace Move3D;
using namespace HRICS;
using std::cout;
using std::endl;

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Squares* global_SquareCostFct=NULL;

bool HRICS_init_square_cost()
{
    cout << "Initializing square cost" << endl;

    global_SquareCostFct = new Squares();
    global_SquareCostFct->initialize();

    if( global_SquareCostFct->getNumberOfFeatures() > 0 )
    {
        cout << "add cost functions : " << "costSquares" << endl;
        global_PlanarCostFct = global_SquareCostFct;
        global_costSpace->addCost( "costSquares", boost::bind( &Squares::cost, global_SquareCostFct, _1) );
        global_costSpace->addCost( "costSquaresJacobian", boost::bind( &PlanarFeature::jacobianCost, global_SquareCostFct, _1) );
        // global_costSpace->setCost( "costSquares" );
        return true;
    }
    else{
        delete global_SquareCostFct;
        global_SquareCostFct = NULL;
        return false;
    }
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

void Square::draw() const
{
    cout << __PRETTY_FUNCTION__ << endl;
    g3d_draw_simple_box( center_[0]-size_[0], center_[0]+size_[0], center_[1]-size_[1], center_[1]+size_[1], 0.0, 0.0, 0, 0, 1.0);
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Squares::Squares() : PlanarFeature()
{
    // Uncomment to draw squares
    //    if( global_DrawModule )
    //    {
    //        global_DrawModule->addDrawFunction( "Squares", boost::bind( &Squares::draw, this) );
    //        global_DrawModule->enableDrawFunction( "Squares" );
    //    }
}

Squares::~Squares()
{
    // Uncomment to draw squares
    //    if( global_DrawModule )
    //    {
    //        global_DrawModule->deleteDrawFunction( "Squares" );
    //    }
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
        computeSize();
        w_[i++] = 100;  w_[i++] = 100;  w_[i++] = 100; w_[i++] = 100;
        w_[i++] = 100;  w_[i++] = 50;   w_[i++] = 8;   w_[i++] = 100;
        w_[i++] = 100;  w_[i++] = 30;   w_[i++] = 50;  w_[i++] = 100;
        w_[i++] = 100;  w_[i++] = 100;  w_[i++] = 100; w_[i++] = 100;
    }

    double max = w_.maxCoeff();
    w_ /= max;

    cout << "w_ : " << w_.transpose() << endl;

    active_dofs_.resize(2);
    active_dofs_[0] = 6;
    active_dofs_[1] = 7;
}

double Squares::getFeaturesJacobianMagnitude( const Configuration& q )
{
    FeatureJacobian J = getFeaturesJacobian( q );
    double magnitude = ( std::abs(J.maxCoeff()) + std::abs(J.minCoeff()) ) / 2;

    // magnitude = 1 / magnitude;
    // cout << J << endl;
    // cout << magnitude << endl;
    // Maybe average the min and max coefficient
    return magnitude;
}

void Squares::produceDerivativeFeatureCostMap(int ith)
{
    double max_1, max_2;
    double min_1, min_2;
    robot_->getJoint(1)->getDofBounds( 0, min_1, max_1 );
    robot_->getJoint(1)->getDofBounds( 1, min_2, max_2 );

    int nb_cells = 100;
    Eigen::MatrixXd mat0( nb_cells, nb_cells );

    for( int i=0; i<nb_cells; i++ )
    {
        for( int j=0; j<nb_cells; j++ )
        {
            confPtr_t q = robot_->getCurrentPos();
            (*q)[6] = min_1 + double(i)*(max_1-min_1)/double(nb_cells-1);
            (*q)[7] = min_2 + double(j)*(max_2-min_2)/double(nb_cells-1);

            mat0(i,j) = jacobianCost( *q );
        }
    }

    std::stringstream ss;
    ss.str(""); ss << cost_map_folder + "cost_jac_" << std::setw(3) << std::setfill( '0' ) << ith << ".txt";
    move3d_save_matrix_to_file( mat0, ss.str() );
}

void Squares::computeSize()
{
    cout << "compute sizes" << endl;

    boxes_.clear();

    for( int i=0; i< int(centers_.size()); i++ )
    {
        p3d_obj* o = p3d_get_robot_body_by_name( centers_[i]->getRobotStruct(), "body" );
        //        cout << o->name << " : " << o->np << " , ";
        //        for(int j=0;j<o->np;j++)
        //            cout << o->pol[j]->entity_type << " , ";

        //        cout << o->name ;
        //        cout << "( " ;
        //        cout << o->pol[0]->primitive_data->x_length << " , ";
        //        cout << o->pol[0]->primitive_data->y_length ; // << " , ";
        //        cout << o->pol[0]->primitive_data->z_length ;
        //        cout << " )" ;

        Eigen::Vector3d p = centers_[i]->getJoint(1)->getVectorPos();

        Eigen::VectorXd center(2);
        center[0] = p[0];
        center[1] = p[1];

        Eigen::VectorXd size(2);
        size[0] = o->pol[0]->primitive_data->x_length/2;
        size[1] = o->pol[0]->primitive_data->y_length/2;

        boxes_.push_back( new Square( center, size ) );

        //        cout << "( " ;
        //        cout << o->pol[0]->pos0[0][3] << " , ";
        //        cout << o->pol[0]->pos0[1][3] << " , ";
        //        cout << o->pol[0]->pos0[2][3] ;
        //        cout << " )" ;

        //        cout << "( " ;
        //        cout << o->jnt->abs_pos[0][3] << " , ";
        //        cout << o->jnt->abs_pos[1][3] << " , ";
        //        cout << o->jnt->abs_pos[2][3] ;
        //        cout << " )" ;

        //        cout << "( " ;
        //        cout << p[0] << " , ";
        //        cout << p[1] ; // << " , ";
        //        cout << p[2] ;
        //        cout << " )" ;

        //        cout << endl;
    }
}

bool Squares::isInAASquare( const std::vector<Eigen::Vector2d>& corners, Eigen::Vector2d p )
{
    bool outside[4];
    outside[0] = true;
    outside[1] = true;
    outside[2] = true;
    outside[3] = true;

    for( int i=0;i<4;i++)
    {
        if( ! (p[0] < corners[i][0]) )
        {
            outside[0] = false;
        }
        if( ! (p[0] > corners[i][0]) )
        {
            outside[1] = false;
        }
        if( ! (p[1] < corners[i][1]) )
        {
            outside[2] = false;
        }
        if( ! (p[1] > corners[i][1]) )
        {
            outside[3] = false;
        }
    }

    if( outside[0] || outside[1] || outside[2] || outside[3] )
        return false;

    return true;
}

double Squares::distToSquare(  const Square& square, const Configuration& q  )
{
    std::vector<Eigen::Vector2d> corners(4);

    Eigen::Vector2d p = q.getEigenVector(6,7);

    corners[0][0] = square.center_[0] + square.size_[0];
    corners[0][1] = square.center_[1] + square.size_[1];

    corners[1][0] = square.center_[0] - square.size_[0];
    corners[1][1] = square.center_[1] + square.size_[1];

    corners[2][0] = square.center_[0] + square.size_[0];
    corners[2][1] = square.center_[1] - square.size_[1];

    corners[3][0] = square.center_[0] - square.size_[0];
    corners[3][1] = square.center_[1] - square.size_[1];

    if( isInAASquare( corners, p ) )
    {
        return 0.0;
    }
    else {
        std::vector<double> distances(4);
        Eigen::VectorXd closestPoint(2); // TODO use results from is in square
        distances[0] = pointToLineSegmentDistance( p, corners[0], corners[1], closestPoint );
        distances[1] = pointToLineSegmentDistance( p, corners[1], corners[3], closestPoint );
        distances[2] = pointToLineSegmentDistance( p, corners[3], corners[2], closestPoint );
        distances[3] = pointToLineSegmentDistance( p, corners[2], corners[0], closestPoint );
        return *std::min_element( distances.begin(), distances.end() );
    }
}

//! Computes the closest point of a point to a segment line.
//! \param p the point
//! \param p1 the first point of the segment line
//! \param p2 the second point of the segment line
//! \param closestPoint the point on the segment that is the closest to p
//! \return the minimimal distance between the segment and the point
double Squares::pointToLineSegmentDistance(const Eigen::VectorXd& p, const Eigen::VectorXd& p1, const Eigen::VectorXd& p2, Eigen::VectorXd& closestPoint)
{
    double alpha, d1, d2, norm_p1p2;
    Eigen::VectorXd p1p2, p1p2_n, p1p, proj;

    // Precompute the norm and the dot product
    p1p2			= p2 - p1;
    norm_p1p2	= p1p2.norm();
    p1p2_n		= p1p2.normalized();

    p1p			= p - p1;
    alpha		= p1p.dot(p1p2_n);

    // the projection is on the segment -> it is the closest point
    if( (alpha > 0) && (alpha < norm_p1p2) )
    {
        proj			= p1 + alpha*p1p2_n;
        closestPoint	= proj;
        return (proj-p).norm();
    }
    else
    {
        d1		= (p1-p).norm();
        d2		= (p2-p).norm();

        if( d1 < d2 )
        {
            closestPoint = p1;
            return d1;
        }
        else
        {
            closestPoint = p2;
            return d2;
        }
    }
}

FeatureVect Squares::getFeatures( const Configuration& q, std::vector<int> active_features )
{
    FeatureVect features(Eigen::VectorXd::Zero(centers_.size()));

    robot_->setAndUpdate(q);

    // The factor distance when larger
    const double factor_distance = 10.0;
    const double factor_height = HriEnv->getDouble(HricsParam::ioc_spheres_power);
    const std::vector<int>& active_features_sq = active_features.empty() ? active_features_ : active_features ;

    for( int i=0; i< int(active_features_sq.size()); i++ )
    {
        int k = active_features_sq[i];
        double dist = distToSquare( *boxes_[k], q );
        features[k] = std::pow( std::exp( -dist/factor_distance ), factor_height );
        // cout << "features[" << k << "] = " << features[k] << endl;
    }

    //    cout << "features.norm() : " << features.norm() << endl;

    return features;
}

void Squares::draw()
{
    // cout << __PRETTY_FUNCTION__ << endl;

    for( int i=0; i<int(boxes_.size()); i++ )
    {
        boxes_[i]->draw();
    }
}

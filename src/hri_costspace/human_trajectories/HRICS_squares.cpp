#include "HRICS_squares.hpp"

#include "HRICS_GestParameters.hpp"
#include "HRICS_parameters.hpp"

#include "API/Graphic/drawModule.hpp"
#include "API/project.hpp"
#include "planner/cost_space.hpp"

#include <boost/bind.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "Graphic-pkg.h"

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
        cout << "add cost functions : " << "costSquares" << endl;
        global_PlanarCostFct = global_SquareCostFct;
        global_costSpace->addCost( "costSquares", boost::bind( &Squares::cost, global_SquareCostFct, _1) );
        global_costSpace->addCost( "costSquaresJacobian", boost::bind( &Squares::jacobianCost, global_SquareCostFct, _1) );
        // global_costSpace->setCost( "costSquares" );
    }
    else{
        delete global_SquareCostFct;
        global_SquareCostFct = NULL;
    }
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

void Square::draw()
{
    g3d_draw_simple_box( center_[0]-x_, center_[0]+x_, center_[1]-y_, center_[1]+y_, 0.0, 0.0, 0, 0, 1.0);
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Squares::Squares()
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
    active_dofs_[1] = 6;
}

FeatureVect Squares::getFeatures( const Configuration& q )
{
    FeatureVect features(Eigen::VectorXd::Zero(centers_.size()));

    robot_->setAndUpdate(q);

    // The factor distance when larger
    const double factor_distance = 10.0;
    const double factor_height = HriEnv->getDouble(HricsParam::ioc_spheres_power);

    for( int i=0; i< int(active_features_.size()); i++ )
    {
        int k = active_features_[i];
        double dist = distToSquare( squares_[k], q );
        features[k] = pow( exp( -dist/factor_distance ), factor_height );
        cout << "features[" << k << "] = " << features[k] << endl;
    }

//    cout << "features.norm() : " << features.norm() << endl;

    return features;
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

double Squares::jacobianCost(const Configuration& q)
{
//    return 1 / Feature::getFeaturesJacobianMagnitude( q );
    return exp(-10*Feature::getFeaturesJacobianMagnitude( q )); // 10 is for scaling TODO findout what to put here
}

void Squares::computeSize()
{
    cout << "compute sizes" << endl;

    squares_.clear();

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
        Eigen::Vector2d center;
        center[0] = p[0];
        center[1] = p[1];

        squares_.push_back( Square( center, o->pol[0]->primitive_data->x_length/2, o->pol[0]->primitive_data->y_length/2 ) );

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

    corners[0][0] = square.center_[0] + square.x_;
    corners[0][1] = square.center_[1] + square.y_;

    corners[1][0] = square.center_[0] - square.x_;
    corners[1][1] = square.center_[1] + square.y_;

    corners[2][0] = square.center_[0] + square.x_;
    corners[2][1] = square.center_[1] - square.y_;

    corners[3][0] = square.center_[0] - square.x_;
    corners[3][1] = square.center_[1] - square.y_;

    if( isInAASquare( corners, p ) )
    {
        return 0.0;
    }
    else {
        std::vector<double> distances(4);
        Eigen::Vector2d closestPoint; // TODO use results from is in square
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
double Squares::pointToLineSegmentDistance(const Eigen::Vector2d& p, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, Eigen::Vector2d& closestPoint)
{
    double alpha, d1, d2, norm_p1p2;
    Eigen::Vector2d p1p2, p1p2_n, p1p, proj;

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

void Squares::draw()
{
    for( int i=0; i<int(squares_.size()); i++ )
    {
        squares_[i].draw();
    }
}

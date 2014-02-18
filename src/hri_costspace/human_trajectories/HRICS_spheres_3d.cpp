#include "HRICS_spheres_3d.hpp"

#include "HRICS_GestParameters.hpp"
#include "HRICS_parameters.hpp"

#include "API/misc_functions.hpp"
#include "API/project.hpp"
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

HRICS::Spheres3D* global_Spheres3DCostFct=NULL;

bool HRICS_init_shperes_3d_cost()
{
    cout << "Initializing square cost" << endl;

    global_Spheres3DCostFct = new Spheres3D();
    global_Spheres3DCostFct->initialize();

    if( global_Spheres3DCostFct->getNumberOfFeatures() > 0 )
    {
        cout << "add cost functions : " << "costSpheres3D" << endl;
        global_costSpace->addCost( "costSpheres3D", boost::bind( &Spheres3D::cost, global_Spheres3DCostFct, _1) );
        global_costSpace->addCost( "costSpheres3DJacobian", boost::bind( &Spheres3D::jacobianCost, global_Spheres3DCostFct, _1) );
        // global_costSpace->setCost( "costSpheres3D" );
        return true;
    }
    else{
        delete global_Spheres3DCostFct;
        global_Spheres3DCostFct = NULL;
        return false;
    }
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

void Sphere::draw() const
{
//    cout << "draw sphere at : " <<
    g3d_set_color( Any, (double *) color_ );
    g3d_draw_solid_sphere( center_[0], center_[1], center_[2], 0.02 , 20 );
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Spheres3D::Spheres3D()
{
    // Uncomment to draw squares
    if( global_DrawModule )
    {
        global_DrawModule->addDrawFunction( "Spheres3D", boost::bind( &Spheres3D::draw, this) );
        global_DrawModule->enableDrawFunction( "Spheres3D" );
    }
    else{
        cout << "Draw module not initialized" << endl;
    }
}

Spheres3D::~Spheres3D()
{
    // Uncomment to draw squares
    if( global_DrawModule )
    {
        global_DrawModule->deleteDrawFunction( "Spheres3D" );
    }

    delete sampler_;
}

void Spheres3D::initialize()
{
    cout << "--------------------------------"  << endl;
    cout << "INIT BOXES" << endl;

    Scene* sce = global_Project->getActiveScene();

    robot_ = sce->getActiveRobot();
    if( robot_ == NULL ){
        return;
    }

    centers_.clear();
    int nb_spheres = addCenters("SPHERE");
    if( nb_spheres == 0 ){
        return;
    }
    computeSize();

    cout << "nb_spheres : " << nb_spheres << endl;

    // SET WEIGHTS AND ACTIVATE FEATURES
    w_.resize( nb_spheres );
    if( nb_spheres == 3 )
    {
        w_[0] = 100;  w_[1] = 100;  w_[2] = 100;
    }

    w_ /= w_.maxCoeff();
    setWeights( w_ );
    cout << "w_ : " << w_.transpose() << endl;

    // SET ACTIVE JOINTS AND BUILD COLLISION POINTS
    active_joints_.resize(7); // Arm
    active_joints_[0] = 3;
    active_joints_[1] = 4;
    active_joints_[2] = 5;
    active_joints_[3] = 6;
    active_joints_[4] = 7;
    active_joints_[5] = 8;
    active_joints_[6] = 9;

    active_dofs_ = robot_->getActiveDoFsFromJoints( active_joints_ );

    // Get all joints active in the motion planning
    // and compute bounding cylinders
    std::vector<Joint*> joints;
    for (size_t i=0; i<active_joints_.size(); i++)
    {
        joints.push_back( robot_->getJoint( active_joints_[i] ) );
        cout << "Add joint : " << joints[i]->getName() << endl;
    }

    // Generate collision points
    sampler_ = new BodySurfaceSampler(0.1);

    sampler_->generateRobotBoudingCylinder( robot_, joints );

    for ( int id=0; id<int(active_joints_.size()); id++ )
        sampler_->generateJointCollisionPoints( robot_, id, active_joints_, active_joints_ );
}

FeatureVect Spheres3D::getFeatures( const Configuration& q )
{
    FeatureVect features(Eigen::VectorXd::Zero(centers_.size()));

    robot_->setAndUpdate(q);

    // The factor distance when larger
    const double factor_distance = 10.0; // 10 -> close 0 when distance is 0.50 m
    const double factor_height = HriEnv->getDouble(HricsParam::ioc_spheres_power);
    const double factor_total = HriEnv->getDouble(HricsParam::ioc_cost_factor);

    for( int i=0; i< int(active_features_.size()); i++ )
    {
        int k = active_features_[i];
//        int k = 0; int i=0;
        double dist = distToShpere( static_cast<const Sphere&>(*spheres_[k]) );
        features[i] = factor_total * pow( exp( - dist * factor_distance ), factor_height ); // replace 0 by i
        // cout << "features[" << k << "] = " << features[k] << endl;
    }

    // cout << "features.norm() : " << features.norm() << endl;

    return features;
}

double Spheres3D::getFeaturesJacobianMagnitude( const Configuration& q )
{
    FeatureJacobian J = getFeaturesJacobian( q );
    double magnitude = ( std::abs(J.maxCoeff()) + std::abs(J.minCoeff()) ) / 2;

    // magnitude = 1 / magnitude;
    // cout << J << endl;
    // cout << magnitude << endl;
    // Maybe average the min and max coefficient
    return magnitude;
}

double Spheres3D::jacobianCost(const Configuration& q)
{
//    return 1 / Feature::getFeaturesJacobianMagnitude( q );
    return exp(-10*Feature::getFeaturesJacobianMagnitude( q )); // 10 is for scaling TODO findout what to put here
}

void Spheres3D::computeSize()
{
    cout << "compute sizes" << endl;

    spheres_.clear();

    for( int i=0; i< int(centers_.size()); i++ )
    {
        p3d_obj* o = p3d_get_robot_body_by_name( centers_[i]->getRobotStruct(), "body" );
//        cout << o->name << " : " << o->np << " , ";
//        for(int j=0;j<o->np;j++)
//            cout << o->pol[j]->entity_type << " , ";

        // Set sphere position and radius
        spheres_.push_back( new Sphere( centers_[i]->getJoint(1)->getVectorPos(), o->pol[0]->primitive_data->radius ) );

        // Set sphere center color
        GroundColorMixGreenToRed( spheres_.back()->color_, 1-double(i)/double(centers_.size()) );

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

double Spheres3D::distToShpere( const Sphere& sph )
{
    double min_dist = std::numeric_limits<double>::max();

    for( size_t i=0; i<active_joints_.size(); i++ )
    {
        Joint* jnt = robot_->getJoint( active_joints_[i] );
        std::vector<CollisionPoint>& points = sampler_->getCollisionPoints(jnt);
        Eigen::Transform3d T = jnt->getMatrixPos();

         for( size_t j=0; j<points.size(); j++ )
        {
            double dist_to_sphere = ( T*points[j].getPosition() - sph.center_ ).norm() - points[j].getRadius() - sph.raduis_;
            // cout << "point[" << i << "][" << j << "] = " << dist_to_sphere << endl;

            if( dist_to_sphere < min_dist )
            {
                min_dist = dist_to_sphere;
            }
        }
    }

    if( min_dist < 0.0 )
        return 0.0;

    return min_dist;
}

void Spheres3D::drawCollisionPoints()
{
    for( size_t i=0; i<active_joints_.size(); i++ )
    {
        Joint* jnt = robot_->getJoint( active_joints_[i] );
        std::vector<CollisionPoint>& points = sampler_->getCollisionPoints(jnt);
        Eigen::Transform3d T = jnt->getMatrixPos();

        // cout << "joint : " << active_joints_[i] << " , points size : " << points.size() << endl;

        for( size_t j=0; j<points.size(); j++ )
        {
            if( points[j].m_is_colliding )
            {
                double color[4];

                color[0] = 1.0;       // (r)ed
                color[1] = 0.0;       // (g)reen
                color[2] = 0.0;       // (b)lue
                color[3] = 0.7;       // transparency

                g3d_set_color(Any,color);
            }

            bool yellow = true;
            //bool yellow = (!points[j].m_is_colliding);

            points[j].draw( T, yellow );
        }
    }
}

void Spheres3D::draw()
{
    // cout << __PRETTY_FUNCTION__ << endl;
    for( int i=0; i<int(spheres_.size()); i++ )
    {
        static_cast<const Sphere*>(spheres_[i])->draw();
    }

    drawCollisionPoints();
}

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

using namespace Move3D;
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

Spheres3D::Spheres3D() : sampler_(NULL)
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
    cout << "INIT SPHERE3D" << endl;

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

    cout << "nb_spheres : " << nb_spheres << endl;

    // SET WEIGHTS AND ACTIVATE FEATURES
    w_.resize( nb_spheres );

    if( nb_spheres == 3 )
    {
        w_[0] = 100;  w_[1] = 100;  w_[2] = 100;
    }
    if( nb_spheres == 27 )
    {
        placeCenterGrid(false);

        w_[0] = 100;   w_[1] = 100;   w_[2] = 100; // BOTTOM
        w_[3] = 005;   w_[4] = 001;   w_[5] = 100;
        w_[6] = 010;   w_[7] = 005;   w_[8] = 020;

        w_[9] = 100;  w_[10] = 100;  w_[11] = 100; // MIDDLE
        w_[12] = 80;   w_[13] = 001;  w_[14] = 001;
        w_[15] = 005;  w_[16] = 005;  w_[17] = 020;

        w_[18] = 100;  w_[19] = 100;  w_[20] = 80; // TOP
        w_[21] = 001;  w_[22] = 80;   w_[23] = 001;
        w_[24] = 001;  w_[25] = 100;  w_[26] = 100;
    }

    w_ /= w_.maxCoeff();
    setWeights( w_ );
    cout << "w_ : " << w_.transpose() << endl;

    // SET RADIUS AND CENTERS
    computeSize();

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

FeatureVect Spheres3D::getFeatures( const Configuration& q, std::vector<int> active_dofs )
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
        p3d_obj* o = p3d_get_robot_body_by_name( centers_[i]->getP3dRobotStruct(), "body" );
        //        cout << o->name << " : " << o->np << " , ";
        //        for(int j=0;j<o->np;j++)
        //            cout << o->pol[j]->entity_type << " , ";

        // Set sphere position and radius
        spheres_.push_back( new Sphere( centers_[i]->getJoint(1)->getVectorPos(), o->pol[0]->primitive_data->radius ) );

        // Set spheres center color
        // low ids are green, high ids are red
        // GroundColorMixGreenToRed( spheres_.back()->color_, double(i)/double(centers_.size()) );
        GroundColorMixGreenToRed( spheres_.back()->color_, w_[i] );

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

void Spheres3D::placeCenterGrid(bool on_wall)
{
    std::vector<double> bounds = global_Project->getActiveScene()->getBounds();

    double x_max = bounds[1], y_max = bounds[3], z_max = bounds[5];
    double x_min = bounds[0], y_min = bounds[2], z_min = bounds[4];

    cout << "x_max : " << x_max << " , y_max : " << y_max << " , z_max : " << z_max << endl;
    cout << "x_min : " << x_min << " , y_min : " << y_min << " , z_min : " << z_min << endl;

    int nb_cells = std::pow( double(centers_.size()), 1/3. );
    int offset = on_wall ? 0 : 1 ;

    cout << "nb_cells : " << nb_cells << endl;

    for( int i=offset; i<nb_cells+offset; i++ )
    {
        for( int j=offset; j<nb_cells+offset; j++ )
        {
            for( int k=offset; k<nb_cells+offset; k++ )
            {
                // The id computation is switched for conveinence when looking
                // at the scenario from above
                int id = (k-offset)*std::pow(nb_cells,2) + (j-offset)*(nb_cells) + (nb_cells-i); // TODO fix offset

                confPtr_t q = centers_[id]->getCurrentPos();

                int divisions = nb_cells-1;
                if( !on_wall )
                    divisions = nb_cells+1;

                (*q)[6] = x_min + double(i)*(x_max-x_min)/double(divisions);
                (*q)[7] = y_min + double(j)*(y_max-y_min)/double(divisions);
                (*q)[8] = z_min + double(k)*(z_max-z_min)/double(divisions);

                centers_[id]->setAndUpdate( *q );
                centers_[id]->setInitPos( *q );

                cout.precision(3);
                cout << "c (" << id << ") : " << (*q)[6] << " , " << (*q)[7] << " , " << (*q)[8] << endl;
            }
        }
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

void Spheres3D::setSphereToDraw(int id, bool enable)
{
    p3d_obj* o = p3d_get_robot_body_by_name( centers_[id]->getP3dRobotStruct(), "body" );
    if( o == NULL ) {
        cout << "Could not get center : " << id << " , with name body" << endl;
        return;
    }

    if( o->np >= 1 ) {
        if( enable )
            o->pol[0]->color_vect[3] = 0.2;
        else
            o->pol[0]->color_vect[3] = 0.0;
    }
    else {
        cout << "Could not set color of body : " << o->name << endl;
    }
}

void Spheres3D::setShperesToDraw()
{
    if( centers_.size() != 27 )
        return;

    for( size_t i=0;i<centers_.size();i++)
        setSphereToDraw( i, false );

    sphere_to_draw_.clear();

    switch( HriEnv->getInt(HricsParam::ioc_spheres_to_draw) )
    {
    case 0:
        for( size_t i=0; i<9; i++)
            sphere_to_draw_.push_back( i );
        break;
    case 1:
        for( size_t i=0; i<9; i++)
            sphere_to_draw_.push_back( i+9 );
        break;
    case 2:
        for( size_t i=0; i<9; i++)
            sphere_to_draw_.push_back( i+18 );
        break;
    case -1:
        break;
    default:
        for( size_t i=0; i<centers_.size(); i++)
            sphere_to_draw_.push_back(i);
        break;
    }

    for( size_t i=0;i<sphere_to_draw_.size();i++)
        setSphereToDraw( sphere_to_draw_[i], true );
}

void Spheres3D::draw()
{
    setShperesToDraw();

    // cout << __PRETTY_FUNCTION__ << endl;
    for( size_t i=0; i<sphere_to_draw_.size(); i++ )
    {
        static_cast<const Sphere*>(spheres_[sphere_to_draw_[i]])->draw();
    }

    drawCollisionPoints();
}

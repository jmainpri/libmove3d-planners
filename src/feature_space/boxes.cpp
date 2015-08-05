/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#include "boxes.hpp"

#include "HRICS_parameters.hpp"

#include "API/project.hpp"
#include "API/Graphic/drawModule.hpp"

#include "utils/misc_functions.hpp"

#include "planner/cost_space.hpp"

#include <boost/bind.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>

// Included for random number
#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Graphic-pkg.h>

using namespace Move3D;
using std::cout;
using std::endl;

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Move3D::Boxes* global_BoxesCostFct=NULL;

bool HRICS_init_boxes_cost()
{
    cout << "Initializing square cost" << endl;

    global_BoxesCostFct = new Boxes();
    global_BoxesCostFct->initialize();

    if( global_BoxesCostFct->getNumberOfFeatures() > 0 )
    {
        cout << "add cost functions : "
             << "costBoxes" << endl;

        global_costSpace->addCost( "costBoxes", boost::bind( &Boxes::cost, global_BoxesCostFct, _1) );
        global_costSpace->addCost( "costBoxesJacobian", boost::bind( &Boxes::jacobianCost, global_BoxesCostFct, _1) );
        // global_costSpace->setCost( "costSquares" );
        return true;
    }
    else{
        delete global_BoxesCostFct;
        global_BoxesCostFct = NULL;
        return false;
    }
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

void Box::draw() const
{
//    cout << __PRETTY_FUNCTION__ << endl;
//    cout << "draw box " << endl;
//    cout << center_.transpose() << endl;
//    cout << size_.transpose() << endl;
    g3d_draw_simple_box( center_[0]-size_[0], center_[0]+size_[0],
                         center_[1]-size_[1], center_[1]+size_[1],
                         center_[2]-size_[2], center_[2]+size_[2],  0, 0, 1.0);
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Boxes::Boxes() : sampler_(NULL)
{
    // Uncomment to draw squares
    if( global_DrawModule )
    {
        global_DrawModule->addDrawFunction( "Boxes", boost::bind( &Boxes::draw, this) );
        global_DrawModule->enableDrawFunction( "Boxes" );
    }
    else{
        cout << "Draw module not initialized" << endl;
    }
}

Boxes::~Boxes()
{
    // Uncomment to draw squares
    if( global_DrawModule )
    {
        global_DrawModule->deleteDrawFunction( "Boxes" );
    }

    delete sampler_;
}

void Boxes::initialize()
{
    cout << "--------------------------------"  << endl;
    cout << "INIT BOXES" << endl;

    Scene* sce = global_Project->getActiveScene();

    robot_ = sce->getActiveRobot();
    if( robot_ == NULL ){
        return;
    }

    centers_.clear();

    int nb_boxes = addCenters("BOX");
    if( nb_boxes == 0 ){
        return;
    }

    w_.resize( centers_.size() );

//    int i=0;

    cout << "nb_boxes : " << nb_boxes << endl;

    computeSize();

//    if( nb_boxes == 16 )
//    {
//        placeCenterGrid( true );
//        computeSize();
//        w_[i++] = 100;  w_[i++] = 100;  w_[i++] = 100; w_[i++] = 100;
//        w_[i++] = 100;  w_[i++] = 50;   w_[i++] = 8;   w_[i++] = 100;
//        w_[i++] = 100;  w_[i++] = 30;   w_[i++] = 50;  w_[i++] = 100;
//        w_[i++] = 100;  w_[i++] = 100;  w_[i++] = 100; w_[i++] = 100;
//    }

    double max = w_.maxCoeff();
    w_ /= max;

    cout << "w_ : " << w_.transpose() << endl;

    active_joints_.resize(7); // Arm
    active_joints_[0] = 1;
    active_joints_[1] = 2;
    active_joints_[2] = 3;
    active_joints_[3] = 4;
    active_joints_[4] = 5;
    active_joints_[5] = 6;
    active_joints_[6] = 7;

    active_dofs_ = robot_->getActiveDoFsFromJoints( active_joints_ );

    // Get all joints active in the motion planning
    // and compute bounding cylinders
    std::vector<Joint*> joints;
    for (size_t i=0; i<active_joints_.size(); i++)
        joints.push_back( robot_->getJoint( active_joints_[i] ) );

    // Generate collision points
    sampler_ = new BodySurfaceSampler(0.1, 0.05);
    sampler_->generateRobotBoudingCylinder( robot_, joints );
    sampler_->generateRobotCollisionPoints( robot_,
                                            active_joints_,
                                            active_joints_ );
}


FeatureVect Boxes::getFeatures( const Configuration& q, std::vector<int> active_dofs )
{
    FeatureVect features(Eigen::VectorXd::Zero(centers_.size()));

    robot_->setAndUpdate(q);

    // The factor distance when larger
    const double factor_distance = 10.0;
    const double factor_height = HriEnv->getDouble(HricsParam::ioc_spheres_power);

    for( int i=0; i< int(active_features_.size()); i++ )
    {
        int k = active_features_[i];
        double dist = distToBox( static_cast<const Box&>(*boxes_[k]), q );
        features[k] = pow( exp( -dist/factor_distance ), factor_height );
        // cout << "features[" << k << "] = " << features[k] << endl;
    }

//    cout << "features.norm() : " << features.norm() << endl;

    return features;
}

double Boxes::getFeaturesJacobianMagnitude( const Configuration& q )
{
    FeatureJacobian J = getFeaturesJacobian( q );
    double magnitude = ( std::abs(J.maxCoeff()) + std::abs(J.minCoeff()) ) / 2;

    // magnitude = 1 / magnitude;
    // cout << J << endl;
    // cout << magnitude << endl;
    // Maybe average the min and max coefficient
    return magnitude;
}

double Boxes::jacobianCost(const Configuration& q)
{
//    return 1 / Feature::getFeaturesJacobianMagnitude( q );
    return exp(-10*Feature::getFeaturesJacobianMagnitude( q )); // 10 is for scaling TODO findout what to put here
}

void Boxes::computeSize()
{
    cout << "compute sizes" << endl;

    boxes_.clear();

    for( int i=0; i< int(centers_.size()); i++ )
    {
        p3d_obj* o = p3d_get_robot_body_by_name( centers_[i]->getP3dRobotStruct(), "body" );
//        cout << o->name << " : " << o->np << " , ";
//        for(int j=0;j<o->np;j++)
//            cout << o->pol[j]->entity_type << " , ";

//        cout << o->name ;
//        cout << "( " ;
//        cout << o->pol[0]->primitive_data->x_length << " , ";
//        cout << o->pol[0]->primitive_data->y_length ; // << " , ";
//        cout << o->pol[0]->primitive_data->z_length ;
//        cout << " )" ;

        Eigen::Vector3d center = centers_[i]->getJoint(1)->getVectorPos();

        Eigen::VectorXd size(3);
        size[0] = o->pol[0]->primitive_data->x_length/2;
        size[1] = o->pol[0]->primitive_data->y_length/2;
        size[2] = o->pol[0]->primitive_data->z_length/2;

        boxes_.push_back( new Box( center, size ) );

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

// 0 -> +x
// 1 -> -x
// 2 -> +y
// 3 -> -y
// 4 -> +z
// 5 -> -z
bool Boxes::isInAABox( const Eigen::VectorXd& limits, Eigen::Vector3d p )
{
    if( p[0] > limits[0] )
        return false;
    if( p[0] < limits[1] )
        return false;
    if( p[1] > limits[2] )
        return false;
    if( p[1] < limits[3] )
        return false;
    if( p[2] > limits[4] )
        return false;
    if( p[2] < limits[5] )
        return false;

    return true;
}

double Boxes::distToBox( const Box& box, const Configuration& q )
{
    Eigen::VectorXd limits(6);
    limits[0] = box.center_[0] + box.size_[0];
    limits[1] = box.center_[0] - box.size_[0];
    limits[2] = box.center_[1] + box.size_[1];
    limits[3] = box.center_[1] - box.size_[1];
    limits[4] = box.center_[2] + box.size_[2];
    limits[5] = box.center_[2] - box.size_[2];

    Eigen::Vector3d p = q.getEigenVector(6,8);

    if( isInAABox( limits, p ) )
    {
        return 0.0; // TODO set fixed cost
    }
    else {
        std::vector<Eigen::Vector3d> corners(8);

        //           V1 -- V0
        //          /      / |
        // z  y    V4 -- V2 V3
        // |/      |      | /
        // 0--x    V7 -- V5

        corners[0][0] = box.center_[0] + box.size_[0];
        corners[0][1] = box.center_[1] + box.size_[1];
        corners[0][2] = box.center_[2] + box.size_[1];

        corners[1][0] = box.center_[0] - box.size_[0];
        corners[1][1] = box.center_[1] + box.size_[1];
        corners[1][2] = box.center_[2] + box.size_[2];

        corners[2][0] = box.center_[0] + box.size_[0];
        corners[2][1] = box.center_[1] - box.size_[1];
        corners[2][2] = box.center_[2] + box.size_[2];

        corners[3][0] = box.center_[0] + box.size_[0];
        corners[3][1] = box.center_[1] + box.size_[1];
        corners[3][2] = box.center_[2] - box.size_[2];

        corners[4][0] = box.center_[0] - box.size_[0];
        corners[4][1] = box.center_[1] - box.size_[1];
        corners[4][2] = box.center_[2] + box.size_[2];

        corners[5][0] = box.center_[0] + box.size_[0];
        corners[5][1] = box.center_[1] - box.size_[1];
        corners[5][2] = box.center_[2] - box.size_[2];

        corners[6][0] = box.center_[0] - box.size_[0];
        corners[6][1] = box.center_[1] + box.size_[1];
        corners[6][2] = box.center_[2] - box.size_[2];

        corners[7][0] = box.center_[0] - box.size_[0];
        corners[7][1] = box.center_[1] - box.size_[1];
        corners[7][2] = box.center_[2] - box.size_[2];

        std::vector<double> distances(12);
        Eigen::VectorXd closestPoint;
        distances[0]   = pointToLineSegmentDistance( p, corners[1], corners[0], closestPoint );
        distances[1]   = pointToLineSegmentDistance( p, corners[2], corners[0], closestPoint );
        distances[2]   = pointToLineSegmentDistance( p, corners[3], corners[0], closestPoint );
        distances[3]   = pointToLineSegmentDistance( p, corners[4], corners[2], closestPoint );
        distances[4]   = pointToLineSegmentDistance( p, corners[4], corners[1], closestPoint );
        distances[5]   = pointToLineSegmentDistance( p, corners[5], corners[2], closestPoint );
        distances[6]   = pointToLineSegmentDistance( p, corners[5], corners[3], closestPoint );
        distances[7]   = pointToLineSegmentDistance( p, corners[6], corners[3], closestPoint );
        distances[8]   = pointToLineSegmentDistance( p, corners[6], corners[4], closestPoint );
        distances[9]   = pointToLineSegmentDistance( p, corners[7], corners[4], closestPoint );
        distances[10]  = pointToLineSegmentDistance( p, corners[7], corners[5], closestPoint );
        distances[11]  = pointToLineSegmentDistance( p, corners[7], corners[6], closestPoint );
        return *std::min_element( distances.begin(), distances.end() );
    }
}

void Boxes::drawCollisionPoints()
{
    for( size_t i=0; i<active_joints_.size(); i++ )
    {
        Joint* jnt = robot_->getJoint( active_joints_[i] );
        std::vector<CollisionPoint>& points = sampler_->getCollisionPoints(jnt);
        Eigen::Transform3d T = jnt->getMatrixPos();

        cout << "joint : " << active_joints_[i] << " , points size : " << points.size() << endl;

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

void Boxes::draw()
{
    cout << __PRETTY_FUNCTION__ << endl;
    for( int i=0; i<int(boxes_.size()); i++ )
    {
        static_cast<const Box*>(boxes_[i])->draw();
    }

    drawCollisionPoints();
}


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
#include "spheres.hpp"

#include "HRICS_parameters.hpp"

#include "API/project.hpp"
#include "planner/cost_space.hpp"

#include <libmove3d/include/P3d-pkg.h>

#include <boost/bind.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>

using namespace Move3D;

using std::cout;
using std::endl;

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Spheres* global_SphereCostFct=NULL;

bool HRICS_init_sphere_cost()
{
    cout << "Initializing sphere cost" << endl;

    global_SphereCostFct = new Spheres();
    global_SphereCostFct->initialize();

    if( global_SphereCostFct->getNumberOfFeatures() > 0 )
    {
        cout << "add cost functions : " << "costSpheres" << endl;
        global_PlanarCostFct = global_SphereCostFct;

        global_costSpace->addCost( "costSpheres", boost::bind( &Spheres::cost, global_SphereCostFct, _1) );
        global_costSpace->addPathCost( "costSpheres", boost::bind( &Feature::costPath, global_SphereCostFct, _1, _2) );

        global_costSpace->addCost( "costSpheresJacobian", boost::bind( &PlanarFeature::jacobianCost, global_SphereCostFct, _1) );

        API_activeFeatureSpace = global_SphereCostFct;

        // global_costSpace->setCost( cost_function );
        return true;
    }
    else{
        delete global_SphereCostFct;
        global_SphereCostFct = NULL;
        return false;
    }
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Spheres::Spheres() : PlanarFeature()
{

}

void Spheres::initialize()
{
    cout << "--------------------------------"  << endl;
    cout << "INIT SPHERE 2D" << endl;

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

    if( nb_spheres == 3 )
    {
//        Configuration q1(*centers_[0]->getCurrentPos());
//        Configuration q2(*centers_[1]->getCurrentPos());
//        Configuration q3(*centers_[2]->getCurrentPos());

//        q1[6] = -0.5; q2[6] = 0; q3[6] = 0.5;
//        q1[7] = 0.5;  q2[7] = 0; q3[7] = 0.5;

//        centers_[0]->setAndUpdate( q1 );
//        centers_[1]->setAndUpdate( q2 );
//        centers_[2]->setAndUpdate( q3 );

        w_[i++] = 3;
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

    setWeights( w_ ); // sets active featues

    nb_dofs_ = 2;

    // Set active dofs
    active_dofs_.resize(2);
    active_dofs_[0] = 6;
    active_dofs_[1] = 7;

    // Set max feature value taken inside the spheres
    max_value_ = 1e7;

    // Set radius of spheres
    radius_ = 1;

    if( !centers_.empty() ) // Set the radius
    {
        p3d_obj* o = p3d_get_robot_body_by_name( centers_[0]->getP3dRobotStruct(), "body" );
        if( o == NULL ) {
            cout << "Could not get center : " << i << " , with name body" << endl;
        }
        else if( o->np >= 1 ) {
            radius_ = o->pol[0]->primitive_data->radius;
        }
    }
}

FeatureVect Spheres::getFeatures( const Configuration& q, std::vector<int> active_features )
{
    const std::vector<int>& features = active_features.empty() ? active_features_ : active_features ;

    FeatureVect phi(Eigen::VectorXd::Zero(centers_.size()));
    Eigen::VectorXd x = q.getEigenVector(6,7);

    // The factor distance when larger
    const double factor_distance = 0.3; // 10
    const double factor_height = HriEnv->getDouble(HricsParam::ioc_spheres_power);

    for( int i=0; i< int(features.size()); i++ )
    {
        int k = features[i];
        Eigen::VectorXd mu = centers_[k]->getCurrentPos()->getEigenVector(6,7);
        double dist = ( x - mu ).norm();
        // phi[k] = dist > radius_ ? std::pow( std::exp( -dist/factor_distance ), factor_height ) : max_value_;
        phi[k] = std::pow( std::exp( -dist/factor_distance ), factor_height );
    }

   // cout << "phi : " << phi.transpose() << endl;

    return phi;
}

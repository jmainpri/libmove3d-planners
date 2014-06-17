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
#include "clearance.hpp"

#include "API/project.hpp"
#include "utils/misc_functions.hpp"
#include "collision_space/collision_space.hpp"
#include "planner/cost_space.hpp"
#include "planner/planEnvironment.hpp"

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

Move3D::Clearance* global_ClearanceCostFct=NULL;

bool FEATURES_init_Clearance_cost()
{
    cout << "------------------------------------------------" << endl;
    cout << "------------------------------------------------" << endl;
    cout << "------------------------------------------------" << endl;
    cout << "Initializing clearance cost" << endl;

    global_ClearanceCostFct = new Clearance();
    global_ClearanceCostFct->initialize();

    if( global_ClearanceCostFct->getNumberOfFeatures() > 0 )
    {
        global_activeFeatureFunction = global_ClearanceCostFct;

        cout << "add cost functions : " << "costClearance" << endl;

        global_costSpace->addCost( "costClearance", boost::bind( &Clearance::cost, global_ClearanceCostFct, _1) );
        global_costSpace->addPathCost( "costClearance", boost::bind( &Clearance::costPath, global_ClearanceCostFct, _1, _2) );

        global_costSpace->setCost( "costClearance" );

        return true;
    }
    else{
        delete global_ClearanceCostFct;
        global_ClearanceCostFct = NULL;
        return false;
    }
}

// ------------------------------------------------------
// ------------------------------------------------------
// ------------------------------------------------------

Clearance::Clearance() : Feature("Clearance")
{

}

Clearance::~Clearance()
{

}

void Clearance::initialize()
{
    w_.resize( 2 );
    w_[0] = 1.0;
    w_[1] = 1.0;
}

FeatureVect Clearance::getFeatures( const Configuration& q, std::vector<int> active_dofs )
{
    FeatureVect phi( FeatureVect::Zero( getNumberOfFeatures() ) );
    phi[0] = 0.0;
    phi[1] = global_collisionSpace->cost( q );
    return phi;
}

FeatureVect Clearance::getFeatureCount( Move3D::LocalPath& path, int& nb_calls )
{
    FeatureVect phi( FeatureVect::Zero( getNumberOfFeatures() ) );

    double t = 0.0;
    double t_max = path.getParamMax();
    double step = ENV.getDouble(Env::dmax)*PlanEnv->getDouble(PlanParam::costResolution);
    int n_step = int(t_max/step);

    confPtr_t q = path.configAtParam(0.0);
    FeatureVect feat1 = getFeatures( *q );

    nb_calls = n_step;

    for ( int i=0; i<n_step; i++ )
    {
        t += step;
        q = path.configAtParam(t);
        FeatureVect feat2 = getFeatures( *q );
        phi += ( (feat1 + feat2) / 2 ) * step;
        feat1 = feat2;
    }

    phi[0] = t_max;
    return phi;
}

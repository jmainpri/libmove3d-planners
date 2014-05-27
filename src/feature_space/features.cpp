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
#include "features.hpp"

#include "HRICS_gest_parameters.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/TrajectoryOptim/Stomp/covariant_trajectory_policy.hpp"
#include "planner/TrajectoryOptim/Chomp/chompTrajectory.hpp"

#include "utils/NumsAndStrings.hpp"
#include "utils/misc_functions.hpp"

#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/Graphic-pkg.h>

using namespace Move3D;

using std::cout;
using std::endl;

Move3D::Feature* API_activeFeatureSpace = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------

double Feature::cost( Configuration& q )
{
    FeatureVect phi = getFeatures( q );
    double cost = is_stacked_ ? this->getWeights().transpose()*phi : w_.transpose()*phi;

    if( GestEnv->getBool(GestParam::print_debug) )
    {
        WeightVect w = getWeights();
        cout << "phi : " << endl;
        cout << phi.transpose() << endl;
        cout << "w : " << endl;
        cout << w.transpose() << endl;
        cout << endl;
        cout << "cost : " << cost << endl;
        cout << "is_stacked_ : " << is_stacked_ << endl;
    }

//    cout << __PRETTY_FUNCTION__ << endl;

    return cost;
}

double Feature::costTraj( const Move3D::Trajectory& t )
{
    FeatureVect  phi = getFeatureCount(t);

    cout << w_.size() << endl;
    cout << phi.size() << endl;

    double cost = w_.transpose()*phi;
    // cout << " w_.transpose() : " << w_.transpose() << endl;
    // cout << " phi.transpose() : " << phi.transpose() << endl;
    // cout << "cost : " << cost << endl;
    return cost;
}

double Feature::costPath( Move3D::LocalPath& path, int& nb_calls )
{
    if( !path.areFeaturesEvaluated() ){
        FeatureVect phi = getFeatureCount( path, nb_calls );
        path.setFeatures( phi );
        double cost = w_.transpose()*phi;
        path.setCost( cost );
        return cost;
    }
    else {
        const FeatureVect& phi = path.getFeatures();
        double cost = w_.transpose()*phi;
        path.setCost( cost );
        return cost;
    }
}

//! The jacobian is of m by n -> nb features by active dofs
//! column per dof
//! row per feature
FeatureJacobian Feature::getFeaturesJacobian( const Configuration& q_0 )
{
    const double eps = 1e-3;

    FeatureVect f_0 = getFeatures( q_0 );
    FeatureJacobian J = Eigen::MatrixXd::Zero( getNumberOfFeatures(), active_dofs_.size() );

    for( size_t j=0;j<active_dofs_.size();j++) // For each colomn
    {
        int dof = active_dofs_[ j ];
        // cout << "dof : " << dof << endl;

        Configuration q_1 = q_0;
        q_1[dof] = q_0[ dof ] + eps;

        FeatureVect f_1 = getFeatures( q_1 );
        J.col(j) = (f_1 - f_0) / eps;
    }

//    cout << "J : " << endl << J.transpose() << std::scientific << endl;
    return J;
}

double Feature::getFeaturesJacobianMagnitude( const Configuration& q )
{
    FeatureJacobian J = getFeaturesJacobian( q );
    return J.norm();
}

double Feature::getJacobianSum( const Move3D::Trajectory& t )
{
    double sum=0.0;

    for( int i=0;i<t.getNbOfViaPoints(); i++ )
    {
        sum += getFeaturesJacobianMagnitude( *t[i] );
        // cout << "sum : " << sum << endl;
    }

    return sum;
}

FeatureVect Feature::getFeatureCount( const Move3D::Trajectory& traj )
{
    FeatureVect phi( FeatureVect::Zero( getNumberOfFeatures() ) );

//    for(int i=1;i<t.getNbOfViaPoints();i++)
//    {
//        confPtr_t q_1 = t[i-1];
//        confPtr_t q_2 = t[i];
//        Eigen::VectorXd pos1 = q_1->getEigenVector(6,7);
//        Eigen::VectorXd pos2 = q_2->getEigenVector(6,7);
//        double dist = ( pos1 - pos2 ).norm();
//        phi += getFeatures( *q_1 )*dist;
//    }

//    int i=0;
//    for(; t <= t_max; i++ )
//    {
//        confPtr_t q = traj.configAtParam( t );

//        double powerOnIntegral = 1.0;

//        phi += getFeatures( *q )*step;
//        t += step;
//    }

    double t = 0.0;
    double t_max = traj.getRangeMax();
    double step = ENV.getDouble(Env::dmax)*PlanEnv->getDouble(PlanParam::costResolution);
    int n_step = int(t_max/step);

    confPtr_t q = traj.configAtParam(0.0);
    FeatureVect feat1 = getFeatures( *q );

    for ( int i=0; i<n_step; i++ )
    {
        t += step;
        q = traj.configAtParam(t);
        FeatureVect feat2 = getFeatures( *q );
        phi += ( (feat1 + feat2) / 2 )  * step;
        feat1 = feat2;
    }
//    cout << "--------- Integral ----------------" << endl;
//    cout << "Range = " << t_max << endl;
//    cout << "step = " << step << endl;
//    cout << "n_step = " << n_step << endl;

    // cout << "ith computation for integral : " << i << endl;

    return phi;
}

FeatureVect Feature::getFeatureCount( Move3D::LocalPath& path, int& nb_calls )
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
        phi += ( (feat1 + feat2) * step / 2 )  ;
        feat1 = feat2;
    }

    return phi;
}

FeatureProfile Feature::getFeatureProfile( const Move3D::Trajectory& t )
{
    FeatureProfile p( FeatureProfile::Zero(t.getNbOfViaPoints()) );

    for(int i=0;i<p.size();i++)
    {
        p[i] = getFeatures( *t[i] ).norm();
    }

    return p;
}

FeatureJacobian Feature::getFeatureJacobian( const Move3D::Trajectory& t )
{
    FeatureJacobian p( Eigen::MatrixXd::Zero( t.getNbOfViaPoints() , getNumberOfFeatures() ) );

    for(int i=0;i<p.rows();i++) // number of via points
    {
        FeatureJacobian J = getFeaturesJacobian( *t[i] );

        for(int j=0;j<p.cols();j++) // number of features
        {
            p(i,j) = J.row( j ).norm();
        }
    }

    return p;
}

FeatureProfile Feature::getFeatureJacobianProfile( const Move3D::Trajectory& t )
{
    FeatureProfile p( Eigen::VectorXd::Zero(t.getNbOfViaPoints()) );

    for(int i=0;i<p.size();i++)
    {
        p[i] = getFeaturesJacobianMagnitude( *t[i] );
    }

    return p;
}

void Feature::setWeights( const WeightVect& w )
{
     w_ = w;

     active_features_.resize( w_.size() );
     for( int i=0;i<int(active_features_.size());i++)
         active_features_[i] = i;
}

std::vector<Move3D::Trajectory*> Feature::extractAllTrajectories( Move3D::Graph* g, confPtr_t q_init, confPtr_t q_goal, int nb_divisions  )
{
    std::vector<Move3D::Trajectory*> trajs;
    std::vector<FeatureVect> ws( pow( nb_divisions, w_.size() ), FeatureVect::Zero( w_.size() ) );

    double delta = 1 / double( nb_divisions-1 );

    cout << "Computing " << ws.size() << " trajectories !!!" << endl;

    for( size_t i=1; i< ws.size(); i ++ )
    {
        std::vector<int> num = move3d_change_basis( i, nb_divisions );
        if( int( num.size()) < w_.size() ) {
            cout << "Error in " << __PRETTY_FUNCTION__ << endl;
            cout << " num.size() : " << num.size() << endl;
            cout << " w_.size() : " << w_.size() << endl;
            return trajs;
        }

        for( int j=0; j< w_.size(); ++j )
            ws[ i ](j) = delta * num[ num.size()-j-1 ];
    }

    global_trajToDraw.clear();

    for( size_t i=1; i< ws.size(); ++i ) // 1 does not include [0,0,...,0]
    {
        cout.precision(2);
        cout << " w[" << i << "] : " ;
        // cout << "\t" << ws[i].transpose() ;
        for( int j=0; j< w_.size(); ++j ) cout << "\t" << ws[i][j] << " ";
        cout << endl;

        w_ = ws[i]; // Set the weight vector

        // Only process non zero queries
//        if( w_.minCoeff() == 0.0 )
//            continue;

        g->resetAllEdgesCost();
        // trajs.push_back(  );
        // trajs.back()->replaceP3dTraj();

        Move3D::Trajectory* traj = g->extractAStarShortestPathsTraj( q_init, q_goal );

        if( find( global_trajToDraw.begin(), global_trajToDraw.end(), *traj ) == global_trajToDraw.end() )
        {
            global_trajToDraw.push_back( *traj );
            global_trajToDraw.back().setUseContinuousColors( false );
            global_trajToDraw.back().setColor( 0 );

            // Color from blue to red
            // global_trajToDraw.back().setUseContinuousColors();
            // global_trajToDraw.back().setColor( double(i) / double(ws.size()-1) );
        }
        else {
            delete traj;
        }

//        g3d_draw_allwin_active();
    }

    cout << "global_trajToDraw.size() : " << global_trajToDraw.size() << endl;

    return trajs;
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------

StackedFeatures::StackedFeatures()
{
    is_stacked_ = true;
    nb_features_ = 0;
}

FeatureVect StackedFeatures::getFeatureCount(const Move3D::Trajectory& t)
{
    FeatureVect f = Eigen::VectorXd::Zero( nb_features_ );

    int height = 0;
    for( int i=0;i<int(feature_stack_.size()); i++)
    {
        FeatureVect fi = feature_stack_[i]->getFeatureCount(t);
        f.segment( height, fi.size() ) = fi;
        height += fi.size();
    }

    return f;
}

FeatureVect StackedFeatures::getFeatures(const Configuration& q, std::vector<int> active_dofs)
{
    FeatureVect f = Eigen::VectorXd::Zero( nb_features_ );

    int height = 0;
    for( int i=0;i<int(feature_stack_.size()); i++)
    {
        FeatureVect fi = feature_stack_[i]->getFeatures(q);
        f.segment( height, fi.size() ) = fi;
        height += fi.size();
    }

//    cout << "f : " << f.transpose() << endl;

    return f;
}

bool StackedFeatures::addFeatureFunction( Feature* fct )
{
    if( feature_stack_.empty() )
    {
        active_dofs_ = fct->getActiveDoFs();
    }
    else
    {
        for( int i=0;i<int(active_dofs_.size()); i++ )
        {
            if( active_dofs_[i] != fct->getActiveDoFs()[i] )
            {
                return false;
            }
        }
    }
    feature_stack_.push_back( fct );
    nb_features_ += feature_stack_.back()->getNumberOfFeatures();
    w_ = Eigen::VectorXd::Zero( nb_features_ );
    return true;
}

void StackedFeatures::setWeights( const WeightVect& w )
{
    int height = 0;
    for( int i=0;i<int(feature_stack_.size()); i++)
    {
        int num = feature_stack_[i]->getNumberOfFeatures();
        Eigen::VectorXd weights = w.segment( height, num );
        feature_stack_[i]->setWeights( weights );
        height += num;
    }

    w_= w;
}

WeightVect StackedFeatures::getWeights()
{
    WeightVect w = Eigen::VectorXd::Zero( nb_features_ );

    int height = 0;
    for( int i=0;i<int(feature_stack_.size()); i++)
    {
        WeightVect wi = feature_stack_[i]->getWeights();
        w.segment( height, wi.size() ) = wi;
        height += wi.size();
    }

    return w;
}

void StackedFeatures::setActiveFeatures( const std::vector<int>& active_features )
{
// TODO active feature acros stack
//    int height = 0;
//    for( int i=0;i<int(feature_stack_.size()); i++)
//    {
//        int num = feature_stack_[i]->getActiveFeatures().size();
//        Eigen::VectorXd weights = active_features.segment( height, num );
//        feature_stack_[i]->setWeights( weights );
//        height += num;
//    }

    if( feature_stack_.empty() )
        return;

    feature_stack_[0]->setActiveFeatures( active_features );
    active_features_= active_features;
}

void StackedFeatures::printWeights() const
{
    for(int i=0;i<int(feature_stack_.size());i++)
    {
        feature_stack_[i]->printWeights();
    }
}

void StackedFeatures::printStackInfo() const
{
    cout << "------------------------------" << endl;
    cout << "stack of features : nb of fct ( " << feature_stack_.size() << " )" << endl;

    for(int i=0;i<int(feature_stack_.size());i++)
    {
        WeightVect w = feature_stack_[i]->getWeights();

        cout << " -- feature fct " << i << " contains : " << w.size() << " features with ( active, weight ) :" ;
        cout << endl;

        for(int j=0;j<w.size();j++)
        {
            std::vector<int>::const_iterator it = find( feature_stack_[i]->getActiveFeatures().begin(),
                                                        feature_stack_[i]->getActiveFeatures().end(),
                                                        j );

            cout << "\t( " << ( it != feature_stack_[i]->getActiveFeatures().end() ) << " , " << w_[j] << " ) ; ";

            if( (j+1) % 5 == 0 )
                cout << endl;
        }

        cout << endl;
    }
    cout << "------------------------------" << endl;
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Smoothness cost

TrajectorySmoothness::TrajectorySmoothness()
{
    w_ = Eigen::VectorXd::Zero( 1 ); // Sets the number of feature in the base class
}

FeatureVect TrajectorySmoothness::getFeatureCount( const Move3D::Trajectory& t )
{
    FeatureVect f;

    int rows = active_dofs_.size();
    int cols = t.getNbOfViaPoints();

    int diff_rule_length = control_cost_.getDiffRuleLength();

    Eigen::MatrixXd mat( rows, cols + 2*(diff_rule_length-1) );

    Eigen::VectorXd q_init = t.getBegin()->getEigenVector( active_dofs_ );
    Eigen::VectorXd q_goal = t.getEnd()->getEigenVector( active_dofs_ );

    mat.block( 0, diff_rule_length-1, rows, cols ) = t.getEigenMatrix( active_dofs_ );

    // cout << mat << endl;

    control_cost_.fillTrajectory( q_init, q_goal, mat );
    std::vector<Eigen::VectorXd> control_cost = control_cost_.getSquaredQuantities( mat );

    // printControlCosts( control_cost );

    f = Eigen::VectorXd::Zero( 1 );
    f[0] = control_cost_.cost( control_cost );
    // cout << f << endl;
    return f;
}

void TrajectorySmoothness::printControlCosts( const std::vector<Eigen::VectorXd>& control_cost )
{
    if( control_cost.empty() )
        return;

    int diff_rule_length = control_cost_.getDiffRuleLength();
    int size = control_cost[0].size() - 2*(diff_rule_length-1);

    FeatureVect f_tmp = Eigen::VectorXd::Zero( size );

    for( int i=0; i<int(control_cost.size()); i++ )
    {
        f_tmp += control_cost[i].segment( diff_rule_length-1, size );
    }

    cout << f_tmp.transpose() << endl;
}

FeatureVect TrajectorySmoothness::getFeatures(const Configuration& q, std::vector<int> active_dofs)
{
    FeatureVect count;
    return count;
}

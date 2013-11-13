#include "HRICS_features.hpp"

using namespace HRICS;
using std::cout;
using std::endl;

//----------------------------------------------------------------------
//----------------------------------------------------------------------

double Feature::cost( Configuration& q )
{
    FeatureVect phi = getFeatures( q );
    double cost = w_.transpose()*phi;
    return cost;
}

double Feature::costTraj( const API::Trajectory& t )
{
    FeatureVect  phi = getFeatureCount(t);

    cout << w_.size() << endl;
    cout << phi.size() << endl;

    double cost = w_.transpose()*phi;
    cout << " w_.transpose() : " << w_.transpose() << endl;
    cout << " phi.transpose() : " << phi.transpose() << endl;
    cout << "cost : " << cost << endl;
    return cost;
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------

StackedFeatures::StackedFeatures()
{
    nb_features_ = 0;
}

FeatureVect StackedFeatures::getFeatureCount(const API::Trajectory& t)
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

FeatureVect StackedFeatures::getFeatures(const Configuration& q)
{
    FeatureVect f;
    return f;
}

void StackedFeatures::addFeatureFunction( Feature* fct )
{
    feature_stack_.push_back( fct );
    nb_features_ += feature_stack_.back()->getNumberOfFeatures();
    w_ = Eigen::VectorXd::Zero( nb_features_ );
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

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Smoothness cost

TrajectorySmoothness::TrajectorySmoothness()
{

}

FeatureVect TrajectorySmoothness::getFeatureCount( const API::Trajectory& t )
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

FeatureVect TrajectorySmoothness::getFeatures(const Configuration& q)
{
    FeatureVect count;
    return count;
}

void TrajectorySmoothness::setActiveDofs( const std::vector<int>& active_dofs )
{
    active_dofs_ = active_dofs;
    w_ = Eigen::VectorXd::Zero( 1 ); // Sets the number of feature in the base class
}


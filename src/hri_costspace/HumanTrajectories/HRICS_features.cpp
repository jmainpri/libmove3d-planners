#include "HRICS_features.hpp"

#include "HRICS_GestParameters.hpp"

using namespace HRICS;
using std::cout;
using std::endl;

//----------------------------------------------------------------------
//----------------------------------------------------------------------

double Feature::cost( Configuration& q )
{
    FeatureVect phi = getFeatures( q );
    double cost = w_.transpose()*phi;

    if( GestEnv->getBool(GestParam::print_debug) )
    {
        WeightVect w = getWeights();
        cout << "phi : " << endl;
        cout << phi.transpose() << endl;
        cout << "w : " << endl;
        cout << w.transpose() << endl;
        cout << endl;
    }

    return cost;
}

double Feature::costTraj( const API::Trajectory& t )
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

FeatureJacobian Feature::getFeaturesJacobian(const Configuration& q_0)
{
    const double eps = 1e-3;

    FeatureVect f_0 = getFeatures( q_0 );
    FeatureJacobian J = Eigen::MatrixXd::Zero( getNumberOfFeatures(), active_dofs_.size() );

    for( int i=0;i<int(active_dofs_.size());i++)
    {
        int dof = active_dofs_[ i ];

        Configuration q_1 = q_0;
        q_1[dof] = q_0[ dof ] + eps;

        FeatureVect f_1 = getFeatures( q_1 );
        J.col(i) = (f_1 - f_0) / eps;
    }

//    cout << "J : " << endl << J << std::scientific << endl;

    return J;
}

double Feature::getFeaturesJacobianMagnitude(const Configuration& q)
{
    FeatureJacobian J = getFeaturesJacobian(q);
    return J.norm();
}

void Feature::setActiveDofs( const std::vector<int>& active_dofs )
{
    active_dofs_ = active_dofs;
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

 void StackedFeatures::printWeights() const
 {
     for(int i=0;i<int(feature_stack_.size());i++)
     {
         feature_stack_[i]->printWeights();
     }
 }

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Smoothness cost

TrajectorySmoothness::TrajectorySmoothness()
{
     w_ = Eigen::VectorXd::Zero( 1 ); // Sets the number of feature in the base class
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

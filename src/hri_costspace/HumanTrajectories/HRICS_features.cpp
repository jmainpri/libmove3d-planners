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
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Smoothness cost

TrajectorySmoothness::TrajectorySmoothness()
{

}

FeatureVect TrajectorySmoothness::getFeatureCount( const API::Trajectory& t )
{
    FeatureVect count;

    int rows = active_dofs_.size();
    int cols = t.getNbOfViaPoints();

    Eigen::MatrixXd mat( rows, cols + 2*(control_cost_.getDiffRuleLength()-1) );
    Eigen::VectorXd q_init = t.getBegin()->getEigenVector( active_dofs_ );
    Eigen::VectorXd q_goal = t.getEnd()->getEigenVector( active_dofs_ );

    mat.block( 0, 0, rows, cols ) = t.getEigenMatrix( active_dofs_ );

    control_cost_.fillTrajectory( q_init, q_goal, mat );
    control_cost_.cost( mat );

    return count;
}

FeatureVect TrajectorySmoothness::getFeatures(const Configuration& q)
{
    FeatureVect count;
    return count;
}

void TrajectorySmoothness::setActivejoints( const std::vector<int>& active_joints )
{

}


#include "HRICS_features.hpp"

using namespace HRICS;
using std::cout;
using std::endl;

double Feature::cost( Configuration& q )
{
    FeatureVect phi = getFeatures( q );
    double cost = w_.transpose()*phi;
    return cost;
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

    control_cost_.fillTrajectory( q_init, q_goal, mat );
    mat.block( 0, 0, rows, cols ) = t.getEigenMatrix( active_dofs_ );
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

//-------------------------------------------------------------------
//-------------------------------------------------------------------

DistanceFeature::DistanceFeature( Robot* active, Robot* passive ) :
    Feature(),
    human_active_(active),
    human_passive_(passive)
{
//    active_joints_.push_back(0); // joint name : J0
    active_joints_.push_back(1); // joint name : Pelvis
    active_joints_.push_back(2); // joint name : TorsoX
//    active_joints_.push_back(3); // joint name : TorsoY
//    active_joints_.push_back(4); // joint name : TorsoZ
//    active_joints_.push_back(5); // joint name : HeadZ
//    active_joints_.push_back(6); // joint name : HeadY
    active_joints_.push_back(7); // joint name : HeadX
    active_joints_.push_back(8); // joint name : rShoulderX
//    active_joints_.push_back(9); // joint name : rShoulderZ
//    active_joints_.push_back(10); // joint name : rShoulderY
//    active_joints_.push_back(11); // joint name : rArmTrans
    active_joints_.push_back(12); // joint name : rElbowZ
//    active_joints_.push_back(13); // joint name : lPoint
    active_joints_.push_back(14); // joint name : rWristX
//    active_joints_.push_back(15); // joint name : rWristY
//    active_joints_.push_back(16); // joint name : rWristZ
    active_joints_.push_back(17); // joint name : lShoulderX
//    active_joints_.push_back(18); // joint name : lShoulderZ
//    active_joints_.push_back(19); // joint name : lShoulderY
//    active_joints_.push_back(20); // joint name : lArmTrans
    active_joints_.push_back(21); // joint name : lElbowZ
//    active_joints_.push_back(22); // joint name : lPoint
    active_joints_.push_back(23); // joint name : lWristX
//    active_joints_.push_back(24); // joint name : lWristY
//    active_joints_.push_back(25); // joint name : lWristZ
    active_joints_.push_back(26); // joint name : rHipX
//    active_joints_.push_back(27); // joint name : rHipY
//    active_joints_.push_back(28); // joint name : rHipZ
    active_joints_.push_back(29); // joint name : rKnee
    active_joints_.push_back(30); // joint name : rAnkleX
//    active_joints_.push_back(31); // joint name : rAnkleY
//    active_joints_.push_back(32); // joint name : rAnkleZ
    active_joints_.push_back(33); // joint name : lHipX
//    active_joints_.push_back(34); // joint name : lHipY
//    active_joints_.push_back(35); // joint name : lHipZ
    active_joints_.push_back(36); // joint name : lKnee
    active_joints_.push_back(37); // joint name : lAnkleX
//    active_joints_.push_back(38); // joint name : lAnkleY
//    active_joints_.push_back(39); // joint name : lAnkleZ
    active_joints_.push_back(40); // joint name : rPalm
    active_joints_.push_back(41); // joint name : lPalm
//    active_joints_.push_back(42); // joint name : rPoint
//    active_joints_.push_back(43); // joint name : lPoint
//    active_joints_.push_back(44); // joint name : lefthandgest
//    active_joints_.push_back(45); // joint name : righthandgest
//    active_joints_.push_back(46); // joint name : Eyes
//    active_joints_.push_back(47); // joint name : HriLookJoint

}

FeatureVect DistanceFeature::getFeatures(const Configuration& q )
{
    FeatureVect count = computeDistances();
    return count;
}

FeatureVect DistanceFeature::getFeatureCount(const API::Trajectory& t)
{
    FeatureVect count;
    return count;
}

FeatureVect DistanceFeature::computeDistances() const
{
    const std::vector<Joint*>& joints_a = human_active_->getAllJoints();
    const std::vector<Joint*>& joints_p = human_passive_->getAllJoints();

    FeatureVect dist(active_joints_.size()*active_joints_.size());

    int k=0;

    for(int i=0;i<int(active_joints_.size());i++)
    {
        for(int j=0;j<int(active_joints_.size());j++)
        {
            Eigen::Vector3d pos_a = joints_a[active_joints_[i]]->getVectorPos();
            Eigen::Vector3d pos_p = joints_p[active_joints_[j]]->getVectorPos();

            double j_dist = ( pos_a - pos_p ).norm();

            if( j_dist > 0.80 )
            {
                j_dist = 10000000;
            }

            dist[k++] = 1e-3/j_dist;
        }
    }

    //cout << "dist is : " << dist.transpose() << endl;

    return dist;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------

FeatureVect VisibilityFeature::getFeatures(const Configuration& q)
{
    FeatureVect count;
    return count;
}

FeatureVect VisibilityFeature::getFeatureCount(const API::Trajectory& t)
{
    FeatureVect count;
    return count;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------

FeatureVect MuskuloskeletalFeature::getFeatures(const Configuration& q)
{
    FeatureVect count = Eigen::VectorXd::Zero(1);
    return count;
}

FeatureVect MuskuloskeletalFeature::getFeatureCount(const API::Trajectory& t)
{
    FeatureVect count;
    return count;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------

FeatureVect LegibilityFeature::getFeatures(const Configuration& q)
{
    FeatureVect count = Eigen::VectorXd::Zero(1);
    return count;
}

FeatureVect LegibilityFeature::getFeatureCount(const API::Trajectory& t)
{
    FeatureVect count;
    return count;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------

FeatureVect ReachabilityFeature::getFeatures(const Configuration& q)
{
    FeatureVect count = Eigen::VectorXd::Zero(1);
    return count;
}

FeatureVect ReachabilityFeature::getFeatureCount(const API::Trajectory& t)
{
    FeatureVect count;
    return count;
}


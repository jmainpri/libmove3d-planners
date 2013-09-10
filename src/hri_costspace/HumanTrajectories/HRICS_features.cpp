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

TrajectorySmoothnessCost::TrajectorySmoothnessCost()
{

}

void TrajectorySmoothnessCost::initPolicy()
{
    policy_.reset(new stomp_motion_planner::CovariantTrajectoryPolicy());
    policy_->setPrintDebug( false );

    std::vector<double> derivative_costs; // = stomp_parameters_->getSmoothnessCosts();
    double ridge_factor=0;
    policy_->initialize( num_vars_free_, num_joints_, group_trajectory_.getDuration(),
                        ridge_factor,
                        derivative_costs,
                        planning_group_);

    // initialize the policy trajectory
    Eigen::VectorXd start = group_trajectory_.getTrajectoryPoint(free_vars_start_-1).transpose();
    Eigen::VectorXd end = group_trajectory_.getTrajectoryPoint(free_vars_end_+1).transpose();

    // set paramters
    int free_vars_start_index = DIFF_RULE_LENGTH - 1;
    std::vector< Eigen::VectorXd > parameters(num_joints_);
    for (int i=0; i<num_joints_; i++)
    {
        parameters[i] =  group_trajectory_.getJointTrajectory(i).segment(free_vars_start_index, num_vars_free_);
    }
    policy_->setToMinControlCost( start, end );
    policy_->setParameters( parameters );
}

void TrajectorySmoothnessCost::setGroupTrajectoryFromVectorConfig(const std::vector<confPtr_t>& traj)
{
    int start = free_vars_start_;
    int end = free_vars_end_;
//    TODO
//    if (iteration_==0) {
//        start = 0;
//        end = num_vars_all_-1;
//    }

    // Get the map from move3d index to group trajectory
    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;

    for (int i=start; i<=end; ++i)
    {
        for(int j=0; j<planning_group_->num_joints_;j++)
        {
            double point = (*traj[i])[joints[j].move3d_dof_index_];
            group_trajectory_.getTrajectoryPoint(i).transpose()[j] = point;

        }
    }
}

FeatureVect TrajectorySmoothnessCost::getFeatureCount(const API::Trajectory& t)
{
    double smoothness_cost = 0.0;

    std::vector<Eigen::MatrixXd> control_cost_matrices;
    std::vector<Eigen::VectorXd> noise(num_joints_);
    std::vector<Eigen::VectorXd> control_costs(num_joints_);

    for (int d=0; d<num_joints_; ++d)
    {
        policy_parameters_[d] = group_trajectory_.getFreeJointTrajectoryBlock(d);
        noise[d] = Eigen::VectorXd::Zero( policy_parameters_[d].size() );
    }

    policy_->computeControlCosts(control_cost_matrices, policy_parameters_, noise, 1, control_costs );

    for (int d=0; d<int(control_costs.size()); ++d)
    {
        smoothness_cost += control_costs[d].sum();
    }

    FeatureVect vect(1);
    vect[0] = smoothness_cost ;
    return vect;
}


FeatureVect TrajectorySmoothnessCost::getFeatures(const Configuration& q)
{
    FeatureVect count;
    return count;
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


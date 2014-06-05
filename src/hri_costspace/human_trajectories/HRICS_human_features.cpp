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
#include "HRICS_human_features.hpp"
#include "API/Graphic/drawModule.hpp"
#include "planner/planEnvironment.hpp"

#include <boost/bind.hpp>
#include <iomanip>

using namespace HRICS;
using namespace Move3D;

using std::cout;
using std::endl;

//-------------------------------------------------------------------
//-------------------------------------------------------------------

DistanceFeature::DistanceFeature( Robot* active, Robot* passive ) :
    Feature(),
    human_active_(active),
    human_passive_(passive)
{
    if ( !active->getUseLibmove3dStruct() ) //We're using OpenRAVE, use different joints
    {
        distance_joint_ids_.push_back( human_active_->getJoint("PelvisRotX")->getId() );
        distance_joint_ids_.push_back( human_active_->getJoint("rShoulderX")->getId() );
        distance_joint_ids_.push_back( human_active_->getJoint("rElbowZ")->getId() );
        distance_joint_ids_.push_back( human_active_->getJoint("rWristX")->getId() );
        distance_joint_ids_.push_back( human_active_->getJoint("lShoulderX")->getId() );
        distance_joint_ids_.push_back( human_active_->getJoint("lElbowZ")->getId() );
        distance_joint_ids_.push_back( human_active_->getJoint("lWristX")->getId() );
    }
    else    //Using Move3D
    {
        distance_joint_ids_.push_back( human_active_->getJoint("Pelvis")->getId() );
        distance_joint_ids_.push_back( human_active_->getJoint("rShoulderX")->getId() );
        distance_joint_ids_.push_back( human_active_->getJoint("rElbowZ")->getId() );
        distance_joint_ids_.push_back( human_active_->getJoint("rWristX")->getId() );
        distance_joint_ids_.push_back( human_active_->getJoint("lShoulderX")->getId() );
        distance_joint_ids_.push_back( human_active_->getJoint("lElbowZ")->getId() );
        distance_joint_ids_.push_back( human_active_->getJoint("lWristX")->getId() );
    }
//    distance_joint_ids_.push_back(0); // joint name : J0
//    distance_joint_ids_.push_back(1); // joint name : Pelvis
//    distance_joint_ids_.push_back(2); // joint name : TorsoX // USED
//    distance_joint_ids_.push_back(3); // joint name : TorsoY
//    distance_joint_ids_.push_back(4); // joint name : TorsoZ
//    distance_joint_ids_.push_back(5); // joint name : HeadZ
//    distance_joint_ids_.push_back(6); // joint name : HeadY
//    distance_joint_ids_.push_back(7); // joint name : HeadX // USED
//    distance_joint_ids_.push_back(8); // joint name : rShoulderX
//    distance_joint_ids_.push_back(9); // joint name : rShoulderZ
//    distance_joint_ids_.push_back(10); // joint name : rShoulderY
//    distance_joint_ids_.push_back(11); // joint name : rArmTrans
//    distance_joint_ids_.push_back(12); // joint name : rElbowZ
//    distance_joint_ids_.push_back(13); // joint name : lPoint
//    distance_joint_ids_.push_back(14); // joint name : rWristX
//    distance_joint_ids_.push_back(15); // joint name : rWristY
//    distance_joint_ids_.push_back(16); // joint name : rWristZ
//    distance_joint_ids_.push_back(17); // joint name : lShoulderX
//    distance_joint_ids_.push_back(18); // joint name : lShoulderZ
//    distance_joint_ids_.push_back(19); // joint name : lShoulderY
//    distance_joint_ids_.push_back(20); // joint name : lArmTrans
//    distance_joint_ids_.push_back(21); // joint name : lElbowZ
//    distance_joint_ids_.push_back(22); // joint name : lPoint
//    distance_joint_ids_.push_back(23); // joint name : lWristX
//    distance_joint_ids_.push_back(24); // joint name : lWristY
//    distance_joint_ids_.push_back(25); // joint name : lWristZ
//    distance_joint_ids_.push_back(26); // joint name : rHipX // USED
//    distance_joint_ids_.push_back(27); // joint name : rHipY
//    distance_joint_ids_.push_back(28); // joint name : rHipZ
//    distance_joint_ids_.push_back(29); // joint name : rKnee // USED
//    distance_joint_ids_.push_back(30); // joint name : rAnkleX // USED
//    distance_joint_ids_.push_back(31); // joint name : rAnkleY
//    distance_joint_ids_.push_back(32); // joint name : rAnkleZ
//    distance_joint_ids_.push_back(33); // joint name : lHipX // USED
//    distance_joint_ids_.push_back(34); // joint name : lHipY
//    distance_joint_ids_.push_back(35); // joint name : lHipZ
//    distance_joint_ids_.push_back(36); // joint name : lKnee // USED
//    distance_joint_ids_.push_back(37); // joint name : lAnkleX // USED
//    distance_joint_ids_.push_back(38); // joint name : lAnkleY
//    distance_joint_ids_.push_back(39); // joint name : lAnkleZ
//    distance_joint_ids_.push_back(40); // joint name : rPalm // USED
//    distance_joint_ids_.push_back(41); // joint name : lPalm // USED
//    distance_joint_ids_.push_back(42); // joint name : rPoint
//    distance_joint_ids_.push_back(43); // joint name : lPoint
//    distance_joint_ids_.push_back(44); // joint name : lefthandgest
//    distance_joint_ids_.push_back(45); // joint name : righthandgest
//    distance_joint_ids_.push_back(46); // joint name : Eyes
//    distance_joint_ids_.push_back(47); // joint name : HriLookJoint


    for( size_t i=0;i<distance_joint_ids_.size();i++)
    {
        human_active_joints_.push_back( human_active_->getJoint(distance_joint_ids_[i]) );
        human_passive_joints_.push_back( human_passive_->getJoint(distance_joint_ids_[i]) );
    }

    // Print active joints
    for(size_t i=0; i<human_active_joints_.size(); i++) {
        cout << std::setw( ceil(log10(human_active_joints_.size())) ) << std::setfill( '0' ) <<  i;
        cout << " , human active joint name : " << human_active_joints_[i]->getName() << endl;
    }

    distance_names_.clear();

    for(size_t i=0;i<distance_joint_ids_.size();i++) // nb of features is nb_joint_ids ^ 2
        for(size_t j=0;j<distance_joint_ids_.size();j++)
        {
            std::string name =  human_active_joints_[i]->getName() + " , " + human_passive_joints_[j]->getName();
            distance_names_.push_back( name );
        }

    int nb_of_features = distance_joint_ids_.size() * distance_joint_ids_.size();

    w_ = Eigen::VectorXd::Ones( nb_of_features );

    for( int i=0;i<w_.size();i++)
        w_[i] = 1.0;

    if( w_.size() == 49 )
    {
        w_ <<   0.50, 0.20, 0.60, 1.00, 0.60, 0.30, 1.00, 0.20, 0.70, 0.60,
                0.20, 0.20, 0.20, 0.80, 0.80, 0.80, 0.90, 0.80, 0.80, 0.80,
                0.50, 1.00, 0.90, 0.70, 0.10, 0.70, 0.80, 0.80, 0.50, 0.30,
                0.20, 0.20, 0.30, 0.20, 0.20, 1.00, 1.00, 1.00, 1.00, 1.00,
                1.00, 0.80, 1.00, 1.00, 1.00, 0.50, 0.80, 0.80, 0.10;
    }

    // Print feature names and weights
    for(size_t i=0; i<distance_names_.size(); i++) {
        cout.precision(2);
        cout.setf( std::ios::fixed, std:: ios::floatfield );
        cout  << std::setw( ceil(log10(distance_names_.size())) ) << std::setfill( '0' ) <<  i;
        cout << " , w : " << w_[i] << " , distance name : " << distance_names_[i] <<  endl;
//        cout << "( " << distance_names_[i] << " ) , " ;
//        if( (i+1)%10 == 0 ){
//            cout << endl;
//        }
    }
    cout << endl;

    if( global_DrawModule )
    {
        global_DrawModule->addDrawFunction( "HumanDistance", boost::bind( &DistanceFeature::draw, this) );
        global_DrawModule->enableDrawFunction( "HumanDistance" );
    }
}

FeatureVect DistanceFeature::getFeatures(const Configuration& q, std::vector<int> active_dofs )
{
    FeatureVect count = computeDistances();
    return count;
}

FeatureVect DistanceFeature::computeDistances() const
{
    FeatureVect dist( distance_joint_ids_.size() * distance_joint_ids_.size() );
    FeatureVect joints_dist( distance_joint_ids_.size() * distance_joint_ids_.size() );

    int k=0;

    Eigen::Vector3d pos_a;
    Eigen::Vector3d pos_p;

    const double max_distance = 0.80; // distance limit when the feature vanishes
    const double factor_distance = 0.16; // max_distance / ( 5 ~ 6 ) -> when the exp(-x) reaches 0

    for(size_t i=0; i<distance_joint_ids_.size(); i++)
        for(size_t j=0; j<distance_joint_ids_.size(); j++)
        {
            pos_a = human_active_joints_[i]->getVectorPos();
            pos_p = human_passive_joints_[j]->getVectorPos();

            double j_dist = ( pos_a - pos_p ).norm();

            joints_dist[k] = j_dist;

            if( j_dist < max_distance )
                dist[k++] = std::exp( -j_dist/factor_distance ); // 1e-3/j_dist;
            else
                dist[k++] = 0.0;
        }

//    cout.setf( std::ios::fixed, std:: ios::floatfield );
//    cout.precision(2);
//    cout.width(1);
//    cout << "dist is : " << dist.transpose() << endl;
//    cout << "joint dist : " << joints_dist.transpose() << endl;

    return dist;
}

void DistanceFeature::draw()
{
    for(size_t i=0; i<distance_joint_ids_.size(); i++)
        for(size_t j=0; j<distance_joint_ids_.size(); j++)
        {
            Eigen::Vector3d pos_a = human_active_joints_[i]->getVectorPos();
            Eigen::Vector3d pos_p = human_passive_joints_[j]->getVectorPos();

            move3d_draw_one_line( pos_a[0], pos_a[1], pos_a[2], pos_p[0], pos_p[1], pos_p[2], Blue, NULL );
        }
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------

VelocityFeature::VelocityFeature( Move3D::Robot* active ) :
    Feature(),
    human_active_(active)
{
    human_active_joints_.push_back( human_active_->getJoint("Pelvis") );
    human_active_joints_.push_back( human_active_->getJoint("rShoulderX") );
    human_active_joints_.push_back( human_active_->getJoint("rElbowZ") );

    for( int i=0;i<human_active_joints_.size();i++)
        veclocity_joint_ids_.push_back( human_active_joints_[i]->getId() );
}

// Compute velocity between two configurations
std::vector<Eigen::Vector3d> VelocityFeature::getVelocity(const Move3D::Configuration& q_0, const Move3D::Configuration& q_1, double dt )
{
    std::vector<Eigen::Vector3d> velocities(veclocity_joint_ids_.size());
    std::vector<Eigen::Vector3d> pos_0(veclocity_joint_ids_.size());
    std::vector<Eigen::Vector3d> pos_1(veclocity_joint_ids_.size());

    human_active_->setAndUpdate(q_0);

    for( int i=0;i<veclocity_joint_ids_.size();i++)
        pos_0[i] = human_active_joints_[i]->getVectorPos();

    human_active_->setAndUpdate(q_1);

    for( int i=0;i<veclocity_joint_ids_.size();i++)
        pos_1[i] = human_active_joints_[i]->getVectorPos();

    // Get velocities
    for( int i=0;i<veclocity_joint_ids_.size();i++)
    {
        velocities[i] = ( pos_0[i] - pos_1[i] ) / dt;
    }

    return velocities;
}

void VelocityFeature::stackVelocities( Move3D::FeatureVect& stack , const std::vector<Eigen::Vector3d>& velocities )
{
    stack.resize( veclocity_joint_ids_.size() * 3 );

    if( veclocity_joint_ids_.size() != velocities.size() )
        return;

    for( int i=0; i<velocities.size(); i++ )
    {

    }
}

void VelocityFeature::draw()
{
//    for( int i=0;i<veclocity_joint_ids_.size();i++)
//        pos_0[i] = human_active_joints_[i]->getVectorPos();
//    q_last_
}

FeatureVect VelocityFeature::getFeatures(const Configuration& q, std::vector<int> active_dofs)
{
    FeatureVect count;
    return count;
}

FeatureVect VelocityFeature::getFeatureCount(const Move3D::Trajectory& traj)
{
    FeatureVect count( veclocity_joint_ids_.size() * 3 );
    FeatureVect feat1( veclocity_joint_ids_.size() * 3 );
    FeatureVect feat2( veclocity_joint_ids_.size() * 3 );

    double t = 0.0;
    double t_max = traj.getParamMax();
    double step = ENV.getDouble(Env::dmax)*PlanEnv->getDouble(PlanParam::costResolution);
    int n_step = int(t_max/step);
    if( n_step < 100 ){ // minumum of 100 steps
        n_step = 100;
        step = t_max / double(n_step);
    }

    std::vector<Eigen::Vector3d> velocities(veclocity_joint_ids_.size());

    confPtr_t q_0 = traj.configAtParam(0);
    confPtr_t q_1 = traj.configAtParam(step);

    for ( int i=0; i<(n_step-1); i++ )
    {
        t += step;
        q_0 = traj.configAtParam(t);
        q_1 = traj.configAtParam(t+step);

        velocities = getVelocity( *q_0, *q_1, step );

        count += ( (feat1 + feat2) / 2 )  * step;
        feat1 = feat2;
    }

    return count;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------

FeatureVect VisibilityFeature::getFeatures(const Configuration& q, std::vector<int> active_dofs)
{
    FeatureVect count;
    return count;
}

FeatureVect VisibilityFeature::getFeatureCount(const Move3D::Trajectory& t)
{
    FeatureVect count;
    return count;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------

FeatureVect MuskuloskeletalFeature::getFeatures(const Configuration& q, std::vector<int> active_dofs)
{
    FeatureVect count = Eigen::VectorXd::Zero(1);
    return count;
}

FeatureVect MuskuloskeletalFeature::getFeatureCount(const Move3D::Trajectory& t)
{
    FeatureVect count;
    return count;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------

FeatureVect LegibilityFeature::getFeatures(const Configuration& q, std::vector<int> active_dofs)
{
    FeatureVect count = Eigen::VectorXd::Zero(1);
    return count;
}

FeatureVect LegibilityFeature::getFeatureCount(const Move3D::Trajectory& t)
{
    FeatureVect count;
    return count;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------

FeatureVect ReachabilityFeature::getFeatures(const Configuration& q, std::vector<int> active_dofs)
{
    FeatureVect count = Eigen::VectorXd::Zero(1);
    return count;
}

FeatureVect ReachabilityFeature::getFeatureCount(const Move3D::Trajectory& t)
{
    FeatureVect count;
    return count;
}

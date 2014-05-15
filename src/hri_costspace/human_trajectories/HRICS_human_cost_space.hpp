#include "API/Device/robot.hpp"

#include "HRICS_human_features.hpp"
#include "hri_costspace/gestures/HRICS_record_motion.hpp"

namespace HRICS
{

class HumanTrajCostSpace : public Move3D::StackedFeatures
{

public:
    HumanTrajCostSpace( Move3D::Robot* active, Move3D::Robot* passive );
    ~HumanTrajCostSpace();

    void draw() { }

//    FeatureVect getFeatureCount(const Move3D::Trajectory& t);
//    FeatureVect getFeatures(const Configuration& t);

    //! Add a passive trajectory
    void setPassiveTrajectory( const motion_t& traj );

    //! Set the passive configuration
    void setPassiveConfig( const Move3D::Configuration& q );

    Move3D::Robot* getActiveHuman() { return human_active_; }
    Move3D::Robot* getPassiveHuman() { return human_passive_; }

private:

    Move3D::Robot* human_active_;
    Move3D::Robot* human_passive_;

    Move3D::Trajectory passive_traj_;
    int nb_way_points_;

    DistanceFeature dist_feat_;
    VisibilityFeature visi_feat_;
    MuskuloskeletalFeature musk_feat_;
    ReachabilityFeature reach_feat_;
    LegibilityFeature legib_feat_;

    Move3D::TrajectorySmoothness smoothness_feat_;
};

class HumanTrajSimulator
{
public:
    HumanTrajSimulator( HumanTrajCostSpace* cost_space );

    bool init();
    bool run();

private:

    HumanTrajCostSpace* cost_space_;

    Move3D::Robot* human_active_;
    Move3D::Robot* human_passive_;

    Move3D::confPtr_t q_init_;
    Move3D::confPtr_t q_goal_;

    bool init_scenario_;

    std::string traj_folder_;

};

}

//! main test function for human planning
void HRICS_run_human_planning();

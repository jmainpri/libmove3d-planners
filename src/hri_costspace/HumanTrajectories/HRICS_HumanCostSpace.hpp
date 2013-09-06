#include "API/Device/robot.hpp"

#include "HRICS_features.hpp"
#include "hri_costspace/Gestures/HRICS_RecordMotion.hpp"

namespace HRICS
{

class HumanTrajCostSpace : public Feature
{

public:
    HumanTrajCostSpace( Robot* active, Robot* passive );
    ~HumanTrajCostSpace();

    void draw() { }

    FeatureVect getFeatureCount(const API::Trajectory& t);
    FeatureVect getFeatures(const Configuration& t);

    //! Add a passive trajectory
    void setPassiveTrajectory( const motion_t& traj );

    //! Set the passive configuration
    void setPassiveConfig( const Configuration& q );

    Robot* getActiveHuman() { return human_active_; }
    Robot* getPassiveHuman() { return human_passive_; }

private:

    Robot* human_active_;
    Robot* human_passive_;

    API::Trajectory passive_traj_;
    int nb_way_points_;

    DistanceFeature dist_feat_;
    VisibilityFeature visi_feat_;
    MuskuloskeletalFeature musk_feat_;
    ReachabilityFeature reach_feat_;
    LegibilityFeature legib_feat_;
};

class HumanTrajSimulator
{
public:
    HumanTrajSimulator( HumanTrajCostSpace* cost_space );

    bool init();
    bool run();

private:

    HumanTrajCostSpace* cost_space_;

    Robot* human_active_;
    Robot* human_passive_;

    confPtr_t q_init_;
    confPtr_t q_goal_;

    bool init_scenario_;

};

}

//! main test function for human planning
void HRICS_run_human_planning();

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

    double getCost(Configuration& q) const;

    FeatureVect getFeatureCount(const API::Trajectory& t);

    //! Add a passive trajectory
    void setPassiveTrajectory( const motion_t& traj );

private:

    FeatureVect getDistance();
    FeatureVect getVisibility();
    FeatureVect getLegibility();
    FeatureVect getMuskuloskeletal();

    Robot* human_active_;
    Robot* human_passive_;

    API::Trajectory passive_traj_;
    int nb_way_points_;
};

class HumanTrajSimulator
{
public:
    HumanTrajSimulator(  Robot* active, Robot* passive  );

    bool init();
    bool run();

private:

    Robot* human_active_;
    Robot* human_passive_;

    confPtr_t q_init_;
    confPtr_t q_goal_;

    HumanTrajCostSpace cost_space_;

    bool init_scenario_;

};

}

extern HRICS::HumanTrajCostSpace* global_humanTrajectoryCostSpace;

double HRICS_getHumanTrajectoryCost(Configuration& q);

//! main test function for human planning
void HRICS_run_human_planning();

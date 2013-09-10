#ifndef HRICS_HUMANIOC_HPP
#define HRICS_HUMANIOC_HPP

#include "HRICS_ioc.hpp"
#include "hri_costspace/Gestures/HRICS_RecordMotion.hpp"

namespace HRICS
{

class HumanIoc : public IocEvaluation
{
public:
    HumanIoc( Robot* active, Robot* passive );

    void runLearning();
    void setPlanningGroup();
    void setDemos( const std::vector<motion_t>& stored_motions );
private:
    API::Trajectory getTrajectoryFromMotion( const motion_t& m ) const;
};

}

void HRICS_run_human_ioc_from_recorded_motion();
void HRICS_run_human_ioc_evaluation();

#endif // HRICS_HUMANIOC_HPP

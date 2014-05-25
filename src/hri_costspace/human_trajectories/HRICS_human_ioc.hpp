#ifndef HRICS_HUMANIOC_HPP
#define HRICS_HUMANIOC_HPP

#include "HRICS_ioc.hpp"
#include "hri_costspace/gestures/HRICS_record_motion.hpp"

namespace HRICS
{

class HumanIoc : public IocEvaluation
{
public:
    HumanIoc( Move3D::Robot* active, Move3D::Robot* passive, int nb_demos, int nb_samples, int nb_way_points, MultiplePlanners& planners,
              std::string folder,  std::string traj_folder, std::string tmp_data_folder );

    void runSampling();
    void setPlanningGroup();
    void setDemos( const std::vector<motion_t>& stored_motions );

private:
    Move3D::Trajectory getTrajectoryFromMotion( const motion_t& m ) const;

};

}

void HRICS_run_human_ioc_from_recorded_motion();
void HRICS_run_human_ioc_evaluation();

#endif // HRICS_HUMANIOC_HPP

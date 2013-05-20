#ifndef HRICS_INVERSEOPTIMALCONTROL_HPP
#define HRICS_INVERSEOPTIMALCONTROL_HPP

#include "../Gestures/HRICS_RecordMotion.hpp"

namespace HRICS
{
class InverseOptimalControl
{
public:
    InverseOptimalControl();
    ~InverseOptimalControl();

    void setActiveRobot(Robot* robot);

    void setTrajectory();
    void lauchOptimization();

private:
    Robot* m_robot;
    std::vector< std::pair<confPtr_t,confPtr_t> > m_start_goal_config;
};
}

#endif // HRICS_INVERSEOPTIMALCONTROL_HPP

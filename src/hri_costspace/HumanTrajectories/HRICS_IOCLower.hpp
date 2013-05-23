#ifndef HRICS_IOCLower_HPP
#define HRICS_IOCLower_HPP

#include "../Gestures/HRICS_RecordMotion.hpp"

namespace HRICS
{
class IOCLower
{
public:
    IOCLower();
    ~IOCLower();

    void setActiveRobot(Robot* robot);

    void setTrajectory();
    void lauchOptimization();

private:
    Robot* m_robot;
    std::vector< std::pair<confPtr_t,confPtr_t> > m_start_goal_config;
    bool m_is_scenario_init;
};
}

#endif // HRICS_IOCLower_HPP

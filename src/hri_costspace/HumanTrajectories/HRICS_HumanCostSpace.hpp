#include "API/Device/robot.hpp"

double HRICS_getHumanTrajectoryCost(Configuration& q);

namespace HRICS
{
class HumanTrajCostSpace
{

public:
    HumanTrajCostSpace();
    ~HumanTrajCostSpace();

    void setActiveHuman(Robot* human) { m_human_active = human; }
    void setPassiveHuman(Robot* human) { m_human_passive = human; }

    void draw() { }

    double getCost(Configuration& q) const;

private:

    double getDistance();
    double getVisibility();
    double getLegibility();

    Robot* m_human_active;
    Robot* m_human_passive;
};
}

extern HRICS::HumanTrajCostSpace* global_humanTrajectoryCostSpace;



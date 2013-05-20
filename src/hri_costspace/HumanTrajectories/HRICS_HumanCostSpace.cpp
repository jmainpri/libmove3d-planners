#include "HRICS_HumanCostSpace.hpp"

using namespace HRICS;

HumanTrajCostSpace::HumanTrajCostSpace()
{

}

HumanTrajCostSpace::~HumanTrajCostSpace()
{

}

double HumanTrajCostSpace::getCost(Configuration& q) const
{
    return 0.0;
}

double HumanTrajCostSpace::getDistance()
{
    return 0.0;
}

double HumanTrajCostSpace::getVisibility()
{
    return 0.0;
}

double HumanTrajCostSpace::getLegibility()
{
    return 0.0;
}

#include "HRICS_HumanCostSpace.hpp"

using namespace HRICS;

HumanTrajCostSpace* global_humanTrajectoryCostSpace = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Cost Function

double HRICS_getHumanTrajectoryCost(Configuration& q)
{
    return global_humanTrajectoryCostSpace->getCost(q);
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Human Trajectory Cost Space

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

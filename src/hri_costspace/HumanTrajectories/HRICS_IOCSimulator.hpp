#ifndef HRICS_IOCSIMULATOR_HPP
#define HRICS_IOCSIMULATOR_HPP

#include "HRICS_InverseOptimalControl.hpp"
#include "HRICS_HumanCostSpace.hpp"

namespace HRICS
{

class IOCSimulator
{
public:

    IOCSimulator();
    ~IOCSimulator();

    void run();

private:

    bool initialize();

    InverseOptimalControl* m_ioc;

    RecordMotion* m_recorder;

    HumanTrajCostSpace* m_costspace;
};

}

#endif // HRICS_IOCSIMULATOR_HPP

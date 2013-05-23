#ifndef HRICS_IOCUpper_HPP
#define HRICS_IOCUpper_HPP

#include "HRICS_IOCLower.hpp"
#include "HRICS_HumanCostSpace.hpp"

extern void HRICS_initIverseOptimalControlFramework();

namespace HRICS
{
class IOCUpper
{
public:

    IOCUpper(const std::vector<Robot*>& humans);
    ~IOCUpper();

    void run();

private:

    bool initialize();

    std::vector<Robot*> m_humans;

    IOCLower* m_ioc;

    RecordMotion* m_recorder;

    HumanTrajCostSpace* m_costspace;
};

}

extern HRICS::IOCUpper* global_IOCUpper;

#endif // HRICS_IOCUpper_HPP

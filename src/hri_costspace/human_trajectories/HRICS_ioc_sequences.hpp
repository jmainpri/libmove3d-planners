#ifndef HRICS_IOC_SEQUENCES_HPP
#define HRICS_IOC_SEQUENCES_HPP

#include "feature_space/features.hpp"

#include <iostream>

namespace HRICS
{

class IocSequences
{
public:
    IocSequences();
    ~IocSequences() { }

    bool run();
    void set_features();

private:

    // IOC PHASES
    enum phase_t { generate=0,
                   sample=1,
                   compare=2,
                   run_planner=3,
                   default_phase=4,
                   monte_carlo=5 };

    // TYPE OF FEATURES
    enum feature_t {
        no_features,
        spheres,
        human_trajs };


    phase_t phase_;
    feature_t features_type_;

    std::vector<int> active_joints_;
    Move3D::StackedFeatures* feature_fct_;

};

}

#endif // HRICS_IOC_SEQUENCES_HPP

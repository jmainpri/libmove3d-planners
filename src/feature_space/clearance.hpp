#ifndef CLEARANCE_HPP
#define CLEARANCE_HPP

#include "features.hpp"
#include "API/Device/robot.hpp"
#include "collision_space/body_surface_sampler.hpp"

namespace Move3D
{

class Clearance : public Feature
{
    public:

        Clearance();
        ~Clearance();

        void initialize();

        FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_dofs=std::vector<int>() );
        FeatureVect getFeatureCount( Move3D::LocalPath& path, int& nb_calls );
    };
}

bool FEATURES_init_Clearance_cost();

extern Move3D::Clearance* global_ClearanceCostFct;

#endif // CLEARANCE_HPP

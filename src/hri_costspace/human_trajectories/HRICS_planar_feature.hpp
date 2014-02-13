#ifndef HRICS_PLANNARFEATURE_HPP
#define HRICS_PLANNARFEATURE_HPP

#include "HRICS_features.hpp"
#include "API/Device/robot.hpp"

namespace HRICS
{
class PlanarFeature : public Feature
{
    public:

        PlanarFeature() { }

        virtual void initialize() = 0;

        virtual void produceCostMap(int ith);
        virtual void produceDerivativeFeatureCostMap(int ith);
        virtual void placeCenterGrid(bool on_wall);
        void printWeights() const;
        int addCenters(std::string type);

    protected:
        Robot* robot_;
        std::vector<Robot*> centers_;
        int nb_dofs_;
    };
}

extern HRICS::PlanarFeature* global_PlanarCostFct;

#endif // HRICS_PLANNARFEATURE_HPP

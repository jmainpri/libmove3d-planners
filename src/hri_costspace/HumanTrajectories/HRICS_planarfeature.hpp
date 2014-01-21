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

        void produceCostMap();
        void produceDerivativeFeatureCostMap();
        void placeCenterGrid(bool on_wall);
        void printWeights() const;
        int addCenters(std::string type);
        void computeSize();

    protected:
        Robot* robot_;
        std::vector<Robot*> centers_;
    };
}

extern HRICS::PlanarFeature* global_PlanarCostFct;

#endif // HRICS_PLANNARFEATURE_HPP

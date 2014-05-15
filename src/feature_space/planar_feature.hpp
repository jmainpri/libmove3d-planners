#ifndef HRICS_PLANNARFEATURE_HPP
#define HRICS_PLANNARFEATURE_HPP

#include "features.hpp"
#include "API/Device/robot.hpp"

extern std::string cost_map_folder;

namespace Move3D
{
class PlanarFeature : public Feature
{
    public:

        PlanarFeature();

        virtual void initialize() = 0;

        virtual void produceCostMap(int ith);
        virtual void produceDerivativeFeatureCostMap(int ith);
        virtual void placeCenterGrid(bool on_wall);
        void printWeights() const;
        int addCenters(std::string type);
        double minDistToDemonstrations( const Move3D::Configuration& q );
        virtual double jacobianCost(const Move3D::Configuration& q);

        Eigen::VectorXd getStompCost( const Move3D::Trajectory& t );
        double getVariance( const Move3D::Trajectory& t );
        double getDeltaSum( const Move3D::Trajectory& t );

        void generateRandomEnvironment();

        FeatureVect phi_demos_;
        FeatureVect phi_cumul_;
        FeatureVect phi_jac_cumul_;

        std::vector<Move3D::Trajectory> demos_;

        double balance_cumul_jac_;
        double min_diff_;
        double cur_min_diff_;

        double increment_delta_;

        int iter_;

    protected:
        Move3D::Robot* robot_;
        std::vector<Move3D::Robot*> centers_;
        int nb_dofs_;
    };
}

extern Move3D::PlanarFeature* global_PlanarCostFct;

#endif // HRICS_PLANNARFEATURE_HPP

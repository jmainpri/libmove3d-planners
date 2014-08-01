/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
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

        double scaling_features_;
        double scaling_weight_;

    protected:
        Move3D::Robot* robot_;
        std::vector<Move3D::Robot*> centers_;
        int nb_dofs_;
    };
}

extern Move3D::PlanarFeature* global_PlanarCostFct;

#endif // HRICS_PLANNARFEATURE_HPP

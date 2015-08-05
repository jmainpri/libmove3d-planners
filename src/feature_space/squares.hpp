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
#ifndef HRICS_SQUARES_HPP
#define HRICS_SQUARES_HPP

#include "planar_feature.hpp"

////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

namespace Move3D
{

class Square
{
public:
    Square( const Eigen::VectorXd& center, const Eigen::VectorXd& size )
    {
        center_ = center;
        size_ = size;
    }

    virtual void draw() const;

    Eigen::VectorXd center_;
    Eigen::VectorXd size_;
};

class Squares : public PlanarFeature
{
public:
    Squares();
    virtual ~Squares();

    virtual void initialize();
    void computeSize();

    FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_features = std::vector<int>(0) );
    double getFeaturesJacobianMagnitude( const Move3D::Configuration& q );

    void produceDerivativeFeatureCostMap(int ith);

    double distToSquare(  const Square& square, const Move3D::Configuration& q  );
    bool isInAASquare( const std::vector<Eigen::Vector2d>& corners, Eigen::Vector2d p );
    double pointToLineSegmentDistance(const Eigen::VectorXd& p, const Eigen::VectorXd& p1, const Eigen::VectorXd& p2, Eigen::VectorXd& closestPoint);

    void draw();

protected:
    std::vector<const Square*> boxes_;
};

}

// Global cost function
bool HRICS_init_square_cost();

#endif // HRICS_SQUARES_HPP

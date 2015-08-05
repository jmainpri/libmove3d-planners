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
#ifndef HRICS_BOXES_HPP
#define HRICS_BOXES_HPP

#include "squares.hpp"
#include "API/Device/robot.hpp"
#include "collision_space/body_surface_sampler.hpp"

namespace Move3D
{

class Box : public Square
{
public:
    Box( const Eigen::VectorXd& center, const Eigen::VectorXd& size ) : Square(center,size) {}
    void draw() const;
};

class Boxes : public Squares
{
    public:

        Boxes();
        virtual ~Boxes();

        void initialize();

        FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_dofs );
        double getFeaturesJacobianMagnitude( const Move3D::Configuration& q );
        double jacobianCost( const Move3D::Configuration& q );

        void computeSize();

        bool isInAABox( const Eigen::VectorXd& limits, Eigen::Vector3d p );
        double distToBox(  const Box& box, const Move3D::Configuration& q  );

        void drawCollisionPoints();
        void draw();

private:
        std::vector<int> active_joints_;
        Move3D::BodySurfaceSampler* sampler_;
    };
}

bool HRICS_init_boxes_cost();

extern Move3D::Boxes* global_BoxesCostFct;

#endif // HRICS_BOXES_HPP

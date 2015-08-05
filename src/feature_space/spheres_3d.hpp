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
#ifndef HRICS_SHPERES_3D_HPP
#define HRICS_SHPERES_3D_HPP

#include "planar_feature.hpp"
#include "API/Device/robot.hpp"
#include "collision_space/body_surface_sampler.hpp"

namespace Move3D
{

class Sphere
{
public:
    Sphere( const Eigen::VectorXd& center, double raduis ) :
        center_(center), raduis_(raduis)
    {}

    void draw() const;

    Eigen::Vector3d center_;
    double color_[4];
    double raduis_;
};

class Spheres3D : public PlanarFeature
{
    public:

        Spheres3D();
        virtual ~Spheres3D();

        void initialize();

        FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0) );
        double getFeaturesJacobianMagnitude(const Move3D::Configuration& q);
        double jacobianCost(const Move3D::Configuration& q);

        void computeSize();
        void placeCenterGrid(bool on_wall);

        double distToShpere( const Sphere& sph );

        void setSphereToDraw(int id, bool enable);
        void setShperesToDraw();
        void drawCollisionPoints();
        void draw();

private:
        std::vector<int> active_joints_;
        Move3D::BodySurfaceSampler* sampler_;
        std::vector<Sphere*> spheres_;
        std::vector<int> sphere_to_draw_;
    };
}

bool HRICS_init_shperes_3d_cost();

extern Move3D::Spheres3D* global_Spheres3DCostFct;

#endif // HRICS_SHPERES_3D_HPP

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
#ifndef POINTS_H
#define POINTS_H

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

/**
@ingroup GRID
*! Vector of 3d points that can be ploted in the 3d viewer as cubes very fast
*! the points are stored in a std::vector and the class has a similar API 
*/
class PointCloud
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloud();
    PointCloud(double PointSize);

    void push_back(const Eigen::Vector3d &point);

    void clear()
    {
        m_AllPoints.clear();
    }

    unsigned int size()
    {
        return m_AllPoints.size();
    }

    void resize(unsigned int sz)
    {
        m_AllPoints.resize(sz);
    }

    /**
     * Access the point
     */
    Eigen::Vector3d& operator [] ( const int &i ) { return m_AllPoints[i]; }


    void drawAllPoints(double* color = NULL);
    void drawAllPoints(const Eigen::Transform3d & t,  double* color = NULL );

private:
    void drawOnePoint(bool withTransform, const Eigen::Transform3d & t, int i);

    std::vector< Eigen::Vector3d > m_AllPoints;
    Eigen::Vector3d m_CubeSize;
};

extern PointCloud* PointsToDraw;

#endif // POINTS_H

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

#ifndef GRAPHIC_M3D_MODULE_HPP
#define GRAPHIC_M3D_MODULE_HPP

#include <string>
#include <vector>
#include <map>
#include <set>
#include <boost/function.hpp>

namespace Move3D
{
class Robot;
}

namespace Graphic 
{

//! Class thats holding the drawing functions
class DrawFunctions
{
public:
    DrawFunctions();
    ~DrawFunctions();

    // Get all draw functions
    std::vector<std::string> getAllDrawFunctions();

    // Get active draw functions
    std::vector<std::string> getActiveDrawFunctions();

    // Select the draw function with the given name in the map
    bool enableDrawFunction( std::string name );

    // Select the draw function with the given name in the map
    bool disableDrawFunction( std::string name );

    // Register a new draw function
    void addDrawFunction( std::string name, boost::function<void()> f );

    // Delete a cost function
    void deleteDrawFunction( std::string name );

    // draw all functions
    void draw();

protected:
    std::map< std::string, boost::function<void()> > functions_;
    std::set< std::string > active_functions_;
};

void initDrawFunctions();

}

void move3d_set_fct_draw_sphere( boost::function<void( double, double, double, double, double*, Move3D::Robot* )> fct );
void move3d_set_fct_draw_one_line( boost::function<void( double ,double ,double, double, double, double, int, double*, Move3D::Robot*)> fct );
void move3d_set_fct_draw_clear_handles( boost::function<void( Move3D::Robot* )> fct );

void move3d_draw_sphere( double x, double y, double z, double radius, double* color_vect, Move3D::Robot* R=NULL );
void move3d_draw_one_line( double x1, double y1, double z1, double x2, double y2, double z2, int color, double *color_vect, Move3D::Robot* R=NULL );
void move3d_draw_clear_handles( Move3D::Robot* R=NULL );

extern Graphic::DrawFunctions* global_DrawModule;

#endif

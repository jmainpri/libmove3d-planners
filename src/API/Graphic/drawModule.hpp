/*
 *  drawModule.hpp
 *
 *  Created by Jim Mainprice on 18/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef GRAPHIC_M3D_MODULE_HPP
#define GRAPHIC_M3D_MODULE_HPP

#include <string>
#include <vector>
#include <map>
#include <boost/function.hpp>

namespace Graphic 
{

void initDrawFunctions();

//! Class thats holding the drawing functions
class DrawFunctions
{
public:
    DrawFunctions();

    // Get all draw functions
    std::vector<std::string> getAllDrawFunctions();

    // Get active draw functions
    std::vector<std::string> getActiveDrawFunctions();

    // Select the draw function with the given name in the map
    bool enableDrawFunction( std::string name );

    // Select the draw function with the given name in the map
    bool disableDrawFunction( std::string name );

    // Register a new draw function
    void addDrawFunction( std::string name, boost::function<void> f );

    // Delete a cost function
    void deleteDrawFunction( std::string name );

protected:
    std::map<std::string, boost::function<void> > mFunctions;
};
}

#endif

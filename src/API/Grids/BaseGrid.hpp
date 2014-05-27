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
#ifndef BASEGRID_HPP
#define BASEGRID_HPP

#include <vector>
#include <libxml/parser.h>

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>

/**
  * Base class for a Grid
  */
namespace Move3D
{

class BaseCell
{
public:
    BaseCell();
    virtual ~BaseCell();

    virtual void draw() = 0;

    virtual bool writeToXml(xmlNodePtr cur);
    virtual bool readCellFromXml(xmlNodePtr cur);
};

class BaseGrid
{
public:
    BaseGrid();
    BaseGrid(const BaseGrid& grid);
    virtual ~BaseGrid();
    
    BaseCell* getCell(unsigned int i);
    unsigned int getNumberOfCells();
    
    virtual void draw() =0;

    virtual std::vector<Eigen::Vector3d> getBox();

    virtual bool writeToXmlFile(std::string file);
    virtual bool loadFromXmlFile(std::string file);

    std::string getName() { return m_name; }
    
protected:
    std::vector<BaseCell*> _cells;
    std::string m_name;
};
}

#endif // BASEGRID_HPP

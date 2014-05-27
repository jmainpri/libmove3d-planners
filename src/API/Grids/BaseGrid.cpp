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
#include "BaseGrid.hpp"

using namespace Move3D;
using namespace std;

// Pointers to grids
BaseGrid* API_activeGrid = NULL;
vector<BaseGrid*> API_allGrids;

BaseCell::BaseCell()
{
}

BaseCell::~BaseCell()
{
}

bool BaseCell::readCellFromXml(xmlNodePtr cur)
{
    return false;
}

bool BaseCell::writeToXml(xmlNodePtr cur)
{
    return false;
}

// Returns all grids
vector<BaseGrid*> api_get_all_grids()
{
  return API_allGrids;
}

BaseGrid::BaseGrid() : m_name("Basic Grid")
{
}

BaseGrid::BaseGrid(const BaseGrid& grid) : m_name(grid.m_name)
{

}

BaseGrid::~BaseGrid()
{
    for(unsigned int i=0;i<_cells.size();i++)
    {
        delete _cells.at(i);
    }
}

/*!
 * \brief Get Cell
 *
 * \param index
 */
BaseCell* BaseGrid::getCell(unsigned int i)
{
    return _cells[i];
}

/*!
 * \brief Get Number Of Cells
 */
unsigned int BaseGrid::getNumberOfCells()
{
    return _cells.size();
}

/*!
 * @breif
 */
std::vector<Eigen::Vector3d> BaseGrid::getBox()
{
	std::vector<Eigen::Vector3d> vect;
	vect.clear();
	return vect;
}

/*!
 * \brief Virtual function for
 * creating an xml document
 */
bool BaseGrid::writeToXmlFile(std::string file)
{
	return true;
}

/*!
 * \brief Virtual function for
 * reading from an xml document
 */
bool BaseGrid::loadFromXmlFile(std::string file)
{
	return true;
}

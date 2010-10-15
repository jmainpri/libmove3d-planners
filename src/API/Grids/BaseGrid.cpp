#include "BaseGrid.hpp"

using namespace API;

BaseGrid* API_activeGrid = NULL;

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

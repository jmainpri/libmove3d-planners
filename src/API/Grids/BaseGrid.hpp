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

#ifndef BASEGRID_HPP
#define BASEGRID_HPP

#include "API/Grids/BaseCell.hpp"

#include <vector>

#include <Eigen/Core>

/**
  * Base class for a Grid
  */
namespace API
{
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

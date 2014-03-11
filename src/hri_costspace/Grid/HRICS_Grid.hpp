#ifndef HRIGRID_HPP
#define HRIGRID_HPP

#include "API/Device/robot.hpp"
#include "API/Grids/ThreeDGrid.hpp"

/**
 @ingroup HRICS
 @brief Cell for the HRICS AStar
 */
namespace HRICS
{
	class Cell;
	
	class Grid : public Move3D::ThreeDGrid
	{
	public:
		Grid();
		Grid( std::vector<int> size );
		Grid(double pace, std::vector<double> envSize);
		
		Move3D::ThreeDCell* createNewCell(unsigned int index, unsigned int x, unsigned int y, unsigned int z );
		void computeAllCellCost();
		
		void drawVectorFeild();
		void drawSpheres();
		void draw();
		void resetCellCost();
		
        void setRobot(Move3D::Robot* rob) { _Robot = rob; }
        Move3D::Robot* getRobot() { return _Robot; }
		
		bool isVirtualObjectPathValid(Cell* fromCell,Cell* toCell);
		
		void computeVectorField();
		
	private:
		
        Move3D::Robot* _Robot;
	};
}

#endif // HRIGRID_HPP

#ifndef HRICELL_H
#define HRICELL_H

#include "API/Grids/ThreeDCell.hpp"

#include "HRICS_Grid.hpp"

#include <libmove3d/include/Graphic-pkg.h>

/**
 @ingroup HRICS
 @brief Cell for the HRICS AStar
 */
namespace HRICS
{
	class Cell : public API::ThreeDCell
	{
		
	public:
		Cell();
		Cell(int i, Eigen::Vector3i pos , Eigen::Vector3d corner, HRICS::Grid* grid);
		
		~Cell() { }
		
		double getCost();
		//        double getHRICostSpace();
		void setBlankCost() { _CostIsComputed = false; this->resetExplorationStatus(); }
		
		Eigen::Vector3i getCoord() { return _Coord; }
		
		bool getOpen() { return _Open; }
		void setOpen() { _Open = true; }
		
		bool getClosed() { return _Closed; }
		void setClosed() { _Closed = true; }
		
		void resetExplorationStatus();
		
		GLint getDisplayList() { return m_list; }
		void createDisplaylist();
		
		bool getIsCostComputed() { return _CostIsComputed; }
		
		void setGradient(const Eigen::Vector3d& grad) { m_GradientDirection = grad; } 
		Eigen::Vector3d getGradient() { return m_GradientDirection; }
		
		void draw();
		
	private:
		
		Eigen::Vector3i _Coord;
		
		double* _v0; double* _v1; double* _v2; double* _v3;
		double* _v4; double* _v5; double* _v6; double* _v7;
		
		bool _Open;
		bool _Closed;
		
		bool _CostIsComputed;
		double _Cost;
		
		GLint m_list;
		
		Eigen::Vector3d m_GradientDirection;
		
	};
	//Contenu du namespace
}


#endif // HRICELL_H

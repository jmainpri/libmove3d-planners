#ifndef HRICS_ROBOTBASEGRID_HPP
#define HRICS_ROBOTBASEGRID_HPP

#include "API/planningAPI.hpp"
#include "API/Grids/gridsAPI.hpp"

#include "HRI_costspace/HRICS_Natural.hpp"

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Grids/TwoDGrid.hpp"

namespace HRICS
{
	class Natural;
	class RobotBaseCell;
	/**
	 @ingroup HRICS
	 @brief Planar Grid for Robot base
	 */
	class RobotBaseGrid : public API::TwoDGrid
	{
	public:
		RobotBaseGrid();
		RobotBaseGrid(double pace, std::vector<double> envSize, Natural* costSpace);

		API::TwoDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y );

		void draw();

		Eigen::Transform3d getTransformFromRobotPos();
		Robot* getRobot();

		Natural* getNaturalCostSpace() { return m_NaturalCostSpace; }
		Eigen::Transform3d getRobotOrigin() { return m_RobotOriginPos; }

		std::vector<std::pair<double,Eigen::Vector3d> > getCells();

		void recomputeAllCosts();

	private:
		Natural* m_NaturalCostSpace;
		Eigen::Transform3d m_RobotOriginPos;
	};

	/**
	 @ingroup HRICS
	 @brief Plannar HRI Cell for Robot Base Grid
	 */
	class RobotBaseCell : public API::TwoDCell
	{
	public:
		RobotBaseCell();
		RobotBaseCell(int i, Eigen::Vector2i coord, Eigen::Vector2d corner, RobotBaseGrid* grid);

		~RobotBaseCell() { }

		double getCost();

		void setBlankCost() { mCostIsComputed = false; }

		void draw();

		Eigen::Vector3d getWorkspacePoint();

	private:
		Eigen::Vector2i _Coord;

		bool mCostIsComputed;
		double mCost;

	};
}

#endif // HRICS_ROBOTBASEGRID_HPP

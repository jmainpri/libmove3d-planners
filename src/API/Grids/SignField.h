/*
 *  SignField.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 23/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

class SignFeildCell;

class SignFeildGrid : public API::ThreeDGrid
{
public:
	Grid();
	Grid( std::vector<int> size );
	Grid(double pace, std::vector<double> envSize);
	
	API::ThreeDCell* createNewCell(unsigned int index, unsigned int x, unsigned int y, unsigned int z );
	void computeAllCellCost();
	
	void drawSpheres();
	void draw();
	void resetCellCost();
	
	void setRobot(Robot* rob) { _Robot = rob; }
	Robot* getRobot() { return _Robot; }
	
	bool isVirtualObjectPathValid(Cell* fromCell,Cell* toCell);
	
private:
	
	Robot* _Robot;
};
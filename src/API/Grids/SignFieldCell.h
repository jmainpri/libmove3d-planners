/*
 *  SignFieldCell.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 23/06/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef SIGNFIELDCELL_H
#define SIGNFIELDCELL_H

#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Geometry> 


class SignFieldCell : public API::ThreeDCell
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
	
};


#endif // SIGNFIELDCELL
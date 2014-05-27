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
#ifndef COST_STAT_HPP
#define COST_STAT_HPP

#include <fstream>
#include <string>
#include <sstream>

/**
  * @ingroup USER_APPLI
  * This class can store cost statistics over sevral run and compute avreages
  */
class StatCost 
{
private:
	p3d_graph* Graph;
	p3d_rob* Robot;

	std::vector<double> MinimalCost;
	std::vector<double> MaximalCost;
	std::vector<double> AverageCost;
	std::vector<double> SumCost;
	std::vector<double> PathLength;
	std::vector<double> Time;
	std::vector<double> nbNodes;
	std::vector<int> nbQRand;

	double aMinimalCost;
	double aMaximalCost;
	double aAverageCost;
	double aSumCost;
	double aPathLength;
	double aTime;
	double anbNodes;
	double anbQRand;

	double bMinimalCost;
	double bMaximalCost;
	double bAverageCost;
	double bSumCost;
	double bPathLength;

	std::ofstream s;

	std::vector<double> pene_MinimalCost;
	std::vector<double> pene_MaximalCost;
	std::vector<double> pene_AverageCost;
	std::vector<double> pene_SumCost;

	double pene_aMinimalCost;
	double pene_aMaximalCost;
	double pene_aAverageCost;
	double pene_aSumCost;

public:
	StatCost();
	StatCost(p3d_graph* g, p3d_rob* r);
	StatCost(p3d_rob* r);

	void setGraph(p3d_graph* g);
	void setValues();

	double average(std::vector<double> vect);
	double average(std::vector<int> vect);
	void setAverages();

	void setBestValues();

	void indexToFile(uint i);
	void saveToFile();
	void print();

};

#endif

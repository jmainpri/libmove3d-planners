#ifndef COST_STAT_HPP
#define COST_STAT_HPP

#include <fstream>
#include <string>
#include <sstream>

/**
  * @ingroup USER_APPLI
  * This class can store cost statistics over sevral run and compute avreages
  */
class StatCost {

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

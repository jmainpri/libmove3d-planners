#include "Planner-pkg.h"
#include <vector>
#include <iostream>
#include "StatCostStructure.hpp"

using namespace std;


StatCost::StatCost(){

}

StatCost::StatCost(p3d_rob* r){
	Robot = r;

	cout << " robot is set : " << r << endl;
}

StatCost::StatCost(p3d_graph* g, p3d_rob* r){
	Graph = g;
	Robot = r;

	cout << " graph is set : " << g << endl;
	cout << " robot is set : " << r << endl;
}

void StatCost::setGraph(p3d_graph* g){
	Graph = g;
}

void StatCost::setValues(){

//	p3d_ExtractBestTraj(Graph);

	double  totalCost = 0;
	double minCost = 0;
	double maxCost = 0;
	double Wsum = 0;
	int nbConfig = 0;

	p3d_traj* trajPt = Robot->tcur;
	Wsum =  p3d_getTrajCost(Robot,trajPt, &minCost, &maxCost, &totalCost,&nbConfig);
	cout << "Time = " << Graph->time << endl;


	MinimalCost.push_back(minCost);
	MaximalCost.push_back(maxCost);
	AverageCost.push_back(totalCost/((double)(nbConfig+1)));
	SumCost.push_back(Wsum);
	PathLength.push_back(trajPt->range_param);
	Time.push_back(Graph->time);

	nbNodes.push_back(Graph->nnode);

	nbQRand.push_back(ENV.getInt(Env::nbQRand));
	ENV.setInt(Env::nbQRand,0);

	if( ENV.getBool(Env::enableHri) /*ENV.getBool(Env::useHriDis) || ENV.getBool(Env::useHriNat)*/ ){

		double  totalCost = 0;
		double minCost = 0;
		double maxCost = 0;
		double Wsum = 0;
		int nbConfig = 0;

                printf("Old broken\n");
//		Wsum =  hri_zones.getPeneTrajCost(Graph, trajPt, &minCost, &maxCost, &totalCost,&nbConfig);

		pene_MinimalCost.push_back(minCost);
		pene_MaximalCost.push_back(maxCost);
		pene_AverageCost.push_back(totalCost/((double)(nbConfig+1)));
		pene_SumCost.push_back(Wsum);
	}
}

void StatCost::setAverages(){

	aMinimalCost = average(MinimalCost);
	aMaximalCost = average(MaximalCost);
	aAverageCost = average(AverageCost);
	aSumCost = average(SumCost);
	aPathLength = average(PathLength);
	aTime = average(Time);
	anbQRand = average(nbQRand);
	anbNodes = average(nbNodes);

	if( ENV.getBool(Env::enableHri) /*ENV.getBool(Env::useHriDis) || ENV.getBool(Env::useHriNat)*/ ){
		pene_aMinimalCost = average(pene_MinimalCost);
		pene_aMaximalCost = average(pene_MaximalCost);
		pene_aAverageCost = average(pene_AverageCost);
		pene_aSumCost = average(pene_SumCost);
	}
}

double StatCost::average(std::vector<double> vect){
	double tot=0;

	for(uint i=0;i<vect.size();i++)
		tot += vect.at(i);

	return tot/vect.size();
}

double StatCost::average(std::vector<int> vect){
	double tot=0;

	for(uint i=0;i<vect.size();i++)
		tot += vect.at(i);

	return tot/vect.size();
}

void StatCost::setBestValues(){

	p3d_graph* tmp_Graph = p3d_CreateDenseRoadmap(Robot);

//	p3d_ExtractBestTraj(tmp_Graph);

	double  totalCost = 0;
	double minCost = 0;
	double maxCost = 0;
	double Wsum = 0;
	int nbConfig = 0;

	bool setting_Nat = ENV.getBool(Env::useHriNat);
	bool setting_Dis = ENV.getBool(Env::useHriDis);

	ENV.setBool(Env::useHriDis,false);
	ENV.setBool(Env::useHriNat,false);

	p3d_traj* trajPt = Robot->tcur;
	Wsum =  p3d_getTrajCost(Robot,trajPt, &minCost, &maxCost, &totalCost,&nbConfig);

	ENV.setBool(Env::useHriDis,setting_Dis);
	ENV.setBool(Env::useHriNat,setting_Nat);

	bMinimalCost = minCost;
	bMaximalCost = maxCost;
	bAverageCost = totalCost/((double)(nbConfig+1));
	bSumCost = Wsum;
	bPathLength = trajPt->range_param;
}

void StatCost::indexToFile(uint i){
	s << i+1 << "," ;
	s << MinimalCost.at(i) << "," ;
	s << MaximalCost.at(i) << "," ;
	s << AverageCost.at(i) << "," ;
	s << SumCost.at(i) << "," ;
	s << PathLength.at(i) << "," ;
	s << nbQRand.at(i) << "," ;
	s << nbNodes.at(i) << "," ;
	s << Time.at(i) << ",";

	if( ENV.getBool(Env::enableHri) /*ENV.getBool(Env::useHriDis) || ENV.getBool(Env::useHriNat)*/ ){
		s << " ,";
		s << pene_MinimalCost.at(i) << "," ;
		s << pene_MaximalCost.at(i) << "," ;
		s << pene_AverageCost.at(i) << "," ;
		s << pene_SumCost.at(i) << "," ;
	}
	s << endl;
}

void StatCost::print(){

	cout << "Round Number" << "\t\t";
	cout << "MinimalCost" << "\t\t";
	cout << "MaximalCost" << "\t\t";
	cout << "AverageCost" << "\t\t";
	cout << "SumCost" << "\t\t";
	cout << "PathLengthr" << "\t\t";
	cout << "nbQRand" << "\t\t";
	cout << "nbNodes" << "\t\t";
	cout << "Time" << "\t\t";

	if( ENV.getBool(Env::enableHri) /*ENV.getBool(Env::useHriDis) || ENV.getBool(Env::useHriNat)*/ ){
		cout << " ,";
		cout << "pene MinimalCost" << "\t\t";
		cout << "pene MaximalCost" << "\t\t";
		cout << "pene AverageCost" << "\t\t";
		cout << "pene SumCost" << "\t\t";
	}

	cout << endl;

	for(uint i=0; i<Time.size();i++){
		cout << i+1 << "\t\t" ;
		cout << MinimalCost.at(i) << "\t\t" ;
		cout << MaximalCost.at(i) << "\t\t" ;
		cout << AverageCost.at(i) << "\t\t" ;
		cout << SumCost.at(i) << "\t\t" ;
		cout << PathLength.at(i) << "\t\t" ;
		cout << nbQRand.at(i) << "\t\t" ;
		cout << nbNodes.at(i) << "\t\t" ;
		cout << Time.at(i) << "\t\t" ;

		if( ENV.getBool(Env::enableHri) /*ENV.getBool(Env::useHriDis) || ENV.getBool(Env::useHriNat)*/ ){
			cout << " ,";
			cout << pene_MinimalCost.at(i) << "\t\t" ;
			cout << pene_MaximalCost.at(i) << "\t\t" ;
			cout << pene_AverageCost.at(i) << "\t\t" ;
			cout << pene_SumCost.at(i) << "\t\t" ;
		}
		cout << endl;
	}

}

void StatCost::saveToFile(){

	std::ostringstream oss;
	oss << ENV.getString(Env::nameOfFile) << ".csv";

	const char *res = oss.str().c_str();

	s.open(res);

	cout << "Opening Cost Stat file" << endl;

	s << "Round Number" << ",";
	s << "MinimalCost" << ",";
	s << "MaximalCost" << ",";
	s << "AverageCost" << ",";
	s << "SumCost" << ",";
	s << "PathLengthr" << ",";
	s << "nbQRand" << ",";
	s << "nbNodes" << ",";
	s << "Time" << ",";

	if( ENV.getBool(Env::enableHri) /*ENV.getBool(Env::useHriDis) || ENV.getBool(Env::useHriNat)*/ ){
		s << " ,";
		s << "pene MinimalCost" << ",";
		s << "pene MaximalCost" << ",";
		s << "pene AverageCost" << ",";
		s << "pene SumCost" << ",";
	}

	s<< endl;

	for(uint i=0; i<Time.size();i++){
		indexToFile(i);
	}

	setAverages();

	s<< endl;
	s<< endl;

	s << "Averages" << ",";
	s << aMinimalCost << "," ;
	s << aMaximalCost << "," ;
	s << aAverageCost << "," ;
	s << aSumCost << "," ;
	s << aPathLength << "," ;
	s << anbQRand << "," ;
	s << anbNodes << "," ;
	s << aTime << "," ;

	if( ENV.getBool(Env::enableHri) /*ENV.getBool(Env::useHriDis) || ENV.getBool(Env::useHriNat)*/ ){
			s << " ,";
			s << pene_aMinimalCost << "," ;
			s << pene_aMaximalCost << "," ;
			s << pene_aAverageCost << "," ;
			s << pene_aSumCost << "," ;
		}

	if(Robot->joints[1]->type == P3D_PLAN)
	{
		setBestValues();

		s<< endl;

		s << "Best From Grid" << ",";
		s << bMinimalCost << "," ;
		s << bMaximalCost << "," ;
		s << bAverageCost << "," ;
		s << bSumCost << "," ;
		s << bPathLength << "," ;
		s << "0" << "," ;
	}

	s.close();
}



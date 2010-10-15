/*
 * Statistics.hpp
 *
 *  Created on: Sep 14, 2009
 *      Author: jmainpri
 */

#ifndef STATISTICS_HPP_
#define STATISTICS_HPP_

/**
  * @ingroup USER_APPLI
  * Attempt to get a fine statistics class which would store all the information
  * of a planning algorithm (partly based on the Statistics structure
  */
class Statistics
{

public:
	Statistics();

	~Statistics();
	/**
	 * enableStats
	 *
	 * Enables the retrieval of
	 * statistical information on the algorithm
	 */
	void enableStats(void);

	/**
	 * disableStats
	 *
	 * Disables the retrieval of
	 * statistical information on the algorithm
	 */
	void disableStats(void);

	/**
	 * getStatStatus
	 *
	 * Get if the statistics retrieval
	 * is local enabled
	 * @return: TRUE if the module is enabled
	 */
	int getStatStatus(void);

	/**
	 * createStat
	 *
	 * Allocates a statistic data structure
	 * @return: the structure
	 */
	p3d_stat * createStat(void);

	/**
	 * destroyStat
	 *
	 * Frees a statistic data structure
	 * @return: the structure
	 */
	void destroyStat(p3d_stat ** s);

	/**
	 * initStats
	 *
	 * Resets all fields of the stat data structure to
	 */
	void initStats(p3d_stat * s);

	/**
	 * setTotalCountVar
	 *
	 * Copy all graph data into the stat data structure
	 */
	void setTotalCountVar(p3d_graph * graph);

	/**
	 * setTotalCountVar
	 *
	 * Merge tow data structures by adding the src to the dest
	 */
	void mergeStat(p3d_stat * src, p3d_stat * dest);

	/**
	 * getPathStat
	 *
	 * Gets path statistics and sets it to the stat data structure
	 */
	void getPathStat(p3d_stat * s, p3d_traj * path, int beforePost);

	/**
	 * printStatsGraph
	 *
	 * Prints the statistics from a stat structure
	 * not all printed statistics are taken from the structure
	 */
	void printStatsGraph(p3d_stat * s, int Print);

	/**
	 * printStatsEnv
	 *
	 * Same as the printStatgraph
	 * it sets a Average to note that the statistics are global
	 * Meant to be printed after several runs of the same algorithm
	 */
	void printStatsEnv(p3d_stat * s, int Print);

	/**
	 * addStatsToFile
	 *
	 * Same as the printStatgraph to a file
	 * in Comma-separated values (CSV)
	 */
	void addStatToFile(p3d_stat * s, FILE* Stat_output);

	/**
	 * openStatsFile
	 */
	FILE* openStatFile(FILE* Stat_output, char* s);

	/**
	 * closeStatsFile
	 */
	void closeStatFile(FILE* Stat_output);

	/**
	 * saveInStatToFile
	 */
	void saveInStatFile();

public:

	int loopNum; // Number of loops
	double preTime; // Pre-planning time
	double planTime; // Planning Time
	int planConfTot; // Total Number of Configurations
	int planConfCol; // Number of conf in collision
	int planConfFree; // Number of conf free
	int planConfAdd; // Number of conf added
	int planConfGuard; // Number of guardian
	int planConfConn; // Number of connectors
	int planEdge; // Number of edges in the graph
	int planCycle; // Number of cycles in the graph
	double cyclingTime; // Time spent to construct cycles
	int planNeigCall; // Number of call to neigh function
	int planLpNum; // Number of computed local paths
	double planLpLenght; // Local path length
	int planLpColNum; // Number of local path collision test
	double planLpStepSize; // Local path step size
	double postTime; // Post planning time
	int colNum; // Number of collisions
	int lenLpNumBeforePost; // Number of local path before post processing
	int lenLpNumAfterPost; // Number of local path after post processing
	double lenPathBeforePost; // Path length before postProcessing
	double lenPathAfterPost; // Path length after postProcessing
	unsigned long memory; // Memory usage
#ifdef MULTIGRAPH
	double mgTime;
	int mgNodes;
	int mgEdges;

};

#endif /* STATISTICS_HPP_ */

#ifndef MULTIRUNS_HPP
#define MULTIRUNS_HPP

#include <vector>
#include <string>

/**
  * Enables mutliple runs
  */
class MultiRun
{
public:
    MultiRun();

    void runMutliRRT();
    void runMutliGreedy();
    void runMutliSmooth();

private:
    void saveVectorToFile(int Context);
    void saveGraph(int i);
    void loadGraph();

    std::vector<std::string>                mNames;
    std::vector< std::vector<double> >      mVectDoubles;
    std::vector<double>                     mTime;

};

#endif // MULTIRUNS_HPP

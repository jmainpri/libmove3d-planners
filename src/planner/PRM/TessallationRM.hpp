#ifndef TESSALLATIONRM_HPP
#define TESSALLATIONRM_HPP

#include "PRM.hpp"
#include "API/Grids/TwoDGrid.hpp"

class TessallationRM : public Planner
{
public:
    /**
     * Creates a perturbation roadmap from a given robot
     * and a given graph
     * @param Robot
     * @param Graph
     */
    TessallationRM(Robot* R, Graph* G);

    /**
     * Deletes a perturbation planner
     */
    ~RRM();

    /**
      * Creates a graph for a plannar robot
      */
    void CreateTessalatedGraph();

protected:

    API::TwoDGrid grid_;

    confPtr_t m_qi;
    confPtr_t m_qf;
};

#endif // TESSALLATIONRM_HPP

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
#ifndef EDGE_HPP
#define EDGE_HPP

#include "API/Roadmap/node.hpp"
#include "API/ConfigSpace/localpath.hpp"

#ifndef _ROADMAP_H
struct edge;
#endif

namespace Move3D
{

class Graph;

/**
 * @ingroup CPP_API
 * @defgroup ROADMAP Roadmap
 * @brief Nodes, Edges and Graph
 */

/**
 @ingroup ROADMAP
 \brief Classe représentant une Edge d'un Graph
 @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */
class Edge
{
public:
    //contructor and destructor
    Edge(Graph* G, unsigned int i, unsigned j);

    /**
     * Constructeur de la classe
     * @param G le Graph pour laquel l'Edge est créée
     * @param E la structure d'edge qui sera stockée
     */
    Edge(Graph* G, edge* E);


    /**
     * Constructeur de la classe
     * @param G le Graph pour laquel l'Edge est créée
     * @param E la structure d'edge qui sera stockée
     */
    //    Edge(cpp_Graph* G, p3d_edge* E);

    /**
     * Constructeur de la classe
     * @param G le Graph pour lequel l'Edge est créée
     * @param N1 le Node initial de l'Edge
     * @param N2 le Node final de l'Edge
     * @param Long la longueur de l'Edge
     */
    Edge(Graph* G, Node* N1, Node* N2, bool compute_length, double& length, bool compute_cost, double& cost );

    /**
     * Destructeur de la classe
     */
    ~Edge();

    //Accessors
    /**
     * obtient la structure p3d_edge stockée
     * @return la structure p3d_edge stockée
     */
    edge* getEdgeStruct();

    /**
     * obtient le Graph pour lequel l'Edge est créée
     * @return le Graph pour lequel l'Edge est créée
     */
    Graph* getGraph();

    /**
     * obtient le Robot pour lequel l'Edge est créée
     * @return le Robot pour lequel l'Edge est créée
     */
    Robot* getRobot();

    /**
     * obtient la longueur de l'Edge
     * @return la longueur de l'Edge
     */
    double length();

    /**
     * obtient le Node initial de l'Edge
     * @return le Node initial de l'Edge
     */
    Node* getSource();

    /**
     * obtient le Node final de l'Edge
     * @return le Node final de l'Edge
     */
    Node* getTarget();

    /**
     * Computes the edge cost and returns it
     */
    double cost();

    /**
     * Get the LocalPath associated
     * with the edge
     */
    pathPtr_t getLocalPath();


    /**
     * Set the LocalPath associated
     * with the edge
     */
    void setLocalPath(pathPtr_t pathPtr);

    //--------------------------------------
    // BGL
    BGL_Edge	getDescriptor();

    void			setDescriptor(const BGL_Edge& E)
    {
        m_is_BGL_Descriptor_Valid=true;
        m_BGL_Descriptor=E;
    }

    void			unSetDescriptor()
    {
        m_is_BGL_Descriptor_Valid=false;
    }

private:
    edge*       m_Edge;

    bool        m_is_cost_computed;

    Node*		m_Source;
    Node*		m_Target;

    Graph*		m_Graph;
    Robot*		m_Robot;

    double		m_Long;

    bool        m_is_BGL_Descriptor_Valid;
    BGL_Edge    m_BGL_Descriptor;

    bool		m_is_LocalPath_Computed;

    MOVE3D_PTR_NAMESPACE::shared_ptr<LocalPath> m_path;
};

}

#endif



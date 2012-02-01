#ifndef EDGE_HPP
#define EDGE_HPP

#include "API/Roadmap/node.hpp"
#include "API/ConfigSpace/localpath.hpp"

class Graph;

#ifndef _ROADMAP_H
struct edge;
#endif

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
	Edge(Graph* G, Node* N1, Node* N2, double Long, bool compute_cost = true);
	
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
	double longueur();
	
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
	 * Computes the edge cost and returns it
	 */
	
	/**
	 * Get the LocalPath associated
	 * with the edge
	 */
	std::tr1::shared_ptr<LocalPath> getLocalPath();
  
  
  /**
	 * Set the LocalPath associated
	 * with the edge
	 */
	 void setLocalPath(std::tr1::shared_ptr<LocalPath> pathPtr);
	
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
	edge*			m_Edge;
	
	bool m_is_BGL_Descriptor_Valid;
	BGL_Edge m_BGL_Descriptor;
	
	Node*			m_Source;
	Node*			m_Target;
	
	Graph*		m_Graph;
	Robot*		m_Robot;
	
	double		m_Long;
	
	bool														m_is_LocalPath_Computed;
	std::tr1::shared_ptr<LocalPath> m_path;
};

#endif



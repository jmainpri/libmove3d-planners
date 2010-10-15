#ifndef NODE_HPP
#define NODE_HPP

#include <tr1/memory>

#include "API/ConfigSpace/configuration.hpp"

#ifndef _ROADMAP_H
struct node;
struct edge;
struct compco;
#endif
 
#include "BGL_Graph.hpp"

#ifndef EDGE_HPP
class Edge;
#endif

#ifndef COMPCO_HPP
class ConnectedComponent;
#endif

class Graph;

/**
 @ingroup ROADMAP
 \brief Classe représentant un Node d'un Graph
 @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */
class Node
{	
public:
	
  //Constructor and destructor
	Node();
	/**
	 * Constructeur de la classe
	 * @param G le Graph pour lequel le Node est créé
	 * @param C la Configuration stockée dans le Node
	 */
	Node(Graph* G, std::tr1::shared_ptr<Configuration> C, bool newCompco=true);
	
	/**
	 * Constructeur de la classe
	 * @param G le Graph pour lequelle Node est créé
	 * @param N la structure de p3d_node qui sera stockée
	 */
	Node(Graph* G, node* N);
	
	/**
	 * Constructeur de la classe
	 * @param G le Graph pour lequelle Node est créé
	 * @param N la structure de p3d_node qui sera stockée
	 */
	//Node(cpp_Graph* G, p3d_node* N);
	
	/**
	 * Copy the node
	 */
	Node(const Node& N);
	
	/**
	 * destructeur de la classe
	 */
	~Node();
	
	/**
	 * Deletes the p3d node without deleting the 
	 * configuration allready stored in a shared_ptr
	 */
	void deleteNode();
	
	/**
	 * Equal operator
	 */
	bool operator==(Node& N);
	
  //Accessors
	/**
	 * obtient la structure p3d_node
	 * @return la structure p3d_node
	 */
	node* getNodeStruct();
	
	/**
	 * obtient le Graph pour lequel le Node a été créé
	 * @return le GRaph pour lequel le Node a été créé
	 */
	Graph* getGraph();
	
	/**
	 * obtient le Robot pour lequel le Node a été créé
	 * @return le Robot pour lequel le Node a été créé
	 */
	Robot* getRobot();
	
	/**
	 * obtient la Configuration stockée
	 * @return la Configuration stockée
	 */
	std::tr1::shared_ptr<Configuration> getConfiguration();
	
	/**
	 * modifie la valeur de activ
	 * @param b la nouvelle valeur de activ
	 */
	void activ(bool b);
	/**
	 * obtient la valeur de activ
	 * @return la valeur de activ
	 */
	bool isActiv();
	
	/**
	 * Returns the node id
	 */
	unsigned int getId();
	
	/**
	 * Sets the Id
	 */
	void setId(unsigned int id);
	
	/**
	 * returns a reference to the node parent when in a tree like structure
	 * the pointer is editable
	 */
	Node*& parent() { return m_parent; }
	
	/**
	 * obtient le cout du Node
	 * @return le cout du Node
	 */
	double cost();
	
	/**
	 * Returns the Sum of cost along the path
	 */
	double& sumCost();
	
	/**
	 * obtient la temperature du Node
	 * @return la temperature du Node
	 */
	double getTemp();
	
	/**
	 * modifie la temperature du Node
	 * @param t la nouvelle temperature
	 */
	void setTemp(double t);
	
	/**
	 * obtient la distance au Node courant créé
	 * @return la distance au Node courant créé
	 */
	double getDist();
	
	/**
	 * calcule la distance entre deux Node et stocke la valeur dans DistNew
	 * @param N le Node à partir duquel on veut calculer la distance
	 * @return la distance entre les deux Node
	 */
	double dist(Node* N);
	
	/**
	 * teste si deux Node sont égaux
	 * @param N le Node à tester
	 * @return les Node sont égaux
	 */
	bool equal(Node* N);
	
	/**
	 * teste si deux Node sont dans la même composante connexe
	 * @param N le Node à tester
	 * @return les deux Node sont dans la même composante connexe
	 */
	bool inSameComponent(Node* N);
	
	/**
	 * Get Number of neighbors
	 */
	int getNumberOfNeighbors();
	
	/**
	 * Get All neighbors
	 */
	std::vector<Node*> getNeighbors();
	
	/**
	 * Get Number of Edges
	 */
	int getNumberOfEdges();
	
	/**
	 * Get All Edges
	 */
	std::vector<Edge*> getEdges();

	/**
	 * Returns the parent in a tree structure
	 */
	Node* getParent() { return NULL; }
	
	/**
	 * To be implemented for retro-compatibility
	 */
	Node* getSearchFrom() { return NULL; }
	Node* getSearchTo() { return NULL; }
	Edge* getEdgeFrom() { return NULL; }
	
	/**
	 * teste si deux Node peuvent être liés
	 * @param N le Node à tester
	 * @param dist la distance entre les Node
	 * @return les Node peuvent être liés
	 */
	bool isLinkable(Node* N, double* dist);
	
	/**
	 * vérifie si le poids max du Node a été atteint
	 */
	void checkStopByWeight();
	
	/**
	 * Gets the number of nodes
	 * in the connected componnent
	 */
	unsigned int getNumberOfNodesInCompco();
	
	/**
	 * obtient la structure de composante connexe à laquelle appartient le Node
	 * @return la structure de composante connexe à laquelle appartient le Node
	 */
	ConnectedComponent* getConnectedComponent() { return m_Compco; }
	
	/**
	 * Sets the compco of the node
	 * @param the connected component in which is the node
	 */
	void setConnectedComponent(ConnectedComponent* compco);
	
	/**
	 * detruit la composante connexe
	 */
	void deleteCompco();
	
	/**
	 * teste si la composante connexe a atteint le nombre max de Nodes
	 * @return la composante connexe a atteint le nombre max de Nodes
	 */
	bool maximumNumberNodes();
	
	/**
	 * connect un Node à la composante connexe
	 * @param N le Node à connecter
	 * @return le Node est connecté
	 */
	bool connectNodeToCompco(Node* N, double step);
	
	/**
	 * merge deux composantes connexes
	 * @param compco la composante connexe à merger
	 */
	void merge(Node* compco);
	
	/**
	 * teste si deux composante connexe sont égales
	 * @param compco la composante connexe à tester
	 * @return les deux composantes sont égales
	 */
	bool equalCompco(Node* compco);
	
	/**
	 * tire un Node aléatoirement dans la composante connexe
	 * @return le Node tiré
	 */
	Node* randomNodeFromComp();
	
	/**
	 * Method for EST
	 */
	void			setSelectCost(double Cost) { _SelectCost = Cost; }
	double		getSelectCost() { return _SelectCost; }
	void			setExpandFailed() { _nbExpan++;  }
	int				getNbExpandFailed() { return _nbExpan; }
	
	std::vector<Node*>& getSortedNodes() {return _SortedNodes;}
	void								setSortedNodes( std::vector<Node*>& nodes ) { _SortedNodes = nodes;}
	
	
	/**
	 * Prints the node to the standard output
	 */
	void print();
	
	//--------------------------------------
	// BGL
	BGL_Vertex	getDescriptor();
	void				setDescriptor(const BGL_Vertex& V);
	void				unSetDescriptor();
	
private:
	
	node* _Node;
	
	bool m_is_BGL_Descriptor_Valid;
	BGL_Vertex m_BGL_Descriptor;
	
	Graph* _Graph;
	Robot* _Robot;
	ConnectedComponent* m_Compco;
	
	// In tree graphs
	Node* m_parent;
	
	std::tr1::shared_ptr<Configuration> _Configuration;
	bool _activ;
	
	double _SelectCost;
	int _nbExpan;
	std::vector<Node*> _SortedNodes;
};

#endif

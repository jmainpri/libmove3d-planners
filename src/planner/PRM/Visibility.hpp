#ifndef VIS_PRM_HPP
#define VIS_PRM_HPP

#include "PRM.hpp"
/**
  @ingroup PRM

	\brief Classe représentant l'algorithme Vis_PRM
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
class Vis_PRM : public PRM
{
public:
    /**
     * Constructeur de la classe
     * @param WS Le WorkSpace de l'application
     */
    Vis_PRM(Robot* R, Graph* G);

    /**
     * Destructeur de la classe
     */
    ~Vis_PRM();
	
	/**
     * test si un node est linkable en suivant la visibilité
     * @param N le Node à lier
     * @param link in/out le nombre de composantes connexes auxquelles le node peut être lié
     * @return le vecteur des composantes connexes auxquelles le Node peut être lié
     */
    std::vector<Node*> isOrphanLinking(Node* N, int & link);
	
	/**
	 * crée un Node dans le graph en suivant la visibilité
	 * @param type le type de Node que l'on veut créé (gradien:0 ou connecteur:1 ou indifférent:2)
	 * @param ADDED in/out le nombre de Node créés
	 * @param nb_fail in/out le nombre d'échecs consecutifs
	 */
    void createOneOrphanLinking(int type, unsigned int & ADDED, int & nb_fail);
	
	/**
     * lie un Node en suivant la visibilité
     * @param N le Node à lier
     * @param type le type de Node que l'on veut ajouté (gradien:0 ou connecteur:1 ou indifférent:2)
     * @param ADDED in/out le nombre de Node ajoutés
     * @param nb_fail in/out le nombre d'échecs consecutifs
     * @return le Node est lié
     */
    bool linkOrphanLinking(Node* N, int type, unsigned int & ADDED, int & nb_fail);
	
	
	/**
     * crée des Node dans le Graph en suivant la visibilité
     * @param nb_node le nombre de Node à créer
     * @param type le type de Node que l'on veut créé (gradien:0 ou connecteur:1 ou indifférent:2)
     * @return le nombre de Node créés
     */
    int createOrphansLinking(unsigned int nb_node, int type);
	

    /**
     * fonction principale de l'algorithme Vis_PRM
	 * crée des Node dans le Graph en suivant la visibilité
     * @return le nombre de Node ajoutés au Graph
     */
    void expandOneStep();
	
private:
	unsigned int m_nbOfExpand;
};

#endif

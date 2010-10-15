#ifndef CELLPRM_HPP
#define CELLPRM_HPP

/**
  @ingroup PRM
  */
class CellPRM : public PRM
{
public:
    /**
     * Class constructor
     * @param The workspace
     */
    (WorkSpace* WS);

    /**
     * Class destructor
     */
    ~Vis_PRM();

    /**
     * fonction principale de l'algorithme Vis_PRM
     * @param Graph_Pt le graphPt affiché
     * @param (*fct_stop)(void) la fonction d'arret
     * @param (*fct_draw)(void) la fonction d'affichage
     * @return le nombre de Node ajoutés au Graph
     */
    uint expand(p3d_graph* Graph_Pt,int (*fct_stop)(void), void (*fct_draw)(void));
};


#endif // CELLPRM_HPP

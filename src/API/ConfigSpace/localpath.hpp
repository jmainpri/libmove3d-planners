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
#ifndef LOCALPATH_HPP
#define LOCALPATH_HPP

#include "API/ConfigSpace/configuration.hpp"

class localpath;

namespace Move3D {

class Robot;
class LocalPath ;

typedef MOVE3D_PTR_NAMESPACE::shared_ptr<LocalPath> pathPtr_t;

/**
 @ingroup CONFIG_SPACE
 \brief Classe représentant un chemin local
 @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */
class LocalPath 
{
public:
    /**
     * Class Constructor
     * The type is linear by default
     * @param B la Configuration initiale du LocalPath
     * @param E la Configuration finale du LocalPath
     */
    LocalPath( confPtr_t B, confPtr_t E );

    /**
     * smaller local path (for the extend method)
     */
    LocalPath( LocalPath& path, double& pathDelta , bool lastValidConfig = true );

    /**
     * Copy constructor
     * @param path a LocalPath
     */
    LocalPath( const LocalPath& path );

    /**
     * Constructor from a struct
     * @param p3d struct
     */
    LocalPath( Robot* R, localpath* lpPtr );

    /**
     * Destructor
     */
    ~LocalPath();

    //Accessor
    /**
     * obtient la structure p3d_localpath stockée
     * @return la structure p3d_localpath stockée
     */
    localpath* getP3dLocalpathStruct( bool multi_sol = false );

    /**
     * obtient la structure p3d_localpath stockée
     * @return la structure p3d_localpath stockée
     */
    localpath* getP3dLocalpathStructConst() const { return static_cast<localpath*>(_LocalPath); }

    /**
     * obtient la structure p3d_localpath stockée
     * @return la structure p3d_localpath stockée
     */
    void setP3dLocalpathStructConst(localpath* path) { _LocalPath = path; }

    /**
     * obtient la configuration initiale
     * @return la configuration initiale
     */
    confPtr_t getBegin();
    /**
     * obtient la configuration finale
     * @return la configuration finale
     */
    confPtr_t getEnd();

    /**
     * obtient le Graph pour lequel le LocalPath est créé
     * @return le Graph pour lequel le LocalPath est créé
     */
    //	Graph* getGraph();
    /**
     * obtient le Robot pour lequel le LocalPath est créé
     * @return le Robot pour lequel le LocalPath est créé
     */
    Robot* getRobot();

    /**
     * Returns the number of
     * Colision test done to test the local path
     */
    int getNbColTest();

    /**
     * Returns the number of
     * Colision test done to test the local path
     */
    int getNbCostTest();

    /**
     * teste si le LocalPath à été évalué
     * @return le LocalPath à été évalué
     */
    bool getEvaluated();

    /**
     * obtient le type de LocalPath
     * @return le type de LocalPath
     */
    // p3d_localpath_type getType();
    int  getType();

    /**
      * Sets the type of the local path
      */
    void setType( int type ) { _Type = type; }

    /**
     * obtient la dernière Configuration valide le long du LocalPath
     * @param p in/out le paramètre correspondant à la dernière Configauration valide
     * @return la dernière Configuration valide le long du LocalPath
     */
    confPtr_t getLastValidConfig( double& p );

    /**
     * Set the localpath as untested
     */
    void setLocalpathAsNotTested() { _Evaluated = false; }

    /**
     * teste si le LocalPath est valide
     * @return le LocalPath est valide
     */
    bool isValid();

    /**
     * test si le LocalPath est valide
     * @param R le Robot pour lequel le LocalPath est testé
     * @param ntest in/out le nombre de tests
     * @return ! le LocalPath n'est pas valide
     */
    // bool unvalidLocalpathTest( Robot* R, int* ntest );

    /**
   * Test the localpath using the classic method as opposed to dichotomic test
   * @return true if the localpath is valid
   */
    bool classicTest();

    /**
     * obtient la longueur du LocaPath
     * @return la longueur du LocalPath
     */
    double length();

    /**
     * Equivalent to the length when using linear interpolation
     */
    double getParamMax();

    /**
     * obtient une Configuration se trouvant à une distance donnée du début du LocalPath
     * @param R le Robot pour lequel le LocalPath est créé
     * @param dist la distance par rapport au début
     * @return la Configuration
     */
    confPtr_t configAtDist( double dist );

    /**
     * obtient une Configuration se trouvant sur le LocalPath à un paramètre donnée
     * @param R le Robot pour lequel le LocalPath est créé
     * @param param le paramètre
     * @return la Configuration
     */
    confPtr_t configAtParam( double param );

    /**
     * Stay within dist
     * From a parameter along the LocalPath and distance a vector of distance in WorkSapce
     * Stay within dist computes the maximum parameter that the robot can move
     * in free space
     */
    double stayWithInDistance(double u, bool goForward, double* distance);

    /**
     * Cost resolution for
     * integral and work along LocalPath
     */
    double getResolution(double step = 0.0);

    /**
     * Get number of cost segments
     */
    unsigned int getNumberOfCostSegments();

    /**
     * Returns the cost profile of the localPath
     */
    std::vector< std::pair<double,double> > getCostProfile();

    /**
     * Return param at which integral of cost
     * is higher than the input (going from begin to end)
     */
    double whenCostIntegralPasses(double thresh);

    /**
     * returns true if the cost is already evaluated
     */
    double isCostEvaluated() { return _costEvaluated; }

    /**
     * Gets the LocalPath cost
     */
    double cost();

    /**
     * Set the cost as computed and set the value (suposed to be tested outside
     */
    void setCost(double cost) { _Cost = cost; _costEvaluated = true; }

    /**
     * When reset the next cost querry will compute it
     */
    void resetCostComputed() { _costEvaluated = false; }

    /**
     * returns true if the cost is already evaluated
     */
    bool areFeaturesEvaluated() { return _phiEvaluated; }

    /**
     * Set the freature as computed and set the value (suposed to be tested outside
     */
    void setFeatures( const Eigen::VectorXd& phi ) { _phi = phi; _phiEvaluated = true; }

    /**
     * Returns the freature as computed and set the value (suposed to be tested outside
     */
    const Eigen::VectorXd& getFeatures() { return _phi; }

    /**
     * When reset the next feature querry will compute it
     */
    void resetFeaturesComputed() { _phiEvaluated = false; }

    /**
     * Set the ik sol to be used in the localplanner
     */
    void setIkSol(int* iksol) { _ikSol = iksol; }

    /**
     * Returns the ik solutions returned by the local planner
     */
    int* getIkSol() { return _ikSol; }

    /**
     * Prints the variables
     * inside the LocalPath
     */
    void print();

protected:

    Robot* _Robot;

    confPtr_t _Begin;
    confPtr_t _End;

private:

    void* _LocalPath;

    bool _Valid;
    bool _Evaluated;
    double _lastValidParam;
    confPtr_t _lastValidConfig;
    bool _lastValidEvaluated;
    int _NbColTest;

    int* _ikSol;

    bool _costEvaluated;
    double _Cost;
    int _NbCostTest;

    bool _ResolEvaluated;
    double _Resolution;

    // p3d_localpath_type _Type; //type du local path(mahantan, linear ...)
    int _Type;

    // Features
    bool _phiEvaluated;
    Eigen::VectorXd _phi;
};

}

void move3d_set_fct_localpath_copy( boost::function<void*(const Move3D::LocalPath&, Move3D::Robot*)> fct );
void move3d_set_fct_localpath_copy_p3d( boost::function<void*(const Move3D::LocalPath&, Move3D::Robot*)> fct );
void move3d_set_fct_localpath_path_destructor( boost::function<void(Move3D::LocalPath&)> fct );
void move3d_set_fct_localpath_copy_from_struct( boost::function<void*(Move3D::LocalPath&, void*, Move3D::confPtr_t&, Move3D::confPtr_t& )> fct );
void move3d_set_fct_localpath_get_struct( boost::function<void*(Move3D::LocalPath&, bool, int&)> fct );
void move3d_set_fct_localpath_classic_test( boost::function<bool(Move3D::LocalPath&, Move3D::confPtr_t, int&, double&)> fct );
void move3d_set_fct_localpath_is_valid( boost::function<bool(Move3D::LocalPath&, int&)> fct );
void move3d_set_fct_localpath_get_length( boost::function<double(Move3D::LocalPath&)> fct );
void move3d_set_fct_localpath_get_param_max( boost::function<double(Move3D::LocalPath&)> fct );
void move3d_set_fct_localpath_config_at_dist( boost::function<Move3D::confPtr_t(Move3D::LocalPath&,double)> fct );
void move3d_set_fct_localpath_config_at_param( boost::function<void(Move3D::LocalPath&,double,Move3D::confPtr_t&)> fct );
void move3d_set_fct_localpath_stay_within_dist( boost::function<double(Move3D::LocalPath&,double,bool,double&)> fct );

#endif

#ifndef LOCALPATH_HPP
#define LOCALPATH_HPP

#include "API/ConfigSpace/configuration.hpp"

class Robot;
class localpath;

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
    LocalPath(MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> B, MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> E);

    /**
     * smaller local path (for the extend method)
     */
    LocalPath(LocalPath& path, double& pathDelta , bool lastValidConfig = true);

    /**
     * Copy constructor
     * @param path a LocalPath
     */
    LocalPath(const LocalPath& path);

    /**
     * Constructor from a struct
     * @param p3d struct
     */
    LocalPath(Robot* R, localpath* lpPtr);

    /**
     * Destructor
     */
    ~LocalPath();

    //Accessor
    /**
     * obtient la structure p3d_localpath stockée
     * @return la structure p3d_localpath stockée
     */
    localpath* getLocalpathStruct(bool multi_sol = false);

    /**
     * obtient la configuration initiale
     * @return la configuration initiale
     */
    MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> getBegin();
    /**
     * obtient la configuration finale
     * @return la configuration finale
     */
    MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> getEnd();

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
     * obtient la dernière Configuration valide le long du LocalPath
     * @param p in/out le paramètre correspondant à la dernière Configauration valide
     * @return la dernière Configuration valide le long du LocalPath
     */
    MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> getLastValidConfig(double& p);

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
    bool unvalidLocalpathTest(Robot* R, int* ntest);

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
    MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> configAtDist(double dist);

    /**
     * obtient une Configuration se trouvant sur le LocalPath à un paramètre donnée
     * @param R le Robot pour lequel le LocalPath est créé
     * @param param le paramètre
     * @return la Configuration
     */
    MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> configAtParam(double param);

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
   * Set the ik sol to be used in the localplanner
   */
    void setIkSol(int* iksol) { _ikSol = iksol; }

    /**
     * Prints the variables
     * inside the LocalPath
     */
    void print();

protected:
    Robot* _Robot;

    MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> _Begin;
    MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> _End;

private:

    localpath* _LocalPath;

    bool _Valid;
    bool _Evaluated;
    double _lastValidParam;
    MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> _lastValidConfig;
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
};

#endif

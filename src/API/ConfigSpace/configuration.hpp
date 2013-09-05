#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API

#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <tr1/memory>

//#ifdef LINUX
//#define MOVE3D_USING_SHARED_PTR_NAMESPACE
//#define MOVE3D_PTR_NAMESPACE std
//#define MOVE3D_USING_BOOST_NAMESPACE
//#define MOVE3D_BOOST_PTR_NAMESPACE std::shared_ptr;
//#endif

#ifdef MACOSX
#define MOVE3D_USING_SHARED_PTR_NAMESPACE using namespace std::tr1;
#define MOVE3D_PTR_NAMESPACE std::tr1
#define MOVE3D_USING_BOOST_NAMESPACE using namespace boost;
#endif

#ifndef MOVE3D_USING_SHARED_PTR_NAMESPACE
#define MOVE3D_USING_SHARED_PTR_NAMESPACE using namespace std::tr1;
#define MOVE3D_PTR_NAMESPACE std::tr1
#define MOVE3D_USING_BOOST_NAMESPACE using namespace boost;
#define MOVE3D_BOOST_PTR_NAMESPACE boost::shared_ptr
#endif

class Robot;
class Configuration;

typedef MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> confPtr_t;

/**
 * @ingroup CPP_API
 * @defgroup CONFIG_SPACE Configuration space
 * @brief C-Space make generic motion planners possible
 */

/**
 @ingroup CONFIG_SPACE
 @brief Classe représentant une Configuration d'un Robot
 @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */
class Configuration {

public:
    //constructor and destructor
    //    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructeur de la classe
     * @param R le Robot pour lequel la Configuration est créée
     */
    Configuration(Robot* R);

    /**
     * Constructeur de la classe
     * @param R le Robot pour lequel la Configuration est créée
     * @param C la structure de Configuration qui sera stockée
     * @param noCopy if set to true, _Configuration is set to C,
     * otherwise a copy of C is made.
     */
    Configuration(Robot* R, double* C, bool noCopy = false);

    /**
     * Copy constructor of the class
     * @param confguration
     **/
    Configuration(const Configuration& conf);

    /**
     * Destructeur de la classe
     */
    ~Configuration();

    /**
     * Asignation
     */
    Configuration & operator= (const Configuration & other);

    /**
     * Acces the configuration
     */
    double& at(const int &i) { return _Configuration[i]; }

    /**
     * Acces the configuration
     */
    double& operator [] ( const int &i ) const { return _Configuration[i]; }

    /**
     * Acces the configuration
     */
    double& operator () ( const int &i ) const { return _Configuration[i]; }

    /**
     * détruie la configPt stockée
     */
    void Clear();

    //Accessors
    /**
     * obtient le Robot pour lequel la Configuration est créée
     * @return le Robot pour lequel la Configuration est créée
     */
    Robot* getRobot();

    /**
     * Gets the quaternion
     * @return le vecteur des Quaternions
     */
    Eigen::Quaterniond getQuaternion();

    /**
     * Sets EulersAngles
     */
    void setQuaternionsToEuler();

    /**
     * obtient le pointeur sur la ConfigPt
     * @return la pointeur sur la ConfigPt
     */
    double* getConfigStruct();

    /**
     * obtient le pointeur sur la ConfigPt
     * @return la pointeur sur la ConfigPt
     */
    double* getConfigStructCopy();

    /**
     * modifie la structure configPt stockée
     * @param C la nouvelle structure configPt
     */
    void setConfiguration(double* C);
    /**
     * modifie la structure configPt stockée
     * @param C la Configuration contentant la nouvelle structure
     */
    void setConfiguration(Configuration& C);

    /**
     * indique si le vecteur de Quaternions est initialisé
     * @return le vecteur de Quaternions est initialisé
     */
    //    bool isQuatInit();

    /**
     * initialise le vecteur de Quaternions
     */
    //    void initQuaternions();

    /**
     * Set Quaternions
     */
    //    void initQuaternions(int quatDof,Eigen::Quaternion<double> quat);

    /**
     * Convert Configuration in radian
     */
    void convertToRadian();

    /**
     * Get Config in degrees
     */
    MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> getConfigInDegree();

    /**
     * calcule la distance à une Configuration
     * @param Conf la Configuration entrée
     * @return la distance
     */
    double dist(Configuration& Conf, bool print=false);

    /**
     * calcule la distance à une Configuration
     * @param q la Configuration entrée
     * @param distChoice le type de calcul de distance
     * @return la distance
     */
    double dist(Configuration& q, int distChoice);

    /**
     * indique si la Configuration est en collision
     * @return la Configuration est en collision
     */
    bool isInCollision();

    /**
     * True is the configuration respects
     * DoFs bounds
     */
    bool isOutOfBounds(bool print = false);

    /**
     * True is the configuration respects
     * DoFs bounds
     */
    void adaptCircularJointsLimits();

    /**
     * Set the configuration as not tested
     */
    void setAsNotTested();


    /**
     * Compute the min distance to a CSpace obstacle
     * @return min dist
     */
    double distEnv();

    /**
     * Get the Ith Active Dof
     * @return the ith active DoF
     */
    double getActiveDoF(unsigned int ith);

    /**
     * Set the ith active DoF
     */
    void setActiveDoF(unsigned int ith, double value);

    /**
     * compare à une autre Configuration
     * @param Conf la Configuration entrée
     * @return les deux Configurations sont égales
     */
    bool equal(Configuration& Conf, bool print=false);

    /**
     * Compare tow configurations
     */
    bool operator==(Configuration& Conf) { return this->equal(Conf); }

    /**
     * Compare tow configurations
     */
    bool operator!=(Configuration& Conf) { return !(this->equal(Conf)); }

    /**
     * copie une Configuration
     * @return une copie de la Configuration
     */
    MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> copy();

    /**
     * copie les joints passifs de la Configuration courante dans la Configuration entrée
     * @param C in/out la Configuration à modifier
     */
    void copyPassive(Configuration& C);

    /**
     * obtient le cout de la Configuration suivant l'espace des fonctions de cout
     * @return le cout de la Configuration
     */
    double cost();

    /**
     * Set the configuration as not tested
     */
    void setCostAsNotTested();

    /**
     * Sets the configuration to respect robot constraints
     * Leaves the robot in the last configuration
     */
    bool setConstraintsWithSideEffect();

    /**
     * Sets the configuration to respect robot constraints
     */
    bool setConstraints();

#ifdef LIGHT_PLANNER
    /**
     * Sets the config constraints
     * and returns the task 3d pos (Virtual Object)
     */
    Eigen::Vector3d getTaskPos();
#endif

    /**
     *
     */
    confPtr_t add(Configuration& C);

    /**
     * Adds tow configurations
     */
    confPtr_t operator+(Configuration& Conf) { return this->add(Conf); }

    /**
     *
     */
    confPtr_t sub(Configuration& C);

    /**
     * Adds tow configurations
     */
    confPtr_t operator-(Configuration& Conf) { return this->sub(Conf); }

    /**
     *
     */
    Configuration& mult(double coeff);

    /**
     *
     */
    Configuration& operator*(double coeff) { return this->mult(coeff); }

    /**
     * Get the Eigen Vector of the configuration
     */
    Eigen::VectorXd getEigenVector();
    Eigen::VectorXd getEigenVector(int startIndex, int endIndex);
    Eigen::VectorXd getEigenVector(const std::vector<int>& indices);

    /**
     * set the Eigen Vector of the configuration
     */
    void setFromEigenVector(const Eigen::VectorXd& conf);
    void setFromEigenVector(const Eigen::VectorXd& conf, int startIndex, int endIndex);

    /**
     *
     */
    void print(bool withPassive = false);


private:
    bool _flagInitQuaternions;/*!< Booleen indiquant que les Quaternions ont été initialisés*/
    int _QuatDof;
    //        Eigen::Quaterniond _Quaternions;

    bool _CollisionTested;
    bool _InCollision;

    bool _CostTested;
    double _Cost;

    Robot* _Robot; /*!< Le Robot pour lequel la Configuration est créée*/
    double* _Configuration;/*!< une structure de congitPt contenant les données sur la Configuration*/
};

#endif

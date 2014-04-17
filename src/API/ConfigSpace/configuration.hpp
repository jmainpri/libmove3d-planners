#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API

#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <tr1/memory>
#include <boost/function.hpp>

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

namespace Move3D
{

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
    Move3D::Robot* getRobot();

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
    double* getConfigStruct() { return _Configuration; }

    /**
      * Returns a const pointer to the configuration
      * Should only be used when configuration is unchanged
      */
    double* getConfigStructConst() const;

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
     * modifie la structure configPt stockée
     * @param C la Configuration contentant la nouvelle structure
     */
    void setConfigurationCopy(Configuration& C);

    /**
     * Convert Configuration in radian
     */
    void convertToRadian();

    /**
     * Get Config in degrees
     */
    confPtr_t getConfigInDegree();

    /**
     * calcule la distance à une Configuration
     * @param Conf la Configuration entrée
     * @return la distance
     */
    double dist(const Configuration& Conf, bool print=false) const;

    /**
     * calcule la distance à une Configuration
     * @param q la Configuration entrée
     * @param distChoice le type de calcul de distance
     * @return la distance
     */
    double dist(const Configuration& q, int distChoice) const;

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
    confPtr_t copy();

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
    confPtr_t mult(double coeff);

    /**
     *
     */
    confPtr_t operator*(double coeff) { return this->mult(coeff); }

    /**
     * Get the Eigen Vector of the configuration
     */
    Eigen::VectorXd getEigenVector()  const;
    Eigen::VectorXd getEigenVector(int startIndex, int endIndex)  const;
    Eigen::VectorXd getEigenVector(const std::vector<int>& indices)  const;

    /**
     * set the Eigen Vector of the configuration
     */
    void setFromEigenVector(const Eigen::VectorXd& conf);
    void setFromEigenVector(const Eigen::VectorXd& conf, int startIndex, int endIndex);

    /**
     *
     */
    void print(bool withPassive = false) const;

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

}

void move3d_set_fct_configuration_constructor_robot( boost::function<double*(Move3D::Robot*)> fct );
void move3d_set_fct_configuration_constructor_config_struct( boost::function<double*(Move3D::Robot*, double*, bool)> fct );
void move3d_set_fct_configuration_assignment( boost::function<void(const Move3D::Configuration& q_s, Move3D::Configuration& q_t)> fct );
void move3d_set_fct_configuration_clear( boost::function<void(Move3D::Robot*,double*)> fct );
void move3d_set_fct_configuration_convert_to_radians( boost::function<void(Move3D::Robot*,double*)> fct );
void move3d_set_fct_configuration_convert_to_degrees( boost::function<Move3D::confPtr_t(Move3D::Robot*,double*)> fct );
void move3d_set_fct_configuration_get_struct_copy( boost::function<double*(Move3D::Robot*,double*)> fct );
void move3d_set_fct_configuration_dist( boost::function<double(Move3D::Robot*, double*, double*, bool)> fct );
void move3d_set_fct_configuration_dist_choice( boost::function<double( Move3D::Robot*, const Move3D::Configuration&, const Move3D::Configuration&, int)> fct );
void move3d_set_fct_configuration_in_collision( boost::function<bool(Move3D::Robot* R)> fct );
void move3d_set_fct_configuration_is_out_of_bounds( boost::function<bool(Move3D::Robot*,double*,bool)> fct );
void move3d_set_fct_configuration_adapt_circular_joint_limits( boost::function<void(Move3D::Robot*,double*)> fct );
void move3d_set_fct_configuration_dist_env( boost::function<double(Move3D::Robot*)> fct );
void move3d_set_fct_configuration_equal( boost::function<bool(Move3D::Robot*, double*, double*, bool)> fct );
void move3d_set_fct_configuration_copy( boost::function<void(Move3D::Robot*, double*)> fct );
void move3d_set_fct_configuration_copy_passive( boost::function<void(Move3D::Robot*, double*, double*)> fct );
void move3d_set_fct_configuration_add( boost::function<void(Move3D::Robot* R, double*, double*, double*)> fct );
void move3d_set_fct_configuration_sub( boost::function<void(Move3D::Robot* R, double*, double*, double*)> fct );
void move3d_set_fct_configuration_mult( boost::function<void(Move3D::Robot* R, double*, double*, double)> fct );
void move3d_set_fct_configuration_print( boost::function<void(Move3D::Robot* R, double*, bool)> fct );

#endif

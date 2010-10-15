#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <tr1/memory>

class Robot;

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
	std::tr1::shared_ptr<Configuration> getConfigInDegree();
	
	/**
	 * calcule la distance à une Configuration
	 * @param Conf la Configuration entrée
	 * @return la distance
	 */
	double dist(Configuration& Conf);
	
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
	bool isOutOfBounds();
	
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
	bool equal(Configuration& Conf);
	
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
	std::tr1::shared_ptr<Configuration> copy();
	
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
	std::tr1::shared_ptr<Configuration> add(Configuration& C);
	
	/**
	 * Adds tow configurations
	 */
	std::tr1::shared_ptr<Configuration> operator+(Configuration& Conf) { return this->add(Conf); }
	
	/**
	 *
	 */
	std::tr1::shared_ptr<Configuration> sub(Configuration& C);
	
	/**
	 * Adds tow configurations
	 */
	std::tr1::shared_ptr<Configuration> operator-(Configuration& Conf) { return this->sub(Conf); }
	
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
	
	/**
	 * set the Eigen Vector of the configuration
	 */
	void setFromEigenVector(const Eigen::VectorXd& conf); 
	
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
	
	Robot* _Robot;/*!< Le Robot pour lequel la Configuration est créée*/
	double* _Configuration;/*!< une structure de congitPt contenant les données sur la Configuration*/
};

#endif

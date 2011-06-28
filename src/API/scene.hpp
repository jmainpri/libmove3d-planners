#ifndef ENVIRONNEMENT_HPP
#define ENVIRONNEMENT_HPP

#include "API/Device/robot.hpp"
#include "API/Roadmap/graph.hpp"

#ifndef _ENVIRONMENT_H
struct env;
#endif

extern std::string global_ActiveRobotName;

/**
 @ingroup CPP_API
 
 @brief Class that represents a Scene,
 Described by a p3d file
 
 @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */
class Scene
{	
public:
	/**
	 * Constructeur de la classe
	 * @param name le nom de l'Scene
	 */
	Scene(env* environnment);
	
	/**
	 * Destructeur de la classe
	 */
	~Scene();
	
	/**
	 * obtient le nom de l'Scene
	 * @return le nom de l'Scene
	 */
	std::string getName();
	
	/**
	 * modifie le Robot actif
	 * @param name le nom du nouveau Robot actif
	 */
	void setActiveRobot(std::string name);
	
	/**
	 * obtient le Robot actif
	 * @return le Robot actif; NULL si le Robot ne peux pas être créé
	 */
	Robot* getActiveRobot();
	
	/**
	 * Returns the robot by id
	 */
	Robot* getRobot(unsigned int i) { return m_Robot[i]; }
	
	/**
	 * Returns the robot ID
	 */
	unsigned int getRobotId(std::string str);
	
	/**
	 * Get robot by name
	 */
	Robot* getRobotByName(std::string name);
	
	/**
	 * Get robot by name containing
	 */
	Robot* getRobotByNameContaining(std::string name);
	
	/**
	 * insert un nouveau Robot au vecteur des Robot
	 * @param R le nouveau Robot
	 */
	void insertRobot(Robot* R);
	
	/**
	 * Returns the number of Robots in the
	 * Scene
	 */
	unsigned int getNumberOfRobots() { return m_Robot.size(); }
	
	/**
	 * Get Active Graph
	 */
	Graph* getActiveGraph();
	
	/**
	 * Returns the scene resolution step DMax
	 */
	double getDMax();
  
  /**
   * Returns boundries of scene
   */
  std::vector<double> getBounds();
	
private:
	std::vector<Robot*> m_Robot;/*!< All Robots in the scene */
	std::string m_Name;/*!< The environnement name */
	
	env* m_Scene; 
	
};

#endif

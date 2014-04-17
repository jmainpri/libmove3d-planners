#ifndef ENVIRONNEMENT_HPP
#define ENVIRONNEMENT_HPP

#include "API/Device/robot.hpp"

extern std::string global_ActiveRobotName;

namespace Move3D
{

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
    Scene(void* environnment);

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
    void setActiveRobot(const std::string& name);

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
    int getRobotId(const std::string& str);

    /**
     * Get robot by name
     */
    Robot* getRobotByName(const std::string& name);

    /**
     * Get robot by name containing
     */
    Robot* getRobotByNameContaining(const std::string& name);

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
     * Returns the scene resolution step DMax
     */
    double getDMax();

    /**
     * Returns boundries of scene : x_min, x_max, y_min, y_max, z_min, z_max
     */
    std::vector<double> getBounds();

private:
    std::vector<Robot*> m_Robot;/*!< All Robots in the scene */
    std::string m_Name;/*!< The environnement name */

    void* m_Scene;

};

}

void move3d_set_fct_scene_constructor( boost::function<void*(void*, std::string&, std::vector<Move3D::Robot*>&, std::string& )> fct );
void move3d_set_fct_set_active_robot( boost::function<void( Move3D::Scene*, void*, const std::string& )> fct );
void move3d_set_fct_get_active_robot( boost::function<Move3D::Robot*( Move3D::Scene*, void* )> fct );
void move3d_set_fct_get_dmax( boost::function<double(void*)> fct );
void move3d_set_fct_get_bounds( boost::function<std::vector<double>( void* )> fct );

#endif

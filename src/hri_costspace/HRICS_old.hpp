#ifndef P3D_HRICOST_HPP
#define P3D_HRICOST_HPP

#include "API/Roadmap/node.hpp"
#include <libmove3d/include/P3d-pkg.h>

struct p3d_poly;
struct obj;

/**
 * Structure pour la sauvegarde du coût
 * */
namespace HRICS
{

typedef struct confCost {
    double* q;
    double cost;
} confCost;

/**
 * Classe Hri
 * */
class Hri
{
public:
    //robot
    rob* Robot;

    //Human
    rob* Human;

private:
    // Cost variables
    double c_to_goal;
    double c_penetra;
    double c_natural;
    double c_jlimits;
    double c_taskdist;

    // Max variables
    double max_to_goal;
    double max_taskdist;
    double max_jlimits;

    double cost;


    class zoneHri{

    public:
        // identifier for robot object
        int id;

        // nom du corps
        std::string name;

        // radius of the zone
        double radius;

        // position
        std::vector<double> position;

        // dist hri
        double dist_pene;

        zoneHri(){
            id = -1;
            name = "";
            radius = 0;
            position.clear();
            position.resize(3);
            dist_pene = std::numeric_limits<double>::max();
        }

        bool done() const {
            if( id==-1 || radius==0 )
                return false;
            else
                return true;
        }

        void reset(){
            zoneHri zone;
            *this = zone;
        }

        inline bool operator < (const zoneHri& zone) const
        {
            if (dist_pene < zone.dist_pene)
                return true;
            return false;
        }

    };

    std::vector<zoneHri> zones;

    //Vector for HRI closeness
    std::vector<double> vect_jim;

    //distances to points
    std::vector<double> dist_penetration;

    double dist_min;

    double safe_radius;
    double safe_offset;

    double color;

    double* fromComp;
    double* toComp;
    double* qConfort;

public:
    //Constructor
    Hri();
    Hri(double zone_size);

    void setNodes( Move3D::Node* fromComp, Move3D::Node* toComp );

    void parseEnvForZone();
    void parseEnvHoleHuman();
    void parseEnvBarre();
    void parseEnv();
    void changeColor();

    void offSetPrim(p3d_poly* poly,double offset);
    void computeMaxValues();
    /**
     * p3d_GetMinDistCost
     * Get the cost of a current configuration based
     */
    double getHriDistCost(rob* robotPt, int disp);
    double getDistZone(int zone, int disp);
    void getDistBody(int disp);
    void getDistHoleHuman(int disp);

    void *beg_zone_hri(char *name);

    obj* getObjectByName(char *name);

    void activateAll(int disp);
    void deactivateAllButHri(int disp);
    void deactivateHri(int disp);
    void deactivateAllButHriAchile(int zone,int disp);
    void activateAllAchile(int disp);

    /**
     * p3d_GetVectJim
     */
    std::vector<double>& getVectJim(void);

    /**
     * Gives the goal configuration
     */
    double* getGoalConfig( rob* robotPt );

    double	getPeneTrajCost(graph* graphPt, traj* trajPt,
                            double* minCostPt, double* maxCostPt,
                            double* totalCostPt, int* nbConfigPt);

    void getDistToGoalCost(int disp);
    void getNaturalLookingCost(int disp);
    void getSafeZoneCost(int disp);
    double getCustomDistConfig(double* q_i, double* q_f);

    void saveGPlotCostTraj(int iteration);
    void saveGPlotCostNodeChro(void);

    /**
     * Puts all costs in a tab
     * for 2D environment
     */
    void setCostToTab( rob* robotPt, confCost *tab , int nbPart );
    void writeConfCostToCsv(confCost *tab,int size);
    void writeConfCostToObPlane(confCost *tab,int size);

    void costTabFormFunction(confCost* tab);
    double costMapFunctions(double x, double y, int id_func);


};
}

//extern HRICS::Hri hri_zones;

#endif

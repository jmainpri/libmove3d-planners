#ifndef HRICS_OPENRAVE_HUMAN_MAP_HPP
#define HRICS_OPENRAVE_HUMAN_MAP_HPP

#include <string>
#include <map>

namespace HRICS
{

static std::map<std::string,int> herakles_move3d_map;
static std::map<std::string,int> herakles_openrave_map;

static inline void set_human_maps()
{
    herakles_move3d_map["PelvisTransX"]  =6;
    herakles_move3d_map["PelvisTransY"]  =7;
    herakles_move3d_map["PelvisTransZ"]  =8;
    herakles_move3d_map["PelvisRotX"]    =9;
    herakles_move3d_map["PelvisRotY"]    =10;
    herakles_move3d_map["PelvisRotZ"]    =11;
    herakles_move3d_map["TorsoX"]        =12;
    herakles_move3d_map["TorsoY"]        =13;
    herakles_move3d_map["TorsoZ"]        =14;
    herakles_move3d_map["HeadZ"]         =15;
    herakles_move3d_map["HeadY"]         =16;
    herakles_move3d_map["HeadX"]         =17;
    herakles_move3d_map["rShoulderX"]    =18;
    herakles_move3d_map["rShoulderZ"]    =19;
    herakles_move3d_map["rShoulderY"]    =20;
    herakles_move3d_map["rArmTrans"]     =21;
    herakles_move3d_map["rElbowZ"]       =22;
    herakles_move3d_map["rWristX"]       =24;
    herakles_move3d_map["rWristY"]       =25;
    herakles_move3d_map["rWristZ"]       =26;
    herakles_move3d_map["lShoulderX"]    =27;
    herakles_move3d_map["lShoulderZ"]    =28;
    herakles_move3d_map["lShoulderY"]    =29;
    herakles_move3d_map["lArmTrans"]     =30;
    herakles_move3d_map["lElbowZ"]       =31;
    herakles_move3d_map["lWristX"]       =33;
    herakles_move3d_map["lWristY"]       =34;
    herakles_move3d_map["lWristZ"]       =35;
    herakles_move3d_map["rHipX"]         =36;
    herakles_move3d_map["rHipY"]         =37;
    herakles_move3d_map["rHipZ"]         =38;
    herakles_move3d_map["rKnee"]         =39;
    herakles_move3d_map["rAnkleX"]       =40;
    herakles_move3d_map["rAnkleY"]       =41;
    herakles_move3d_map["rAnkleZ"]       =42;
    herakles_move3d_map["lHipX"]         =43;
    herakles_move3d_map["lHipY"]         =44;
    herakles_move3d_map["lHipZ"]         =45;
    herakles_move3d_map["lKnee"]         =46;
    herakles_move3d_map["lAnkleX"]       =47;
    herakles_move3d_map["lAnkleY"]       =48;
    herakles_move3d_map["lAnkleZ"]       =49;

    herakles_openrave_map["PelvisTransX"] =  0;
    herakles_openrave_map["PelvisTransY"] =  1;
    herakles_openrave_map["PelvisTransZ"] =  2;
    herakles_openrave_map["PelvisRotX"] =    3;
    herakles_openrave_map["PelvisRotY"] =    4;
    herakles_openrave_map["PelvisRotZ"] =    5;
    herakles_openrave_map["TorsoX"] =        6;
    herakles_openrave_map["TorsoY"] =        7;
    herakles_openrave_map["TorsoZ"] =        8;
    herakles_openrave_map["HeadZ"] =         9;
    herakles_openrave_map["HeadY"] =         10;
    herakles_openrave_map["HeadX"] =         11;
    herakles_openrave_map["rShoulderX"] =    12;
    herakles_openrave_map["rShoulderZ"] =    13;
    herakles_openrave_map["rShoulderY"] =    14;
    herakles_openrave_map["rArmTrans"] =     15;
    herakles_openrave_map["rElbowZ"] =       16;
    herakles_openrave_map["rWristX"] =       17;
    herakles_openrave_map["rWristY"] =       18;
    herakles_openrave_map["rWristZ"] =       19;
    herakles_openrave_map["lShoulderX"] =    20;
    herakles_openrave_map["lShoulderZ"] =    21;
    herakles_openrave_map["lShoulderY"] =    22;
    herakles_openrave_map["lArmTrans"] =     23;
    herakles_openrave_map["lElbowZ"] =       24;
    herakles_openrave_map["lWristX"] =       25;
    herakles_openrave_map["lWristY"] =       26;
    herakles_openrave_map["lWristZ"] =       27;
    herakles_openrave_map["rHipX"] =         28;
    herakles_openrave_map["rHipY"] =         29;
    herakles_openrave_map["rHipZ"] =         30;
    herakles_openrave_map["rKnee"] =         31;
    herakles_openrave_map["rAnkleX"] =       32;
    herakles_openrave_map["rAnkleY"] =       33;
    herakles_openrave_map["rAnkleZ"] =       34;
    herakles_openrave_map["lHipX"] =         35;
    herakles_openrave_map["lHipY"] =         36;
    herakles_openrave_map["lHipZ"] =         37;
    herakles_openrave_map["lKnee"] =         38;
    herakles_openrave_map["lAnkleX"] =       39;
    herakles_openrave_map["lAnkleY"] =       40;
    herakles_openrave_map["lAnkleZ "] =      41;
}

}

#endif // HRICS_OPENRAVE_HUMAN_MAP_HPP

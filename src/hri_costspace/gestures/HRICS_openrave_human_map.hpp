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
#ifndef HRICS_OPENRAVE_HUMAN_MAP_HPP
#define HRICS_OPENRAVE_HUMAN_MAP_HPP

#include <string>
#include <map>

namespace HRICS
{

static std::map<std::string,int> herakles_move3d_map;
static std::map<std::string,int> herakles_openrave_map;
static std::map<std::string,int> herakles_bio_move3d_map;
static std::map<std::string,int> herakles_bio_openrave_map;

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

    herakles_bio_move3d_map["PelvisTransX"]=6;
    herakles_bio_move3d_map["PelvisTransY"]=7;
    herakles_bio_move3d_map["PelvisTransZ"]=8;
    herakles_bio_move3d_map["PelvisRotX"]=9;
    herakles_bio_move3d_map["PelvisRotY"]=10;
    herakles_bio_move3d_map["PelvisRotZ"]=11;
    herakles_bio_move3d_map["TorsoX"]=12;
    herakles_bio_move3d_map["TorsoZ"]=13;
    herakles_bio_move3d_map["TorsoY"]=14;
    herakles_bio_move3d_map["HeadZ"]=15;
    herakles_bio_move3d_map["HeadY"]=16;
    herakles_bio_move3d_map["HeadX"]=17;
    herakles_bio_move3d_map["rShoulderTransX"]=18;
    herakles_bio_move3d_map["rShoulderTransY"]=19;
    herakles_bio_move3d_map["rShoulderTransZ"]=20;
    herakles_bio_move3d_map["rShoulderY1"]=21;
    herakles_bio_move3d_map["rShoulderX"]=22;
    herakles_bio_move3d_map["rShoulderY2"]=23;
    herakles_bio_move3d_map["rArmTrans"]=24;
    herakles_bio_move3d_map["rElbowZ"]=25;
    herakles_bio_move3d_map["rElbowX"]=26;
    herakles_bio_move3d_map["rElbowY"]=27;
    herakles_bio_move3d_map["rForeArmTrans"]=28;
    herakles_bio_move3d_map["rWristZ"]=29;
    herakles_bio_move3d_map["rWristX"]=30;
    herakles_bio_move3d_map["rWristY"]=31;
    herakles_bio_move3d_map["lShoulderX"]=32;
    herakles_bio_move3d_map["lShoulderZ"]=33;
    herakles_bio_move3d_map["lShoulderY"]=34;
    herakles_bio_move3d_map["lArmTrans"]=35;
    herakles_bio_move3d_map["lElbowZ"]=36;
    herakles_bio_move3d_map["lPoint"]=37;
    herakles_bio_move3d_map["lWristX"]=38;
    herakles_bio_move3d_map["lWristY"]=39;
    herakles_bio_move3d_map["lWristZ"]=40;
    herakles_bio_move3d_map["rHipX"]=41;
    herakles_bio_move3d_map["rHipY"]=42;
    herakles_bio_move3d_map["rHipZ"]=43;
    herakles_bio_move3d_map["rKnee"]=44;
    herakles_bio_move3d_map["rAnkleX"]=45;
    herakles_bio_move3d_map["rAnkleY"]=46;
    herakles_bio_move3d_map["rAnkleZ"]=47;
    herakles_bio_move3d_map["lHipX"]=48;
    herakles_bio_move3d_map["lHipY"]=49;
    herakles_bio_move3d_map["lHipZ"]=50;
    herakles_bio_move3d_map["lKnee"]=51;
    herakles_bio_move3d_map["lAnkleX"]=52;
    herakles_bio_move3d_map["lAnkleY"]=53;
    herakles_bio_move3d_map["lAnkleZ"]=54;

    herakles_bio_openrave_map["PelvisTransX"]=   0;
    herakles_bio_openrave_map["PelvisTransY"]=   1;
    herakles_bio_openrave_map["PelvisTransZ"]=   2;
    herakles_bio_openrave_map["PelvisRotX"]=     3;
    herakles_bio_openrave_map["PelvisRotY"]=     4;
    herakles_bio_openrave_map["PelvisRotZ"]=    5;
    herakles_bio_openrave_map["TorsoX"]=         6;
    herakles_bio_openrave_map["TorsoZ"]=         7;
    herakles_bio_openrave_map["TorsoY"]=         8;
    herakles_bio_openrave_map["rShoulderTransX"]= 9;
    herakles_bio_openrave_map["rShoulderTransY"]= 10;
    herakles_bio_openrave_map["rShoulderTransZ"]= 11;
//    herakles_bio_openrave_map["TorsoTrans"]=     12;
    herakles_bio_openrave_map["HeadZ"]=          13;
    herakles_bio_openrave_map["HeadY"]=          14;
    herakles_bio_openrave_map["HeadX"]=          15;
    herakles_bio_openrave_map["rShoulderY1"]=    16;
    herakles_bio_openrave_map["rShoulderX"]=     17;
    herakles_bio_openrave_map["rShoulderY2"]=    18;
    herakles_bio_openrave_map["rArmTrans"]=      19;
    herakles_bio_openrave_map["rElbowZ"]=        20;
    herakles_bio_openrave_map["rElbowX"]=        21;
    herakles_bio_openrave_map["rElbowY"]=        22;
    herakles_bio_openrave_map["rForeArmTrans"]=  23;
    herakles_bio_openrave_map["rWristZ"]=        24;
    herakles_bio_openrave_map["rWristX"]=        25;
    herakles_bio_openrave_map["rWristY"]=        26;
    herakles_bio_openrave_map["lShoulderX"]=     27;
    herakles_bio_openrave_map["lShoulderZ"]=     28;
    herakles_bio_openrave_map["lShoulderY"]=     29;
    herakles_bio_openrave_map["lArmTrans"]=      30;
    herakles_bio_openrave_map["lElbowZ "]=       31;
    herakles_bio_openrave_map["lWristX"]=        32;
    herakles_bio_openrave_map["lWristY"]=        33;
    herakles_bio_openrave_map["lWristZ"]=        34;
    herakles_bio_openrave_map["rHipX"]=          35;
    herakles_bio_openrave_map["rHipY"]=          36;
    herakles_bio_openrave_map["rHipZ"]=          37;
    herakles_bio_openrave_map["rKnee"]=          38;
    herakles_bio_openrave_map["rAnkleX"]=        39;
    herakles_bio_openrave_map["rAnkleY"]=        40;
    herakles_bio_openrave_map["rAnkleZ"]=        41;
    herakles_bio_openrave_map["lHipX"]=          42;
    herakles_bio_openrave_map["lHipY"]=          43;
    herakles_bio_openrave_map["lHipZ"]=          44;
    herakles_bio_openrave_map["lKnee"]=          45;
    herakles_bio_openrave_map["lAnkleX"]=        46;
    herakles_bio_openrave_map["lAnkleY"]=        47;
    herakles_bio_openrave_map["lAnkleZ"]=        48;

}

}

#endif // HRICS_OPENRAVE_HUMAN_MAP_HPP

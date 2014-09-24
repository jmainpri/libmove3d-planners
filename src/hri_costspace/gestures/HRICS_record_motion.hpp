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
#ifndef RECORDMOTION_HPP
#define RECORDMOTION_HPP

#include <vector>
#include <string.h>
#include <iostream>

#include "API/ConfigSpace/configuration.hpp"
#include "API/Trajectory/trajectory.hpp"

typedef std::vector< std::pair<double,Move3D::confPtr_t> > motion_t;

namespace HRICS
{

Move3D::Trajectory motion_to_traj( const motion_t& traj, Move3D::Robot* robot, int max_index=-1 );

inline double motion_duration( const motion_t& traj )
{
    double time=0.0;
    for( size_t i=1; i<traj.size(); i++ )
        time += traj[i].first;
    return time;
}

inline motion_t traj_to_motion( Move3D::Trajectory& traj, double duration )
{
    motion_t motion;
    double dt = duration / double(traj.getNbOfPaths());

    for( int i=0; i<traj.getNbOfViaPoints(); i++)
        motion.push_back( std::make_pair( dt, traj[i] ) );

    return motion;
}

class RecordMotion {

public:
    RecordMotion();
    RecordMotion( Move3D::Robot* robot );
    ~RecordMotion();

    void setRobot(const std::string& robotname);
    void saveCurrentConfig();
    void reset();
    void saveCurrentToFile();
    void saveToXml( const std::string& filename );
    void saveToXml( const std::string& filename, const motion_t& motion );

    motion_t loadFromXml( const std::string &filename );
    void loadMotionFromMultipleFiles( const std::string& baseFilename, int number_of_files );
    bool loadRegressedFromCSV( const std::string& foldername );
    void translateStoredMotions();
    void invertTranslationStoredMotions();
    motion_t invertTranslation( const motion_t& motion );
    Move3D::confPtr_t getConfigOpenRave( const std::vector<std::string>& config );
    Move3D::confPtr_t getConfigTwelveDoF( const std::vector<std::string>& config );
    std::pair<double,Move3D::confPtr_t> getConfigBio( const std::vector<std::string>& config );
    motion_t loadFromCSV( const std::string& filename, bool quiet = false );
    void loadXMLFolder();
    bool loadXMLFolder( const std::string& foldername  );
    void loadCSVFolder( const std::string& foldername, bool quiet = false, double threshold=0.0 );

    void storeMotion( const motion_t& motion, std::string name, bool new_motion = true);
    void addToCurrentMotion( const motion_t& motion );
    void saveToCSV( const std::string &filename, const motion_t& motion);
    void saveStoredToCSV( const std::string &filename, bool break_into_files );
    motion_t resample( const motion_t& motion, int nb_sample );
    void resampleAll( int nb_sample );
    motion_t getArmTorsoMotion( const motion_t& motion, Move3D::confPtr_t q );

    void showStoredMotion();
    void showCurrentMotion();
    void showMotion( const motion_t& motion );

    bool setRobotToStoredMotionConfig(int motion_id, int config_id);
    bool setRobotToConfiguration(int ith);
    bool setShowMotion(int ith);

    void drawMotion( const motion_t& motion, int nb_frames );
    void dawColorSkinedCylinder( const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    void drawHeraklesArms();
    void draw();

    motion_t extractSubpart(int init, int end );
    motion_t extractSubpart(int init, int end, const motion_t& motion);

    void incrementMotionId() { m_id_motion++; }

    const std::vector<motion_t>& getStoredMotions() { return m_stored_motions; }

    std::string getStoredMotionName(size_t i)
    {
        if( i < m_stored_motions_names.size() )
            return m_stored_motions_names[i];
        return "";
    }

    void useOpenRAVEFormat( bool use_or_format ) { m_use_or_format = use_or_format; }
    void useBioFormat( bool use_bio_format ) { m_use_bio_format = use_bio_format; }

    void setOffsetValue( double x, double y, double z, double rot ) { m_transX = x; m_transY = y; m_transY = z; m_transR = rot; }
    Eigen::Transform3d getOffsetTransform();

    bool m_is_recording;

private:

    void intialize();

    Move3D::Robot* m_robot;
    Move3D::confPtr_t m_init_q;
    double m_time_last_saved;
    double m_time_to_record;
    double m_time_last_record;
    int m_id_file;
    int m_id_motion;
    motion_t m_motion;
    std::vector<motion_t> m_stored_motions;
    std::vector<std::string> m_stored_motions_names;
    int m_ith_shown_motion;
    bool m_use_or_format;
    bool m_use_bio_format;

    // offset
    double m_transX;
    double m_transY;
    double m_transZ;
    double m_transR;
};
}

extern std::vector<HRICS::RecordMotion*> global_motionRecorders;
extern void show_recorded_human_motions();

#endif // RECORDMOTION_HPP

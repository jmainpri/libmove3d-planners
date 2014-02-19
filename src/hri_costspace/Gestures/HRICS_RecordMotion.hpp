#ifndef RECORDMOTION_HPP
#define RECORDMOTION_HPP

#include <vector>
#include <string.h>

#include "API/ConfigSpace/configuration.hpp"

typedef std::vector< std::pair<double,confPtr_t> > motion_t;

namespace HRICS
{
class RecordMotion {

public:
    RecordMotion();
    RecordMotion( Robot* robot );
    ~RecordMotion();

    void setRobot(const std::string& robotname);
    void saveCurrentConfig();
    void reset();
    void saveCurrentToFile();
    void saveToXml( const std::string& filename );
    void saveToXml( const std::string& filename, const motion_t& motion );

    motion_t loadFromXml( const std::string &filename );
    void loadMotionFromMultipleFiles( const std::string& baseFilename, int number_of_files );
    bool loadRegressedFromCSV();
    void translateStoredMotions();
    motion_t invertTranslation( const motion_t& motion );
    confPtr_t getConfigOpenRave( const std::vector<std::string>& config );
    confPtr_t getConfigTwelveDoF( const std::vector<std::string>& config );
    motion_t loadFromCSV( const std::string& filename, bool quiet = false );
    void loadXMLFolder();
    bool loadXMLFolder( const std::string& foldername  );
    void loadCSVFolder( const std::string& foldername, bool quiet = false );

    void storeMotion( const motion_t& motion, bool new_motion = true);
    void addToCurrentMotion( const motion_t& motion );
    void saveToCSV( const std::string &filename, const motion_t& motion);
    void saveStoredToCSV( const std::string &filename );
    motion_t resample( const motion_t& motion, int nb_sample );
    motion_t getArmTorsoMotion( const motion_t& motion, confPtr_t q );

    void showStoredMotion();
    void showCurrentMotion();
    void showMotion( const motion_t& motion );

    bool setRobotToStoredMotionConfig(int motion_id, int config_id);
    bool setRobotToConfiguration(int ith);
    bool setShowMotion(int ith);

    void drawMotion( const motion_t& motion );
    void dawColorSkinedCylinder( const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    void drawHeraklesArms();
    void draw();

    motion_t extractSubpart(int init, int end );
    motion_t extractSubpart(int init, int end, const motion_t& motion);

    void incrementMotionId() { m_id_motion++; }

    const std::vector<motion_t>& getStoredMotions() { return m_stored_motions; }

    bool m_is_recording;

private:
    Robot* m_robot;
    confPtr_t m_init_q;
    double m_time_last_saved;
    double m_time_to_record;
    double m_time_last_record;
    int m_id_file;
    int m_id_motion;
    motion_t m_motion;
    std::vector<motion_t> m_stored_motions;
    int m_ith_shown_motion;
    bool m_use_or_format;
};
}

extern std::vector<HRICS::RecordMotion*> global_motionRecorders;
extern void show_recorded_human_motions();

#endif // RECORDMOTION_HPP

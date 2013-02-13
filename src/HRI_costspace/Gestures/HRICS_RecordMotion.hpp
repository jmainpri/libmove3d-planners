#ifndef RECORDMOTION_HPP
#define RECORDMOTION_HPP

#include <vector>
#include <string.h>
#include "API/ConfigSpace/configuration.hpp"

typedef std::vector< std::pair<double,confPtr_t> > motion_t;

class RecordMotion {

public:
    RecordMotion();
    RecordMotion(const std::string& robotname);
    ~RecordMotion();

    void setRobot(const std::string& robotname);
    void saveCurrentConfig();
    void reset();
    void saveCurrentToFile();
    void saveToXml( const std::string& filename );
    void saveToXml( const std::string& filename, const motion_t& motion );

    motion_t loadFromXml(const std::string &filename);
    void loadMotionFromMultipleFiles( const std::string& baseFilename, int number_of_files );

    void storeMotion( const motion_t& motion, bool new_motion = true);
    void addToCurrentMotion( const motion_t& motion );
    void saveToCSV(const std::string &filename,const motion_t& motion);
    void saveStoredToCSV( const std::string &filename );

    void showStoredMotion();
    void showCurrentMotion();
    void showMotion( const motion_t& motion );
    bool setConfiguration(int ith);

    motion_t extractSubpart(int init, int end );
    motion_t extractSubpart(int init, int end, const motion_t& motion);

    void incrementMotionId() { m_id_motion++; }

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
};

extern RecordMotion* global_motionRecorder;

#endif // RECORDMOTION_HPP

#ifndef HRICS_PLAYMOTION_HPP
#define HRICS_PLAYMOTION_HPP

#include "HRICS_RecordMotion.hpp"

namespace HRICS
{

class PlayMotion {

public:
    PlayMotion( const std::vector<HRICS::RecordMotion*>& recorders);

    void play(int id);
    void play(const std::vector<std::string>& filepaths);

//    void setDirection(const bool dir);
    void setStep(const int step);
    void setControlled(const bool controlled);
    void setRecentInput(const bool input);
    bool getRecentInput();
    int getCurrentFrame();

private:

    void runRealTime(int id);
    void runControlled();

    std::vector<HRICS::RecordMotion*> _motion_recorders;

    int _current_frame;
    int _step_size;
    bool _play_controlled;
    bool _recent_input;

};

}

#endif // HRICS_PLAYMOTION_HPP

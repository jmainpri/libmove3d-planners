#include "HRICS_PlayMotion.hpp"
#include "planEnvironment.hpp"
#include "Graphic-pkg.h"
#include <sys/time.h>

using namespace HRICS;
using std::cout;
using std::endl;

PlayMotion::PlayMotion( const std::vector<HRICS::RecordMotion*>& recorders )
{
    _play_controlled = false;
    _motion_recorders = recorders;
}

//void PlayMotion::setDirection(const bool dir) { _play_dir = dir; }
void PlayMotion::setStep(const int step) { _step_size = step; }
void PlayMotion::setControlled(const bool controlled) { _play_controlled = controlled; }
void PlayMotion::setRecentInput(const bool input) { _recent_input = input;}
bool PlayMotion::getRecentInput() { return _recent_input;}
int PlayMotion::getCurrentFrame() { return _current_frame; }


void PlayMotion::play( const std::vector<std::string>& filepaths )
{
    if( filepaths.size() > _motion_recorders.size() )
    {
        cout << "The number of motion recorder is reater than te number of motion to play" << endl;
        return;
    }

    for (int i=0; i<int(filepaths.size()); i++)
    {
        _motion_recorders[i]->storeMotion( _motion_recorders[i]->loadFromCSV(filepaths[i]), true );
    }

    play(0);
}

void PlayMotion::play(int id)
{
    if ( _play_controlled )
        runControlled();
    else
        runRealTime(id);
}

void PlayMotion::runRealTime(int id)
{
    if( _motion_recorders.empty() ) {
        return;
    }

    int StopRun = false;
    _current_frame=0;
    double tu_last = 0.0;
    double dt = 0.0;

    while ( !StopRun )
    {
        timeval tim;
        gettimeofday(&tim, NULL);
        double tu = tim.tv_sec+(tim.tv_usec/1000000.0);
        dt += ( tu - tu_last );
        tu_last = tu;

        if ( dt>=0.025 )
        {
            for (int j=0; j<int(_motion_recorders.size()); j++)
            {
                _motion_recorders[j]->setRobotToStoredMotionConfig( id, _current_frame );
            }

            dt = 0.0;
            _current_frame++;
        }

        g3d_draw_allwin_active();

//        if( _current_frame == 1 )
//        {
//            StopRun = true;
//        }

        if ( PlanEnv->getBool(PlanParam::stopPlanner) ) {
            StopRun = true;
        }

        if ( _current_frame >= int(_motion_recorders[0]->getStoredMotions()[id].size()))
        {
            StopRun = true;
        }
    }

    cout << "End play motion" << endl;
    return;
}

void PlayMotion::runControlled() //TODO weird implementation. should be fixed at some point, but definitely working.
{
    int numFrames = int(_motion_recorders[0]->getStoredMotions()[0].size());

    if( _motion_recorders.empty() )
    {
        return;
    }

    int StopRun = false;
    _current_frame = 0;
    _recent_input = false;

    while ( !StopRun )
    {

        for (int j=0; j<int(_motion_recorders.size()); j++)
        {
            _motion_recorders[j]->setRobotToStoredMotionConfig(0,_current_frame);
        }

        if (_recent_input) _current_frame+=_step_size; //I don't it working like this.  Shouldn't constantly be updatin to current frame.

        if (_current_frame >= numFrames) _current_frame = numFrames; //Bounds check
        if (_current_frame < 0) _current_frame = 0; //Bounds check

        if ( _current_frame >= numFrames)
        {
            StopRun = true;
        }

        _recent_input = false;

    }

    cout << "End play motion" << endl;
    return;
}

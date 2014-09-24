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
#include "HRICS_play_motion.hpp"
#include "HRICS_record_motion.hpp"
#include "planEnvironment.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <sys/time.h>

using namespace HRICS;
using std::cout;
using std::endl;

PlayMotion::PlayMotion( const std::vector<HRICS::RecordMotion*>& recorders )
{
    _play_controlled = false;
    _motion_recorders = recorders;
}

PlayMotion::PlayMotion( const std::vector<std::vector<motion_t> >& motions)
{
    _play_controlled = false;
    _stored_motions = motions;
}

//void PlayMotion::setDirection(const bool dir) { _play_dir = dir; }
void PlayMotion::setStep(const int step) { _step_size = step; }
void PlayMotion::setControlled(const bool controlled) { _play_controlled = controlled; }
void PlayMotion::setRecentInput(const bool input) { _recent_input = input;}
bool PlayMotion::getRecentInput() { return _recent_input;}
int PlayMotion::getCurrentFrame() { return _current_frame; }

int PlayMotion::getNumberOfMotions() const
{
    if( _stored_motions.empty() && _motion_recorders.empty() )
        return 0;

    if( _stored_motions.empty() )
        return _motion_recorders[0]->getStoredMotions().size();

    if( _motion_recorders.empty() )
        return _stored_motions[0].size();

    return 0;
}

void PlayMotion::play( const std::vector<std::string>& filepaths )
{
    if( filepaths.size() > _motion_recorders.size() )
    {
        cout << "The number of motion recorder is reater than te number of motion to play" << endl;
        return;
    }

    for (int i=0; i<int(filepaths.size()); i++)
    {
        _motion_recorders[i]->storeMotion( _motion_recorders[i]->loadFromCSV(filepaths[i]), "", true );
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

//! id is the motion
void PlayMotion::runRealTime(int id)
{
    if( _motion_recorders.empty() && _stored_motions.empty() ) {
        cout << "no stored motions to play" << endl;
        return;
    }

    int StopRun = false;
    _current_frame=0;
    double tu_last = 0.0;
    double dt = 0.0;
    double time = 0.0;

    cout << "start playback" << endl;

    if( id < _motions_names.size() && _motion_recorders.empty() ){
        cout << " motion : " << _motions_names[id] << " , duration : " << motion_duration( _stored_motions[0][id] ) <<  endl;
    }

    while( !StopRun )
    {
        timeval tim;
        gettimeofday(&tim, NULL);
        double tu = tim.tv_sec+(tim.tv_usec/1000000.0);
        if( tu_last == 0.0 ) tu_last = tu;

        dt += ( tu - tu_last );
        tu_last = tu;

        if ( dt>=0.025 )
        {
//            cout << "dt >= 0.025 : " << dt << endl;

            time += dt;
            dt = 0.0;

            if( _stored_motions.empty() )
            {
                // TODO switch to time
                for (size_t j=0; j<_motion_recorders.size(); j++)
                {
                    _motion_recorders[j]->setRobotToStoredMotionConfig( id, _current_frame );
                }
            }
            else if( _motion_recorders.empty() )
            {
                for (size_t j=0; j<_stored_motions.size(); j++) // for each human or robot
                {
                    int i =0;
                    double time_traj = 0.0;

                    while( i < _stored_motions[j][id].size() )
                    {
                        time_traj += _stored_motions[j][id][i].first; // compute time along trajectory

                        if( time_traj >= time )
                        {
//                            cout << "i : " << i << " , :  _stored_motions[j][id].size() "  << _stored_motions[j][id].size() << " , time_traj : " << time_traj << " , time : " << time  << endl;

                            Move3D::confPtr_t q = _stored_motions[j][id][i].second;
                            Move3D::Robot* robot = q->getRobot();
                            robot->setAndUpdate( *q );
                            break;
                        }

                        i++;
                    }
                    if( i >= _stored_motions[j][id].size() ){
                        cout << "reach end of trajectory" << endl;
                        StopRun = true;
                    }
                }
            }
            _current_frame++;
        }

        g3d_draw_allwin_active();

        //        if( _current_frame == 1 )
        //        {
        //            StopRun = true;
        //        }

        if ( PlanEnv->getBool(PlanParam::stopPlanner) ) {
            cout << "stopped by user" << endl;
            StopRun = true;
        }


        // USING FRAMES
//        if( _stored_motions.empty() )
//        {
//            for (size_t j=0; j<_motion_recorders.size(); j++)
//            {
//                if ( _current_frame >= int(_motion_recorders[j]->getStoredMotions()[id].size()))
//                {
//                    StopRun = true;
//                }
//            }
//        }
//        else if( _motion_recorders.empty() )
//        {
//            for (size_t j=0; j<_stored_motions.size(); j++)
//            {
//                if ( _current_frame >= int(_stored_motions[j][id].size()))
//                {
//                    StopRun = true;
//                }
//            }
//        }
    }

    cout << "End play motion" << endl;
    return;
}

void PlayMotion::runControlled() // TODO weird implementation. should be fixed at some point, but definitely working.
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

        if (_recent_input) _current_frame += _step_size; //I don't it working like this.  Shouldn't constantly be updatin to current frame.

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

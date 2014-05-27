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
#ifndef HRICS_PLAYMOTION_HPP
#define HRICS_PLAYMOTION_HPP

#include "HRICS_record_motion.hpp"

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

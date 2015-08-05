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

#include "ik_generator.hpp"

#include "hri_costspace/HRICS_costspace.hpp"

#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Graphic-pkg.h>

using namespace Move3D;
using std::cout;
using std::endl;

IKGenerator::IKGenerator(Robot* robot) : GeneralIK(robot)
{

}

Move3D::confPtr_t IKGenerator::sample(Move3D::confPtr_t q, double variance_factor)
{
    double jmin, jmax;

    Move3D::confPtr_t q_tmp = q->copy();

    for(size_t j=0; j<active_joints_.size(); j++)
    {
        Move3D::Joint* jntPt = active_joints_[j];

        for(size_t i=0; i<jntPt->getNumberOfDof(); i++)
        {
           int k = jntPt->getIndexOfFirstDof() + i;

            if( jntPt->isJointDofUser(i) )
            {
                jntPt->getDofRandBounds( i, jmin, jmax );

                if ( std::fabs(jmax - jmin) > 1e-6 )
                {
                    double alpha=1.0;
                    double var = ( std::fabs( jmax - jmin) / 2 ) * variance_factor;
                    do
                    {
                        (*q_tmp)[k] = p3d_gaussian_random2( (*q)[k], alpha*var );
//                        cout << "(*q_tmp)[" << k << "] = " << (*q_tmp)[k] << " , jmax : "  << jmax << " , jmin : " << jmin << endl;
//                        alpha *= 0.9;
                    }
                    while (( jmin >= (*q_tmp)[k] ) || ((*q_tmp)[k] >= jmax));
                }
            }
        }
    }

    return q_tmp;
}

bool IKGenerator::generate(const Eigen::VectorXd& xdes)
{
    int nb_iterations=100;
    int nb_samples=10;
    double temperature=0.3;
    double alpha=temperature/(nb_iterations+1);
    std::vector< std::pair<double, Move3D::confPtr_t> > configs;
    std::vector< std::pair<double, Move3D::confPtr_t> > best;
    best.push_back( std::make_pair( std::numeric_limits<double>::max(), robot_->getCurrentPos() ) );


    for(int i=0; i<nb_iterations; i++)
    {
        cout << "temp : " << temperature << endl;

        configs.clear();

        for(int j=0; j<nb_samples; j++)
        {
            Move3D::confPtr_t q_tmp =
                    sample( best[ j<int(best.size()) ? j : 0 ].second,
                    temperature );

            robot_->setAndUpdate(*q_tmp);
//            q_tmp->print();

//            cout << "SOLVE IK" << endl;
            if( solve(xdes) )
            {
                q_tmp = robot_->getCurrentPos();
                q_tmp->adaptCircularJointsLimits();
                robot_->setAndUpdate(*q_tmp);
                configs.push_back( std::make_pair( q_tmp->cost(), q_tmp ) );
                cout.precision(3);

//                cout << std::scientific << "cost : " << configs.back().first << " iter : ( " << i << " , " << j << " )" <<  endl;
            }
            else{
//                cout << std::scientific << "no ik iter : ( " << i << " , " << j << " )" <<  endl;
            }
        }

        if( !configs.empty() )
        {
            std::sort( configs.begin(), configs.end() );

//            robot_->setAndUpdate(*configs[0].second);
//            HRICS_activeNatu->setRobotColorFromConfiguration( true );
//            g3d_draw_allwin_active();

            if( configs[0].first < best[0].first ){
                cout << "improvement at : " << i << ", cost " << configs[0].first << endl;
            }

            best.push_back( configs[0] );
            std::sort( best.begin(), best.end() );
            if( best.size() > size_t(nb_samples) )
                best.resize( nb_samples );
        }
        temperature -= alpha;
    }

    cout << "END" << endl;

    if( best[0].first != std::numeric_limits<double>::max() )
    {
        robot_->setAndUpdate( *best[0].second );
        HRICS_activeNatu->setRobotColorFromConfiguration( true );
        g3d_draw_allwin_active();
        return true;
    }

    return false;
}

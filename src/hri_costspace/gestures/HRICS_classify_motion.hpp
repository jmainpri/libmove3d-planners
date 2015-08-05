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
#ifndef HRICS_CLASSIFYMOTION_HPP
#define HRICS_CLASSIFYMOTION_HPP

////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API

#include <string>

#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace HRICS
{
    class ClassifyMotion
    {
    public:
        ClassifyMotion();
        ~ClassifyMotion();

        bool load_model( const std::string& folder );
        std::vector<double> classify_motion( const Eigen::MatrixXd& motion );

    private:
        Eigen::VectorXd gauss_pdf( const Eigen::MatrixXd&, int id_class, int id_state );
        Eigen::MatrixXd load_from_csv( const std::string& filename );
        Eigen::MatrixXd convert_to_matrix( const std::vector< std::vector< std::string > >& matrix );

        int m_nb_classes;
        int m_nb_states;
        std::vector<Eigen::VectorXd> m_priors;
        std::vector<Eigen::MatrixXd> m_mu;
        std::vector< std::vector<Eigen::MatrixXd> > m_sigma;
    };
}

extern HRICS::ClassifyMotion* global_classifyMotion;

#endif // HRICS_CLASSIFYMOTION_HPP

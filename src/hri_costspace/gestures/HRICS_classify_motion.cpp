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
#include "HRICS_classify_motion.hpp"

#include <iomanip>
#include <sstream>
#include <fstream>

using namespace std;
using namespace HRICS;

//#include <matio.h>
#include <iostream>

ClassifyMotion* global_classifyMotion = NULL;

ClassifyMotion::ClassifyMotion()
{

}

ClassifyMotion::~ClassifyMotion()
{

}

Eigen::VectorXd ClassifyMotion::gauss_pdf(const Eigen::MatrixXd& motion, int id_class, int id_state )
{
    double realmin = numeric_limits<double>::min();
//    [nbVar,nbData] = size(Data);
    int nbVar = motion.rows();
    int nbData = motion.cols();

    Eigen::MatrixXd Data = motion.transpose() - m_mu[id_class].col(id_state).transpose().replicate(nbData,1);
    Eigen::MatrixXd& Sigma = m_sigma[id_class][id_state];
    Eigen::VectorXd prob;
    prob = ((Data*(Sigma.inverse())).cwise()*Data).rowwise().sum();
    prob = (-0.5*prob).cwise().exp() / std::sqrt(std::abs((std::pow(2*M_PI,nbVar)*Sigma.determinant()+realmin)));
    return prob;
}

std::vector<double> ClassifyMotion::classify_motion(const Eigen::MatrixXd &motion )
{
    double realmin = numeric_limits<double>::min();

    std::vector<double> result( m_nb_classes );
    std::vector<Eigen::MatrixXd> Pxi( m_nb_classes );

    // Compute probability of each class
    for(int i=0;i<m_nb_classes;i++)
    {
        Pxi[i].setZero( motion.cols(), m_nb_states );

        for(int j=0;j<m_nb_states;j++)
        {
            //Compute the new probability p(x|i)
            Pxi[i].col(j) = gauss_pdf( motion, i, j );
        }

        // Compute the log likelihood of the class
        Eigen::VectorXd F = Pxi[i]*m_priors[i];

        for(int k=0;k<F.size();k++)
            if( F(k) < realmin || std::isnan(F(k)) )
                F(k) = realmin;

//        for(int k=0;k<F.size();k++)
//            if( F(k) > 1 )
//                cout << "F(" << k << ") : " << F(k) << endl;

        result[i] = F.cwise().log().sum()/F.size();
    }


    return result;
}

 Eigen::MatrixXd ClassifyMotion::convert_to_matrix( const std::vector< std::vector< std::string > >& matrix )
{
    Eigen::MatrixXd result;

    if(matrix.empty())
        return result;

    if(matrix[0].empty())
        return result;

    result.setZero( matrix.size(), matrix[0].size() );

    for( int i=0; i<int(matrix.size()); i++ )
    {
        for( int j=0; j<int(matrix[i].size()); j++ )
        {
            std::istringstream convert( matrix[i][j] );
            convert >> result(i,j);
        }
    }

    return result;
}

Eigen::MatrixXd ClassifyMotion::load_from_csv( const std::string& filename )
{
    std::ifstream       file( filename.c_str() );
    std::vector< std::vector<std::string> >   matrix;
    std::vector< std::string >   row;
    std::string                line;
    std::string                cell;

    while( file )
    {
        std::getline(file,line);
        std::stringstream lineStream(line);
        row.clear();

        while(std::getline( lineStream, cell, ',' ))
        {
            row.push_back( cell );
        }

        if( !row.empty() )
            matrix.push_back( row );
    }

    if( matrix.empty() ) {
        std::cout << "no data has been loaded" << std::endl;
        return Eigen::MatrixXd();
    }
//    std::cout << "text matrix fully loaded : ";
//    std::cout << "size : " << matrix.size() << " , " << matrix[0].size() << std::endl;

    return convert_to_matrix(matrix);
}

bool ClassifyMotion::load_model( const std::string& folder )
{
    m_nb_classes = 8;

    //------------------------------------------------------
    cout << "Load Priors" << endl;
    m_priors.resize( m_nb_classes );

    for( int i=0; i<int(m_priors.size()); i++)
    {
        ostringstream filename;
        filename << "Prior_1_" << i+1 << ".csv";
        //cout << folder + filename.str() << endl;
        Eigen::MatrixXd mat = load_from_csv( folder + filename.str() );
        if(  mat.size() == 0 )
            return false;

        m_priors[i] = mat.transpose();
    }

    //------------------------------------------------------
    cout << "Load Mu" << endl;
    m_mu.resize( m_nb_classes );

    for( int i=0; i<int(m_mu.size()); i++)
    {
        ostringstream filename;
        filename << "Mu_1_" << i+1 << ".csv";
        //cout << folder + filename.str() << endl;
         m_mu[i] = load_from_csv( folder + filename.str() );
        if(  m_mu[i].size() == 0 )
            return false;
    }

    //------------------------------------------------------
    cout << "Load Sigma" << endl;
    m_sigma.resize( m_nb_classes );
    m_nb_states = m_priors[0].size();

    for( int i=0; i<int(m_sigma.size()); i++)
    {
        m_sigma[i].resize( m_nb_states );

        for( int j=0; j<int(m_sigma[i].size()); j++)
        {
            ostringstream filename;
            filename << "Sigma_1_" << j+1 << "_"<< i+1   << ".csv";
            m_sigma[i][j] = load_from_csv( folder + filename.str() );
            if( m_sigma[i][j].size() == 0 )
                return false;
        }
    }

    return true;
}



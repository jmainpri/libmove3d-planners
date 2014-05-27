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
#include "misc_functions.hpp"

#include <iostream>
#include <fstream>

// For random number generator (seed can be passed as argument)
#include <libmove3d/include/P3d-pkg.h>

using std::cout;
using std::endl;

void move3d_save_matrix_to_file( const Eigen::MatrixXd& mat, std::string filename )
{
     cout << "save matrix to : " << filename << endl;
    std::ofstream file( filename.c_str() );
    if (file.is_open())
        file << mat << '\n';
//        for( int i=0;i<mat.rows();i++){
//            for( int j=0;j<mat.cols();j++)
//                file << mat(i,j) << " ";
//            file << endl;
//        }
    file.close();
}

std::vector<std::string>  move3d_get_files_in_folder( std::string foldername, std::string extension, int nb_max_files )
{
    bool quiet = true;
    std::vector<std::string> files;

    std::string command = "ls " + foldername;
    FILE* fp = popen( command.c_str(), "r");
    if (fp == NULL) {
        cout << "ERROR in system call" << endl;
        return files;
    }

    char str[PATH_MAX];
    while ( fgets( str, PATH_MAX, fp) != NULL )
    {
        std::string filename( str );
        filename = filename.substr(0, filename.size()-1);
        std::string file_extension( filename.substr( filename.find_last_of(".") + 1 ) );
        if( extension == file_extension )
        {
            if( !quiet ) {
                cout << "add : " << filename << endl;
            }
            files.push_back( filename );
        }

        if( (nb_max_files > 0) && (int(files.size()) >= nb_max_files) ) {
            break;
        }
    }
    pclose(fp);
    return files;
}

std::vector<int> move3d_change_basis( int number, int basis )
{
    std::vector<int> result( basis, 0 );

    size_t i = 1;
    while( number > 0 )  // number in base 10
    {
        int place = basis-(i++);
        if( place < 0 )
        {
            result.insert( result.begin(), number%basis );
        }
        else {
            result[place] = number%basis;
        }

        number = number/basis;
    }
    return result;
}

Eigen::VectorXd move3d_lerp( const Eigen::VectorXd& v0, const Eigen::VectorXd& v1, double t ) // t \in [0 1]
{
    return v0+(v1-v0)*t; // v0*(1-t)+v1*t
}

double move3d_random_integer( int min, int max )
{
    return p3d_random_integer( min, max );
}

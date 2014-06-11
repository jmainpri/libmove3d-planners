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
#include "NumsAndStrings.hpp"

#include <iostream>
#include <fstream>

// For random number generator (seed can be passed as argument)
#include <libmove3d/include/P3d-pkg.h>

using std::cout;
using std::endl;

void move3d_save_matrix_to_file( const Eigen::MatrixXd& matrix, std::string filename )
{
    cout << "save matrix to : " << filename << endl;
    std::ofstream file( filename.c_str() );
    if (file.is_open())
        file << matrix << '\n';
    //        for( int i=0;i<mat.rows();i++){
    //            for( int j=0;j<mat.cols();j++)
    //                file << mat(i,j) << " ";
    //            file << endl;
    //        }
    file.close();
}

void move3d_save_matrix_to_csv_file( const Eigen::MatrixXd& matrix, std::string filename )
{
    std::ofstream s;
    s.open( filename.c_str() );

    for (int i=0; i<matrix.rows(); i++)
    {
        for (int j=0; j<matrix.cols(); j++)
        {
            s << matrix( i, j ) << ",";
        }
        s << endl;
    }

    s.close();
}

// general case, stream interface
inline size_t word_count(std::stringstream& is)  // can pass an open std::ifstream() to this if required
{
    cout << is.str() << endl;
    size_t c = 0;
    for(std::string w; std::getline( is, w, ',' ); ++c)
        cout << "found word : " << w << endl;
    return c;
}

// simple string interface
inline size_t word_count(const std::string& str)
{
    cout << "line is : " << str << endl;
    std::stringstream ss(str);
    return word_count(ss);
}

Eigen::MatrixXd move3d_load_matrix_from_csv_file( std::string filename )
{
    Eigen::MatrixXd matrix;
    cout << "load matrix from : " << filename << endl;

    std::ifstream file( filename.c_str(), std::ifstream::in );

    if (file.is_open())
    {
        std::string line;
        std::string cell;

        int n_rows = std::count(std::istreambuf_iterator<char>(file),
                                std::istreambuf_iterator<char>(), '\n');

        int i=0, j=0;

        file.clear() ;
        file.seekg(0, std::ios::beg );

        while( file.good() )
        {
            std::getline( file, line );
            std::stringstream lineStream( line );

            if( i == 0 ) {
                int n_cols = word_count( line );
                matrix = Eigen::MatrixXd( n_rows, n_cols );
                cout << "size : ( " << n_rows << " , " << n_cols << " )" << endl;
            }

            j = 0;

            while( std::getline( lineStream, cell, ',' ) )
            {
                convert_text_to_num<double>( matrix(i, j), cell, std::dec );
                j++;
            }
            i++;
        }

        file.close();
    }
    else {
        cout << "could not open file : " << filename << endl;
    }

    return matrix;
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

//! This function prints the joint mapping of a robot
void print_joint_mapping( Move3D::Robot* robot )
{
    if( robot == NULL)
        return;

    cout << "print joint mapping" << endl;

    for( size_t i=0; i<robot->getNumberOfJoints(); i++)
    {
        //        cout << i << " , joint name : " << rob->getJoint(i)->getName() << endl;

        for( size_t j=0; j<robot->getJoint(i)->getNumberOfDof(); j++)
        {
            cout << "jnt->getName() : " << robot->getJoint(i)->getName() << "(" <<i<< ") , ";
            cout << "index_dof : " << robot->getJoint(i)->getIndexOfFirstDof()+j << endl;
            //cout << "move3d_map[\"" << rob->getJoint(i)->getName() << "\"]=" << rob->getJoint(i)->getIndexOfFirstDof()+j << ";" << endl;
        }
    }
}

void print_joint_anchors( Move3D::Robot* robot )
{
    if( robot == NULL)
        return;

    cout << "print joint mapping" << endl;

    for( size_t i=0; i<robot->getNumberOfJoints(); i++)
    {
        Move3D::Joint* j = robot->getJoint(i);
        Move3D::Joint* j_prev = j->getPreviousJoint();
        if( j_prev == NULL )
            continue;

        //        Eigen::Transform3d T( j_prev->getMatrixPos().inverse() );
        Eigen::Vector3d v( j_prev->getVectorPos() );

        cout << "j : " << j->getName() << " , j_prev : " << j_prev->getName() << " \t" << ( j->getVectorPos() - v ) .transpose() << endl;
    }

    for( size_t j=0; j<robot->getNumberOfJoints(); j++)
    {
        p3d_obj* o = robot->getJoint(j)->getP3dJointStruct()->o;

        if( o == NULL )
            continue;

        cout << "j : " << robot->getJoint(j)->getName() << endl;

        for( int i=0; i <o->np; i++ )
        {
            cout << "name : " << o->name << endl;
            //            cout << i  << ": " << o->jnt->abs_pos[0][3] << " " << o->jnt->abs_pos[1][3] << " " << o->jnt->abs_pos[2][3] << endl;
            cout << i  << ": " <<  o->pol[i]->pos_rel_jnt[0][3] << " " << o->pol[i]->pos_rel_jnt[1][3] << " " << o->pol[i]->pos_rel_jnt[2][3] << endl;
            cout << i  << ": " <<  o->pol[i]->pos0[0][3] << " " << o->pol[i]->pos0[1][3] << " " << o->pol[i]->pos0[2][3] << endl;

            p3d_matrix4 t;
            p3d_matMultXform( o->jnt->abs_pos, o->pol[i]->pos_rel_jnt, t );
            cout << i  << ": " << t[0][3] << " " << t[1][3]<< " " << t[2][3] << endl;
        }
    }
}

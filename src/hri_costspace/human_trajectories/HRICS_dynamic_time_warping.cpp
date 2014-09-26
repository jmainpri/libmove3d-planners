/*
Copyright (c) 2014, Calder Phillips-Grafflin (calder.pg@gmail.com)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "HRICS_dynamic_time_warping.hpp"

#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include <vector>
#include <string>
#include <sstream>
#include "string.h"
#include <iostream>
#include <algorithm>
#include <stdexcept>

using namespace DTW;
using std::cout;
using std::endl;

SimpleDTW::SimpleDTW()
{
    distance_fn_ = NULL;
    initialized_ = false;
}

SimpleDTW::SimpleDTW(size_t x_size, size_t y_size, double (*distance_fn)(std::vector<double> p1, std::vector<double> p2))
{
    distance_fn_ = distance_fn;
    SimpleDTW::Initialize(x_size, y_size);
}

SimpleDTW::SimpleDTW(double (*distance_fn)(std::vector<double> p1, std::vector<double> p2))
{
    distance_fn_ = distance_fn;
    initialized_ = false;
}

void SimpleDTW::Initialize(size_t x_size, size_t y_size)
{
    x_dim_ = x_size + 1;
    y_dim_ = y_size + 1;
    // Resize the data
    data_.resize(x_dim_ * y_dim_, 0.0);
    //Populate matrix with starting values
    SetInDTWMatrix(0, 0, 0.0);
    for (size_t i = 1; i < x_dim_; i++)
    {
        SetInDTWMatrix(i, 0, INFINITY);
    }
    for (size_t i = 1; i < y_dim_; i++)
    {
        SetInDTWMatrix(0, i, INFINITY);
    }
    initialized_ = true;
}

double SimpleDTW::EvaluateWarpingCost(std::vector< std::vector<double> > sequence_1, std::vector< std::vector<double> > sequence_2)
{
    // Sanity checks
    if (sequence_1.size() == 0 || sequence_2.size() == 0)
    {
        return INFINITY;
    }
    if (sequence_1[0].size() != sequence_2[0].size())
    {
        throw std::invalid_argument("Sequences for evaluation have different element sizes");
    }
    // Safety checks
    if (!distance_fn_)
    {
        throw std::invalid_argument("DTW evaluator is not initialized with a cost function");
    }
    if (!initialized_ || sequence_1.size() >= x_dim_ || sequence_2.size() >= y_dim_)
    {
        std::cout << "Automatically resizing DTW matrix to fit arguments" << std::endl;
        SimpleDTW::Initialize(sequence_1.size(), sequence_2.size());
    }

    // Compute DTW cost for the two sequences
    for (unsigned int i=1; i <= sequence_1.size(); i++)
    {
        for (unsigned int j=1; j <= sequence_2.size(); j++)
        {
            double index_cost = distance_fn_(sequence_1[i - 1], sequence_2[j - 1]);
            double prev_cost = 0.0;

            // Get the three neighboring values from the matrix to use for the update
            double im1j   = GetFromDTWMatrix(i - 1, j);
            double im1jm1 = GetFromDTWMatrix(i - 1, j - 1);
            double ijm1   = GetFromDTWMatrix(i, j - 1);
            // Start the update step
            if (im1j < im1jm1 && im1j < ijm1)
            {
                prev_cost = im1j;
            }
            else if (ijm1 < im1j && ijm1 < im1jm1)
            {
                prev_cost = ijm1;
            }
            else
            {
                prev_cost = im1jm1;
            }
            // Update the value in the matrix
            SetInDTWMatrix( i, j, index_cost + prev_cost );
        }
    }
    // Return total path cost
    return GetFromDTWMatrix( sequence_1.size(), sequence_2.size() );
}


double euclidean_distance(std::vector<double> P1, std::vector<double> P2)
{
    double total = 0.0;
    for (unsigned int i=0; i<P1.size(); i++)
    {
        total = total + pow( (P1[i] - P2[i]), 2 );
    }
    return sqrt(total);
}

double tansform_distance(std::vector<double> P1, std::vector<double> P2)
{
    if( P1.size() != 7 || P2.size() != 7 ){
        return 0;
    }

    double alpha = 1.0;

    double p_total = 0.0;
    for (unsigned int i=0; i<3; i++)
    {
        p_total = p_total + pow( (P1[i] - P2[i]), 2 );
    }
    p_total = sqrt(p_total);


    double r_total = 0.0;
    for (unsigned int i=3; i<7; i++)
    {
        r_total = r_total + pow( (P1[i] - P2[i]), 2 );
    }
    r_total = sqrt(r_total);

    return p_total + alpha * r_total;
}

int dtw_compare_performance(int traj_length, int iterations)
{
    struct timespec bstv, betv;
    printf("Building test arrays\n");
    std::vector< std::vector<double> > test_vec_1;
    for (int i = 0; i < traj_length; i++)
    {
        std::vector<double> state;
        state.push_back(0.0);
        state.push_back(0.0);
        state.push_back(0.0);
        test_vec_1.push_back(state);
    }
    std::vector< std::vector< std::vector<double> > > test_vec_2;
    for (int i = 0; i < iterations; i++)
    {
        std::vector< std::vector<double> > traj;
        for (int j = 0; j < traj_length; j++)
        {
            std::vector<double> state2;
            state2.push_back((double)rand());
            state2.push_back((double)rand());
            state2.push_back((double)rand());
            traj.push_back(state2);
        }
        test_vec_2.push_back(traj);
    }
    DTW::SimpleDTW my_eval = DTW::SimpleDTW(traj_length, traj_length, euclidean_distance);
    printf("Evaluating\n");
    //Run tests
    printf("-----Test single-threaded version-----\n");
    printf("Testing vector variant\n");
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &bstv);
    double scost = 0.0;
    for (int i = 0; i < iterations; i++)
    {
        scost = my_eval.EvaluateWarpingCost(test_vec_1, test_vec_2[i]);
        cout << "scost : " << scost << endl;
    }
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &betv);
    //---------------------------------------------
    //-----Compute runtimes (single-threaded)--------;
    float bsecsv = (float)(betv.tv_sec - bstv.tv_sec);
    bsecsv = bsecsv + (float)(betv.tv_nsec - bstv.tv_nsec) / 1000000000.0;
    printf("Final cost: %f\n", scost);
    printf("SINGLE (vector): %f\n", bsecsv);
    return 0;
}

std::vector< std::vector<double> > get_vector_from_matrix( const Eigen::MatrixXd& mat )
{
    std::vector< std::vector<double> > test_vec( mat.cols() );

    for (int j=0; j<mat.cols(); j++)
    {
        std::vector<double> state( mat.rows() );

        for (int i=0; i<mat.rows(); i++)
            state[i] = mat(i,j);

        test_vec[j] = state;
    }

    return test_vec;
}

std::vector<double> dtw_compare_performance( const std::vector<int>& active_dofs, const Move3D::Trajectory& t0, const std::vector<Move3D::Trajectory>& t_tests, bool use_transform_dist )
{
    std::vector<double> scost;

    struct timespec bstv, betv;
    printf("Building test arrays\n");

//    for (int i=0; i<active_dofs.size(); i++)
//        cout << "active dofs : " << active_dofs[i] << endl;

    // Store initial trajectory
    std::vector< std::vector<double> > test_vec_0;
    Eigen::MatrixXd mat = t0.getEigenMatrix( active_dofs );
    test_vec_0 = get_vector_from_matrix(mat);

    // Store all other trajectories
    std::vector< std::vector< std::vector<double> > > test_vec_1;
    for (int i=0; i<t_tests.size(); i++)
    {
        mat = t_tests[i].getEigenMatrix( active_dofs );

        if( mat.cols() != test_vec_0.size() ) { // Check that the trajectories have the same number of waypoints
            cout << "ERROR in dtw computations" << endl;
            return scost;
        }

        test_vec_1.push_back( get_vector_from_matrix(mat) );
    }

    DTW::SimpleDTW my_eval( test_vec_0.size(), test_vec_0.size(), use_transform_dist ? tansform_distance : euclidean_distance );

    cout << "Evaluating\n";
    //Run tests
    cout << "-----Test single-threaded version-----\n";
    cout << "Testing vector variant\n";

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &bstv);

    scost.resize( test_vec_1.size() );

    for (int i=0; i<scost.size(); i++)
    {
        scost[i] = my_eval.EvaluateWarpingCost( test_vec_0, test_vec_1[i] );
        cout << "scost[" << i << "] : " << scost[i] << endl;
    }
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &betv);
    //---------------------------------------------
    //-----Compute runtimes (single-threaded)--------;
    float bsecsv = (float)(betv.tv_sec - bstv.tv_sec);
    bsecsv = bsecsv + (float)(betv.tv_nsec - bstv.tv_nsec) / 1000000000.0;

    cout << "SINGLE (vector): " << bsecsv << endl;

    return scost;
}

//int main()
//{
//    std::cout << "Running performance tests..." << std::endl;
//    compare_performance(1000, 1000);
//    std::cout << "...done performance testing" << std::endl;
//    return 0;
//}

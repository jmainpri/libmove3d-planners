#ifndef SPRM_HPP
#define SPRM_HPP
//
//  sPRM.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 12/03/14.
//  Copyright (c) 2014 WPI. All rights reserved.
//
#include "planner/planner.hpp"
#include "planner/PRM/PRM.hpp"

namespace Move3D
{

class sPRM : public PRM
{
public:

    /**
     * Creates a perturbation roadmap from a given robot
     * and a given graph
     * @param Robot
     * @param Graph
     */
    sPRM(Robot* R, Graph* G);

    /**
     * Deletes a perturbation planner
     */
    ~sPRM();

    /**
     * Adds nodes to Graph
     */
    virtual void expandOneStep();

    /**
      * Connect nodes
      */
    void postPocess();

protected:

    confPtr_t qi_;
    confPtr_t qf_;

    int dist_choice_;
    double radius_;
};

}

#endif // SPRM_HPP

#ifndef PRMSTAR_HPP
#define PRMSTAR_HPP

//  sPRM.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 16/03/14.
//  Copyright (c) 2014 WPI. All rights reserved.
//

#include "planner/planner.hpp"
#include "planner/PRM/sPRM.hpp"

#include "API/ConfigSpace/cspace.hpp"

namespace Move3D
{

class PRMStar : public sPRM
{
public:

    /**
     * Creates a perturbation roadmap from a given robot
     * and a given graph
     * @param Robot
     * @param Graph
     */
    PRMStar(Robot* R, Graph* G);

    /**
     * Deletes a perturbation planner
     */
    ~PRMStar();

protected:

    void initCSpace();

    /**
   * Get the rgg ball raduis
   */
    double rrgBallRadius();

    /**
   * Compute the radius
   */
    double computeRadius();

    // cspace
    CSpace* cspace_;
};

}

#endif // PRMSTAR_HPP

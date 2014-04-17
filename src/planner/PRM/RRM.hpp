//
//  RRM.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 21/02/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#ifndef RRM_HPP_
#define RRM_HPP_

#include "planner/PRM/PRM.hpp"

class RRM : public PRM
{
public:
    /**
     * Creates a perturbation roadmap from a given robot
   * and a given graph
     * @param Robot
   * @param Graph
     */
    RRM(Robot* R, Graph* G);

    /**
     * Deletes a perturbation planner
     */
    ~RRM();

    /**
     * Adds nodes to Graph
     */
    virtual void expandOneStep();

protected:

    confPtr_t m_qi;
    confPtr_t m_qf;
};

#endif

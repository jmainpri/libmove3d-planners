//
//  TessallationRM.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 23/04/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#include "TessallationRM.hpp"

#include <iostream>

using std::cout;
using std::endl;

TessallationRM::TessallationRM( Robot* R, Graph* G) : Planner(R,G)
{

}

TessallationRM::~TessallationRM()
{

}

void TessallationRM::CreateTessalatedGraph()
{
    int nb_x = 10;
    int nb_y = 10;

    vector<double> env; // TODO set size
    grid_.setEnvSizeAndNumCell( nb_x, nb_y, env );
    grid_.createAllCells();

    std::vector<Node*> nodes( grid_.getNumberOfCells() );

    for( int i=0;i<grid_.getNumberOfCells();i++)
    {
        Eigen::Vector2d x = grid_->getCell(i)->getCenter();

        confPtr_t q = _Robot->getCurrentPos();
        (*q)[6] = x[0];
        (*q)[7] = x[1];

        nodes[i] = new Node(_Graph,q);
        _Graph->addNode( nodes[i] );
    }
}

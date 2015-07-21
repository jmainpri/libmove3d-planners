//
//  main.cpp
//  libmove3d-planners
//
//  Created by Benjamin VADANT on 24/11/12.
//  Copyright 2012 LAAS/CNRS. All rights reserved.


#include <iostream>
#include <map>
#include <vector>
#include <stdio.h>
#include <time.h>

#include "API/scene.hpp"
#include "API/project.hpp"
#include "planner/cost_space.hpp"
#include "planEnvironment.hpp"
#include <Planner-pkg.h>
#include "plannerFunctions.hpp"
#include "P3d-pkg.h"
#include "Collision-pkg.h"

using namespace std;

int main(int argc, char *argv[])
{
    string home, filename;

    home =  getenv("HOME_MOVE3D");
    if ( "" == home )
    {
        cout << "Error : HOME_MOVE3D not define" << endl;
        return 0;
    }

    set_DO_KCD_GJK(TRUE);
    p3d_init_random();
    set_collision_by_object(FALSE);
    p3d_filter_switch_filter_mechanism(TRUE);

    cout<<"Test 1 : Manipulateur"<<endl;

    filename = "/Manipulator/manipulateur.p3d";
    p3d_col_set_mode(p3d_col_mode_none);
    p3d_read_desc((home+filename).c_str());
    p3d_col_set_mode(p3d_col_mode_kcd);
    p3d_col_start(p3d_col_mode_kcd);
    set_collision_by_object(TRUE);

    global_Project = new Project(new Scene(XYZ_ENV));
    ENV.setBool(Env::drawExploration,false);
    GlobalCostSpace::initialize();
    global_Project->getActiveScene()->setActiveRobot(XYZ_ENV->robot[0]->name);
    ENV.setBool(Env::useTRRT,false);
    PlanEnv->setBool(PlanParam::withSmoothing,false);

    cout<<"Test 1 a : Manipulateur with RRT connect"<<endl;
    ENV.setExpansionMethod(Env::Connect);
    p3d_run_rrt(XYZ_ENV->robot[0]);


    cout<<"Test 1 b : Manipulateur with RRT extend"<<endl;
    ENV.setExpansionMethod(Env::Extend);
    p3d_run_rrt(XYZ_ENV->robot[0]);



    p3d_col_stop_all();
    set_collision_by_object(FALSE);

    cout<<"Test 2 : Stones"<<endl;

    filename = "/CostDistanceKCD/2dof/Stones.p3d";
    p3d_col_set_mode(p3d_col_mode_none);
    p3d_read_desc((home+filename).c_str());
    p3d_col_set_mode(p3d_col_mode_kcd);
    p3d_col_start(p3d_col_mode_kcd);
    set_collision_by_object(TRUE);


    global_Project = new Project(new Scene(XYZ_ENV));
    GlobalCostSpace::initialize();
    global_Project->getActiveScene()->setActiveRobot(XYZ_ENV->robot[0]->name);
    ENV.setBool(Env::useTRRT,false);
    PlanEnv->setBool(PlanParam::withSmoothing,false);

    cout<<"Test 2 a : Stones with RRT connect"<<endl;
    ENV.setExpansionMethod(Env::Connect);
    p3d_run_rrt(XYZ_ENV->robot[0]);

    cout<<"Test 2 b : Stones with RRT extend"<<endl;
    ENV.setExpansionMethod(Env::Extend);
    p3d_run_rrt(XYZ_ENV->robot[0]);

    p3d_col_stop_all();
    set_collision_by_object(FALSE);


    cout<<"Test 3 : Montains"<<endl;

    filename = "/CostEnv/Montains3/Montains3.p3d";
    p3d_col_set_mode(p3d_col_mode_none);
    p3d_read_desc((home+filename).c_str());
    p3d_col_set_mode(p3d_col_mode_kcd);
    p3d_col_start(p3d_col_mode_kcd);
    set_collision_by_object(TRUE);


    global_Project = new Project(new Scene(XYZ_ENV));
    GlobalCostSpace::initialize();
    global_Project->getActiveScene()->setActiveRobot(XYZ_ENV->robot[0]->name);
    ENV.setBool(Env::useTRRT,true);
    PlanEnv->setBool(PlanParam::withSmoothing,false);

    p3d_run_rrt(XYZ_ENV->robot[0]);

    p3d_col_stop_all();
    set_collision_by_object(FALSE);

    cout<<"Test 4 : Lydia with VisPRM"<<endl;

    filename = "/Lydia/lydia1.p3d";
    p3d_col_set_mode(p3d_col_mode_none);
    p3d_BB_set_mode_close();
    p3d_read_desc((home+filename).c_str());
    p3d_col_set_mode(p3d_col_mode_kcd);
    p3d_col_start(p3d_col_mode_kcd);
    set_collision_by_object(TRUE);

    global_Project = new Project(new Scene(XYZ_ENV));

    global_Project->getActiveScene()->setActiveRobot(XYZ_ENV->robot[0]->name);
    PlanEnv->setBool(PlanParam::withSmoothing,false);
    clock_t start = clock();

    p3d_run_vis_prm(XYZ_ENV->robot[0]);
    printf("Time elapsed: %f\n", ((double)clock() - start) / CLOCKS_PER_SEC);

    return 0;
}


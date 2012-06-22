//
//  HRICS_Miscellaneous.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 21/11/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

namespace HRICS
{
  void printPr2Config();
  void setSimulationRobotsTransparent();
  void generateGraspConfigurations();
  void setThePlacemateInIkeaShelf();
  
  bool initShelfScenario();
  bool execShelfScenario();
  bool simpShelfScenario();
};
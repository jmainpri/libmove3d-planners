//
//  HRICS_miscellaneous.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 21/11/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

namespace HRICS
{
  void printHumanConfig();
  void printPr2Config();
  void setSimulationRobotsTransparent();
  void generateGraspConfigurations();
  void setThePlacemateInIkeaShelf();
  void setTenAccessiblePositions();
  
  bool initShelfScenario();
  bool execShelfScenario();
  bool simpShelfScenario();
};
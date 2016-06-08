#include <misc_functions.hpp>
#include "API/Device/robot.hpp"
#include "API/project.hpp"

// For the seed ...
#include <libmove3d/include/P3d-pkg.h>

using std::cout;
using std::endl;

int main(int argc, char* argv[])
{
  std::string filename = "/../assets/CostDistanceKCD/3dof/helico_test.p3d";
  Move3D::Robot* robot = move3d_start_all_p3d_environment(filename);
  if (robot == NULL) {
    cout << "Error: could not find robot" << endl;
    return EXIT_FAILURE;
  }

  // Set seed
  p3d_init_random_seed(2);

  cout << "ROBOT is : " << robot->getName() << endl;

  Move3D::confPtr_t q = robot->shoot();
  robot->setAndUpdate(*q);

  Eigen::Quaterniond quaternion = robot->getJoint(1)->getOrientation();
  cout << " ** Quaternion q : " << endl;
  cout << quaternion.w() << " , " << quaternion.x() << " , " << quaternion.y()
       << " , " << quaternion.z() << " , " << endl;

  Eigen::Vector3d eulers = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
  cout << " ** Euler Angles : " << endl;
  cout << eulers.transpose() << endl;

  Eigen::Vector3d eulers_2;
  eulers_2 << (*q)[9], (*q)[10], (*q)[11];
  cout << " ** Configuration : " << endl;
  cout << eulers_2.transpose() << endl;

  double delta = (eulers - eulers_2).norm();
  if (delta > 1e-6) {
    cout << "Error in quaterion and eulers conversions (" << delta << ")"
         << endl;
    return EXIT_FAILURE;
  }

  cout << "Done." << endl;
  return EXIT_SUCCESS;
}

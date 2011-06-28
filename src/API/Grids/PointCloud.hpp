#ifndef POINTS_H
#define POINTS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

/**
@ingroup GRID
*! Vector of 3d points that can be ploted in the 3d viewer as cubes very fast
*! the points are stored in a std::vector and the class has a similar API 
*/
class PointCloud
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  PointCloud();
  PointCloud(double PointSize);
  
  void push_back(Eigen::Vector3d point);
  
  void clear() 
  { 
    m_AllPoints.clear(); 
  }
  
  unsigned int size() 
  { 
    return m_AllPoints.size(); 
  }
  
  void resize(unsigned int sz)
  {
    m_AllPoints.resize(sz);
  }
  
  /**
	 * Acces the configuration
	 */
	 Eigen::Vector3d& operator [] ( const int &i ) { return m_AllPoints[i]; }
  
  
  void drawAllPoints(double* color = NULL);
  void drawAllPoints(const Eigen::Transform3d & t,  double* color = NULL );
  
private:
  void drawOnePoint(bool withTransform, const Eigen::Transform3d & t, int i);
  
  std::vector< Eigen::Vector3d > m_AllPoints;
  Eigen::Vector3d m_CubeSize;
};

extern PointCloud* PointsToDraw;

#endif // POINTS_H

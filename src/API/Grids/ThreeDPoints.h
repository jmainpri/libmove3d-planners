#ifndef POINTS_H
#define POINTS_H

#include <Eigen/Core>
#include <vector>

/**
@ingroup GRID
@brief vector of 3d points that can be ploted in the 3d viewer as cubes very fast
*/
class ThreeDPoints
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ThreeDPoints();

    void push_back(Eigen::Vector3d point);
	void clear() { m_AllPoints.clear(); }
	unsigned int size() { return m_AllPoints.size(); }
    void drawAllPoints();
    void drawOnePoint(int i);

private:
    std::vector< Eigen::Vector3d > m_AllPoints;
    Eigen::Vector3d m_CubeSize;
};

extern ThreeDPoints* PointsToDraw;

#endif // POINTS_H

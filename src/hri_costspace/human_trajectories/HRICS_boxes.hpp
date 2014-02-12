#ifndef HRICS_BOXES_HPP
#define HRICS_BOXES_HPP

#include "HRICS_features.hpp"
#include "API/Device/robot.hpp"

namespace HRICS
{

class Box
{
public:
    Box( const Eigen::Vector3d& center, double x, double y, double z )
    {
        center_ = center;
        x_ = x;
        y_ = y;
        z_ = z;
    }

    void draw();

    Eigen::Vector3d center_;
    double x_;
    double y_;
    double z_;
};

class Boxes : public Feature
{
    public:

        Boxes() { }

        void initialize();

        FeatureVect getFeatures(const Configuration& q );
        double getFeaturesJacobianMagnitude(const Configuration& q);
        double jacobianCost(const Configuration& q);

        void placeCenterGrid(bool on_wall);
        void printWeights() const;
        int addCenters(std::string type);
        void computeSize();

        bool isInAASquare( const std::vector<Eigen::Vector3d>& corners, Eigen::Vector3d p );
        double distToSquare(  const Box& box, const Configuration& q  );
        double pointToLineSegmentDistance(const Eigen::Vector3d& p, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, Eigen::Vector3d& closestPoint);
        void draw();

    protected:
        Robot* robot_;
        std::vector<Robot*> centers_;
        std::vector<Box> boxes_;
    };
}

extern HRICS::Boxes* global_BoxesCostFct;

#endif // HRICS_BOXES_HPP

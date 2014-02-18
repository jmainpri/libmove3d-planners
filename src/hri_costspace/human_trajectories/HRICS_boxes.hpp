#ifndef HRICS_BOXES_HPP
#define HRICS_BOXES_HPP

#include "HRICS_squares.hpp"
#include "API/Device/robot.hpp"
#include "collision_space/BodySurfaceSampler.hpp"

namespace HRICS
{

class Box : public Square
{
public:
    Box( const Eigen::VectorXd& center, const Eigen::VectorXd& size ) : Square(center,size) {}
    void draw() const;
};

class Boxes : public Squares
{
    public:

        Boxes();
        ~Boxes();

        void initialize();

        FeatureVect getFeatures(const Configuration& q );
        double getFeaturesJacobianMagnitude(const Configuration& q);
        double jacobianCost(const Configuration& q);

        void computeSize();

        bool isInAABox( const Eigen::VectorXd& limits, Eigen::Vector3d p );
        double distToBox(  const Box& box, const Configuration& q  );

        void drawCollisionPoints();
        void draw();

private:
        std::vector<int> active_joints_;
        BodySurfaceSampler* sampler_;
    };
}

bool HRICS_init_boxes_cost();

extern HRICS::Boxes* global_BoxesCostFct;

#endif // HRICS_BOXES_HPP

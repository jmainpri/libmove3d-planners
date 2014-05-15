#ifndef HRICS_BOXES_HPP
#define HRICS_BOXES_HPP

#include "squares.hpp"
#include "API/Device/robot.hpp"
#include "collision_space/body_surface_sampler.hpp"

namespace Move3D
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

        FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_dofs );
        double getFeaturesJacobianMagnitude( const Move3D::Configuration& q );
        double jacobianCost( const Move3D::Configuration& q );

        void computeSize();

        bool isInAABox( const Eigen::VectorXd& limits, Eigen::Vector3d p );
        double distToBox(  const Box& box, const Move3D::Configuration& q  );

        void drawCollisionPoints();
        void draw();

private:
        std::vector<int> active_joints_;
        Move3D::BodySurfaceSampler* sampler_;
    };
}

bool HRICS_init_boxes_cost();

extern Move3D::Boxes* global_BoxesCostFct;

#endif // HRICS_BOXES_HPP

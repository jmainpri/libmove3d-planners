#ifndef HRICS_SHPERES_3D_HPP
#define HRICS_SHPERES_3D_HPP

#include "HRICS_planar_feature.hpp"
#include "API/Device/robot.hpp"
#include "collision_space/BodySurfaceSampler.hpp"

namespace HRICS
{

class Sphere
{
public:
    Sphere( const Eigen::VectorXd& center, double raduis ) :
        center_(center), raduis_(raduis)
    {}

    void draw() const;

    Eigen::Vector3d center_;
    double color_[4];
    double raduis_;
};

class Spheres3D : public PlanarFeature
{
    public:

        Spheres3D();
        ~Spheres3D();

        void initialize();

        FeatureVect getFeatures(const Configuration& q );
        double getFeaturesJacobianMagnitude(const Configuration& q);
        double jacobianCost(const Configuration& q);

        void computeSize();
        void placeCenterGrid(bool on_wall);

        double distToShpere( const Sphere& sph );

        void setSphereToDraw(int id, bool enable);
        void setShperesToDraw();
        void drawCollisionPoints();
        void draw();

private:
        std::vector<int> active_joints_;
        BodySurfaceSampler* sampler_;
        std::vector<Sphere*> spheres_;
        std::vector<int> sphere_to_draw_;
    };
}

bool HRICS_init_shperes_3d_cost();

extern HRICS::Spheres3D* global_Spheres3DCostFct;

#endif // HRICS_SHPERES_3D_HPP

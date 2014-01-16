#include "graphSampler.hpp"

#include "API/project.hpp"
#include "planner/planEnvironment.hpp"

#include <iomanip>

using std::cout;
using std::endl;

graphSampler::graphSampler()
{
    num_points_per_dim_ = 2;
    num_joints_ = 2;
    vect_length_ = num_joints_ * pow( double(num_points_per_dim_), double(num_joints_) );
}

graphSampler::graphSampler( int num_points_per_dim, int num_joints ) : num_points_per_dim_(num_points_per_dim), num_joints_(num_joints)
{
    vect_length_ = num_joints_ * pow( double(num_points_per_dim_), double(num_joints_) );
}

graphSampler::~graphSampler()
{
    delete noise_generator_;
}

void graphSampler::initialize()
{
    tmp_noise_ = Eigen::VectorXd::Zero( vect_length_ );
    precision_ = Eigen::MatrixXd::Zero( vect_length_, vect_length_ );

//    precision_ <<   1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 1  -> 1,1
//                    0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 2  -> 1,2
//                    0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 3  -> 1,3
//                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 4  -> 1,4
//                    0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 5  -> 2,1
//                    0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 6  -> 2,2
//                    0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 7  -> 2,3
//                    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 8  -> 2,4
//                    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 9  -> 3,1
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10 -> 3,2
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 11 -> 3,3
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 12 -> 3,4
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 13 -> 4,1
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 14 -> 4,2
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 15 -> 4,3
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 16 -> 4,4
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
//                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    // double a = 1.0;
    // double b = 0.5;
    // double c = 0.01;

    double a = PlanEnv->getDouble(PlanParam::samplegraphVarianceA);
    double c = PlanEnv->getDouble(PlanParam::samplegraphVarianceB);

                 // x  x  x  x      y  y  y  y
                 // 1, 2, 3, 4      1, 2, 3, 4
    precision_ <<   1, a, c, a,     0, a, c, c, // 1  -> 1,1 x
                    a, 1, a, c,     c, 0, a, a, // 2  -> 1,2 x
                    c, a, 1, a,     a, a, 0, c, // 3  -> 2,1 x
                    a, c, a, 1,     c, a, c, 0, // 4  -> 2,2 x

                    0, c, a, c,     1, c, a, a, // 1  -> 1,1 y
                    a, 0, a, a,     c, 1, a, a, // 2  -> 1,2 y
                    c, a, 0, c,     a, a, 1, c, // 3  -> 2,1 y
                    c, a, c, 0,     a, a, c, 1; // 4  -> 2,1 y

    preAllocateMultivariateGaussianSampler();
}

bool graphSampler::preAllocateMultivariateGaussianSampler()
{
    // invert the control costs, initialize noise generators:
    inv_precision_ = precision_.inverse();

    cout.precision(3);
    cout << inv_precision_ << endl; // << std::scientific << endl;

    // TODO see of the noise generator needs to be
    // var free or var all
    MultivariateGaussian* mvg = new MultivariateGaussian( Eigen::VectorXd::Zero( vect_length_ ), inv_precision_ );
    noise_generator_ = mvg;

    return true;
}

Eigen::VectorXd graphSampler::sample_noisy(double std_dev)
{
    // Eigen::VectorXd tmp;
    cout << "tmp_noise_ : " << tmp_noise_.transpose() << endl;
    noise_generator_->sample( tmp_noise_ );
    tmp_noise_ *= std_dev;
    cout << "tmp_noise_ : " << tmp_noise_.transpose() << endl;
    return tmp_noise_;
}

void graphSampler::setNodesInGraph(Graph* g)
{
    int ith=1;
    for( int i=0; i<num_points_per_dim_; i++ )
    {
        for( int j=0; j<num_points_per_dim_; j++ )
        {
            confPtr_t q = g->getRobot()->getCurrentPos();
            int x_id = num_points_per_dim_*i+j;
            int y_id = num_points_per_dim_*i+j+num_points_per_dim_*2;
            (*q)[6] = tmp_noise_[x_id];
            (*q)[7] = tmp_noise_[y_id];
            cout << "node( " << x_id << " , " << y_id << " ) : " << (*q)[6] << " , " << (*q)[7] << endl;
            Node* N = new Node( g, q );
            N->color_ = ith++;
            g->insertNode(N);
        }
    }
}

Graph* graphSampler::sample()
{
    Robot* robot = global_Project->getActiveScene()->getActiveRobot();
    Graph* graph = new Graph(robot);

    int iterations = 1;
    if(PlanEnv->getBool(PlanParam::samplegraphMultiLoop))
        iterations = 100;

    for( int i=0; i<iterations; i++)
    {
        sample_noisy(20);
        setNodesInGraph( graph );
    }
    cout << "Nb of nodes : " << graph->getNumberOfNodes() << endl;

    delete API_activeGraph;
    API_activeGraph = graph;
    return NULL;
}

#include "HRICS_planarfeature.hpp"

#include "API/misc_functions.hpp"
#include "API/project.hpp"

#include <iomanip>
#include <sstream>

using namespace HRICS;
using std::cout;
using std::endl;

HRICS::PlanarFeature* global_PlanarCostFct=NULL;

int PlanarFeature::addCenters(std::string type)
{
    Scene* sce = global_Project->getActiveScene();
    std::stringstream ss;

    double gray_scale = 0.0;
    double color_vect[4];

    int i=1;

    // Add the sphere centers
    while(true)
    {
        ss.str(""); // clear stream
        ss << type << "_MU_" << std::setw(2) << std::setfill( '0' ) << i ;

        Robot* center = sce->getRobotByName( ss.str() );

        if( center != NULL )
        {
            centers_.push_back( center );
            cout << "Add robot : " << centers_.back()->getName() << endl;
            i++;
        }
        else{
            break;
        }
    }

    // Set the colors
    for(int i=0;i<int(centers_.size());i++)
    {
        p3d_obj* o = p3d_get_robot_body_by_name( centers_[i]->getRobotStruct(), "body" );
        cout << o->name << endl;

        gray_scale = 1-double(i)/double(centers_.size()); // TODO fix
        // color_vect[0] = gray_scale;
        // color_vect[1] = gray_scale;
        // color_vect[2] = gray_scale;
        // color_vect[3] = 1.0;

        GroundColorMixGreenToRed( color_vect, gray_scale );
        color_vect[3] = 1.0;

        p3d_poly_set_color( o->pol[1], Any, color_vect );
    }

    return centers_.size();
}

void PlanarFeature::produceCostMap()
{
    double max_1, max_2;
    double min_1, min_2;
    robot_->getJoint(1)->getDofBounds( 0, min_1, max_1 );
    robot_->getJoint(1)->getDofBounds( 1, min_2, max_2 );

    cout << max_1 << " , " << max_2 << " , " << min_1 << " , " << min_2 << endl;

    int nb_cells = 100;
    Eigen::MatrixXd mat( nb_cells, nb_cells );

    int k=0;

    for( int i=0; i<nb_cells; i++ )
    {
        for( int j=0; j<nb_cells; j++ )
        {
            confPtr_t q = robot_->getCurrentPos();
            (*q)[6] = min_1 + double(i)*(max_1-min_1)/double(nb_cells-1);
            (*q)[7] = min_2 + double(j)*(max_2-min_2)/double(nb_cells-1);
            mat(i,j) = cost(*q);
            // mat(i,j) = getFeatures(*q).norm();
            // cout << getFeatures(*q).transpose() << endl;
            // cout << " ( x : " << (*q)[6] << " , y : " << (*q)[7] << " ) = " << mat(i,j) << endl;
            k++;
        }
    }

    cout << "k : " << k << endl;

    move3d_save_matrix_to_file( mat, "matlab/cost_map_64.txt");
}

void PlanarFeature::produceDerivativeFeatureCostMap()
{
    double max_1, max_2;
    double min_1, min_2;
    robot_->getJoint(1)->getDofBounds( 0, min_1, max_1 );
    robot_->getJoint(1)->getDofBounds( 1, min_2, max_2 );

    int nb_cells = 100;
    Eigen::MatrixXd mat0( nb_cells, nb_cells );
    Eigen::MatrixXd mat1( nb_cells, nb_cells );
    Eigen::MatrixXd mat2( nb_cells, nb_cells );
    Eigen::MatrixXd mat3( nb_cells, nb_cells );
    Eigen::MatrixXd mat4( nb_cells, nb_cells );

    for( int i=0; i<nb_cells; i++ )
    {
        for( int j=0; j<nb_cells; j++ )
        {
            confPtr_t q = robot_->getCurrentPos();
            (*q)[6] = min_1 + double(i)*(max_1-min_1)/double(nb_cells-1);
            (*q)[7] = min_2 + double(j)*(max_2-min_2)/double(nb_cells-1);
            FeatureJacobian J = getFeaturesJacobian(*q);
            // cout << "J : " << endl << J << endl;
            mat0(i,j) = getFeatures(*q).norm(); // NORM
            // mat1(i,j) = J.col(0).maxCoeff();
            // mat2(i,j) = J.col(1).maxCoeff();
            mat1(i,j) = J.col(0).norm();
            mat2(i,j) = J.col(1).norm();

            mat3(i,j) = Feature::getFeaturesJacobianMagnitude(*q);
            mat4(i,j) = getFeaturesJacobianMagnitude(*q);
        }
    }

    move3d_save_matrix_to_file( mat0, "matlab/cost_map_feat_64.txt");
    move3d_save_matrix_to_file( mat1, "matlab/cost_map_derv_0_64.txt");
    move3d_save_matrix_to_file( mat2, "matlab/cost_map_derv_1_64.txt");

    move3d_save_matrix_to_file( mat3, "matlab/cost_map_jac_mag_simple.txt");
    move3d_save_matrix_to_file( mat4, "matlab/cost_map_jac_mag_custom.txt");
}

void PlanarFeature::placeCenterGrid(bool on_wall)
{
    double max_1, max_2;
    double min_1, min_2;
    robot_->getJoint(1)->getDofBounds( 0, min_1, max_1 );
    robot_->getJoint(1)->getDofBounds( 1, min_2, max_2 );

    int nb_cells = std::sqrt( double(centers_.size()) );
    int offset = on_wall ? 0 : 1 ;

    for( int i=offset; i<nb_cells+offset; i++ )
    {
        for( int j=offset; j<nb_cells+offset; j++ )
        {
            int id = (i-offset)*(nb_cells)+(j-offset);
            //int id = i*(nb_cells)+j;
            confPtr_t q = centers_[id]->getCurrentPos();

            int divisions = nb_cells-1;
            if( !on_wall )
                divisions = nb_cells+1;

            (*q)[6] = min_1 + double(i)*(max_1-min_1)/double(divisions);
            (*q)[7] = min_2 + double(j)*(max_2-min_2)/double(divisions);
            centers_[id]->setAndUpdate( *q );
            centers_[id]->setInitPos( *q );
            cout << "c (" << id << ") : " << (*q)[6] << " , " << (*q)[7] << endl;
        }
    }
}

void PlanarFeature::printWeights() const
{
    cout << "weights : center" << endl;
    cout.precision(3);

    int n=std::sqrt(w_.size());
    for(int i=0;i<n;i++)
    {
        for(int j=0;j<n;j++)
        {
            cout << "\t " << w_[n*i+j] << std::fixed;
        }
        cout << endl;
    }
}

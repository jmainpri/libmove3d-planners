#include "HRICS_planar_feature.hpp"

#include "HRICS_GestParameters.hpp"
#include "HRICS_parameters.hpp"

#include "API/misc_functions.hpp"
#include "API/project.hpp"

#include <iomanip>
#include <sstream>

using namespace Move3D;
using namespace HRICS;
using std::cout;
using std::endl;

HRICS::PlanarFeature* global_PlanarCostFct=NULL;

std::string cost_map_folder = "matlab/cost_maps/";

PlanarFeature::PlanarFeature()
{
    balance_cumul_jac_ = 0.0;
    cur_min_diff_ = 0.0;
    min_diff_ = std::numeric_limits<double>::max();

    phi_cumul_ = FeatureVect::Ones( getNumberOfFeatures() );
    phi_jac_cumul_ = FeatureVect::Ones( getNumberOfFeatures() );

    cout << "phi_cumul : " << phi_cumul_.transpose() << endl;
    cout << "phi_cumul_jac : " << phi_jac_cumul_.transpose() << endl;
}

int PlanarFeature::addCenters(std::string type)
{
    Scene* sce = global_Project->getActiveScene();
    std::stringstream ss;
    centers_.clear();

    // Add the box centers
    for(int i=1;;i++)
    {
        ss.str(""); // clear stream
        ss << type << "_MU_" << std::setw(2) << std::setfill( '0' ) << i ;

        Robot* center = sce->getRobotByName( ss.str() );
        if( center != NULL )
        {
            centers_.push_back( center );
            cout << "Add robot : " << centers_.back()->getName() << endl;
        }
        else{
            break;
        }
    }

    double color_vect[4];
    color_vect[3] = 1.0; // Set transparency alpha

    // Set boxes colors
    for(int i=0;i<int(centers_.size());i++)
    {
        p3d_obj* o = p3d_get_robot_body_by_name( centers_[i]->getRobotStruct(), "body" );
        if( o == NULL ) {
            cout << "Could not get center : " << i << " , with name body" << endl;
            continue;
        }

        if( o->np > 1 ) {
            // Low ids are green, high ids are red
            GroundColorMixGreenToRed( color_vect, double(i)/double(centers_.size()) );
            p3d_poly_set_color( o->pol[1], Any, color_vect ); // Set the second body color
        }
        else {
            cout << "Could not set color of body : " << o->name << endl;
        }
    }

    return centers_.size();
}

void PlanarFeature::produceCostMap(int ith)
{
    double max_1, max_2;
    double min_1, min_2;
    robot_->getJoint(1)->getDofBounds( 0, min_1, max_1 );
    robot_->getJoint(1)->getDofBounds( 1, min_2, max_2 );
    // cout << max_1 << " , " << max_2 << " , " << min_1 << " , " << min_2 << endl;

    int nb_cells = 100;
    Eigen::MatrixXd mat0( nb_cells, nb_cells );
    Eigen::MatrixXd mat1( nb_cells, nb_cells );

    int k=0;

    for( int i=0; i<nb_cells; i++ )
    {
        for( int j=0; j<nb_cells; j++ )
        {
            confPtr_t q = robot_->getCurrentPos();
            (*q)[6] = min_1 + double(i)*(max_1-min_1)/double(nb_cells-1);
            (*q)[7] = min_2 + double(j)*(max_2-min_2)/double(nb_cells-1);
            // mat0(i,j) = cost(*q); // Class function
            mat0(i,j) = q->cost(); // Global cost
            mat1(i,j) = getFeatures(*q).norm();
            // cout << getFeatures(*q).transpose() << endl;
            // cout << " ( x : " << (*q)[6] << " , y : " << (*q)[7] << " ) = " << mat(i,j) << endl;
            k++;
        }
    }

//    cout << "k : " << k << endl;

    std::stringstream ss;

    ss.str(""); // clear stream
    ss << cost_map_folder + "cost_map_" << std::setw(2) << std::setfill( '0' ) << ith << ".txt";
    move3d_save_matrix_to_file( mat0, ss.str() );

    ss.str(""); // clear stream
    ss << cost_map_folder + "cost_map_feat_" << std::setw(2) << std::setfill( '0' ) << ith << ".txt";
    move3d_save_matrix_to_file( mat1, ss.str() );
}

void PlanarFeature::produceDerivativeFeatureCostMap(int ith)
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
            // mat0(i,j) = getFeatures(*q).norm(); // NORM
            // mat1(i,j) = J.col(0).maxCoeff();
            // mat2(i,j) = J.col(1).maxCoeff();
            mat1(i,j) = J.col(0).norm();
            mat2(i,j) = J.col(1).norm();

            mat3(i,j) = Feature::getFeaturesJacobianMagnitude(*q);
            mat4(i,j) = getFeaturesJacobianMagnitude(*q);
        }
    }

    std::stringstream ss;

    ss.str(""); ss << cost_map_folder + "cost_map_derv_0_" << std::setw(2) << std::setfill( '0' ) << ith << ".txt";
    move3d_save_matrix_to_file( mat1, ss.str() );

    ss.str(""); ss << cost_map_folder + "cost_map_derv_1_" << std::setw(2) << std::setfill( '0' ) << ith << ".txt";
    move3d_save_matrix_to_file( mat2, ss.str() );

    ss.str(""); ss << cost_map_folder + "cost_map_jac_mag_simple_" << std::setw(2) << std::setfill( '0' ) << ith << ".txt";
    move3d_save_matrix_to_file( mat3, ss.str() );

    ss.str(""); ss << cost_map_folder + "cost_map_jac_mag_custom_" << std::setw(2) << std::setfill( '0' ) << ith << ".txt";
    move3d_save_matrix_to_file( mat4, ss.str() );

    ss.str(""); ss << cost_map_folder + "cost_map_jac_cost_" << std::setw(2) << std::setfill( '0' ) << ith << ".txt";
    move3d_save_matrix_to_file( mat4, ss.str() );
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

double PlanarFeature::minDistToDemonstrations( const Configuration& q )
{
    double min_dist = std::numeric_limits<double>::max();

    for( size_t i=0; i<demos_.size(); i++ )
    {
        for( int j=0; j<demos_[i].getNbOfViaPoints(); j++ )
        {
            double dist = demos_[i][j]->dist( q, false );
            if( dist < min_dist ){
                min_dist = dist;
            }
        }
    }
    return min_dist;
}

double PlanarFeature::jacobianCost(const Configuration& q)
{
//    cout << "balance_cumul_jac_ : " << balance_cumul_jac_ << endl;
//    cout << "active feature size : " << active_features_.size() << endl;

//    return 1 / Feature::getFeaturesJacobianMagnitude( q );
//    return exp(-10*Feature::getFeaturesJacobianMagnitude( q )); // 10 is for scaling TODO findout what to put here
//    FeatureJacobian J = getFeaturesJacobian( q );
//    FeatureVect f1(w_.size());

//    if( f1.size() != J.rows() )
//    {
//        cout << __PRETTY_FUNCTION__ << endl;
//        cout << "ERROR IN OPTIMAL WEIGHTS COMPUTATION" << endl;
//        return f1.norm();
//    }

    FeatureVect phi1 = getFeatures( q );
    // Change for parallel stomp
    // Activate all feature
    std::vector<int> active_feature_tmp;
    for( int i=0;i<getNumberOfFeatures();i++)
        active_feature_tmp.push_back(i);

    FeatureVect phi = getFeatures( q, active_feature_tmp );
//    FeatureVect phi_cumul = phi_cumul_ / iter_ ;

//    double dist1 = ( phi_cumul - phi ).norm();
    // FeatureVect phi_delta = phi - phi_demos_;
//    double delta_sum = phi.sum();
//    double dist_to_mean = ( phi ).minElement();
//    FeatureVect f( phi.size() );
//    for( int j=0;j<phi_delta.size();j++)
//        f(j) =  J.row(j).norm();

//    for( int j=0;j<J.rows();j++)
//    {
//        f1(j) =  J.row(j).norm();
//    }

//    FeatureVect f2(w_.size());
//    for( int i=0;i<f2.size();i++)
//    {
//        if( std::isnan( phi_cumul(i) ) )
//        {
//            f2 = FeatureVect::Zero( phi_cumul.size() );
//            cout << "nan numbers at : " << q[6] << " , " << q[7] << endl;
//            break;
//        }
//        f2(i) =  phi_demos_(i) - phi_cumul(i)/phi_cumul.maxCoeff() ;
//    }

//    const double factor_distance = 10.0;
//    const double factor_height = HriEnv->getDouble(HricsParam::ioc_spheres_power);

//    cout << "phi : " << phi.transpose() << endl;
//    cout << "phi_cumul : " << phi_cumul.transpose() << endl;

//    double f1_cost =  std::exp(-factor_distance*( f1.norm() ) );
//    double f2_cost =  f2.norm();
//    double f2_cost = minDistToDemonstrations( q ) / 50;

//    if( f2_cost < min_diff_ ){
//        min_diff_ = f2_cost;
//    }
//    f2_cost -= cur_min_diff_;
//    if( f2_cost < 0.0 ){
//        f2_cost = cur_min_diff_;
//    }

    double f1 = phi.sum();
//    double f2 = 0.0;
//    for( int i=0;i<w_.size();i++){
//        f2 = w_[i] * phi[i];
//    }
    // double f3 = phi1.norm();

    // cout << "f1 : " << f1 << " , f2 : " << f2 << " , f3 : " << f3 << endl;

    return  f1;
}

double PlanarFeature::getVariance( const Move3D::Trajectory& t )
{
    FeatureVect phi = getFeatureCount( t );
    FeatureVect phi_dev = phi - ( phi_cumul_ / double(iter_) );
    if( iter_ == 0 ){
        phi_dev = FeatureVect::Zero( phi.size() );
    }
    // cout << "phi_cumul_ : " << phi_cumul_.transpose() << endl;
    return phi_dev.squaredNorm();
}

double PlanarFeature::getDeltaSum( const Move3D::Trajectory& t )
{
    FeatureVect phi = getFeatureCount( t );
    FeatureVect phi_delta = phi - phi_demos_;
//    cout << phi_demos_.transpose() << endl;
//    cout << phi_delta.transpose() << endl;
    return phi_delta.sum();
}

Eigen::VectorXd PlanarFeature::getStompCost( const Move3D::Trajectory& t )
{
    Eigen::VectorXd cost( t.getNbOfViaPoints() );

    FeatureVect phi = getFeatureCount( t );
    FeatureVect phi_delta = phi - phi_demos_;
    FeatureVect phi_dev = phi - ( phi_cumul_ / double(iter_-1) );
    if( iter_ == 0 ){
        phi_dev = FeatureVect::Zero( phi.size() );
    }
    double delta_sum = phi_delta.sum();

    double increment_dev=1.0;
    double increment_delta=increment_delta_;

    for( int i=0; i<cost.size(); i++ )
    {
        WeightVect w( w_.size() );

        for( int j=0; j<phi.size(); j++ )
        {
            w[j] = EPS6;

            if( delta_sum > 0 )
            {
                if( phi_delta[j] > 0 )
                {
                    w[j] += increment_delta; // Minimize feature
                }
                else
                {
                    w[j] -= increment_delta; // Maximize feature
                }
            }
            else if( delta_sum < 0 )
            {
                if( phi_delta[j] < 0 )
                {
                    w[j] -= increment_delta; // Maximize feature
                }
                else
                {
                    w[j] += increment_delta; // Minimize feature
                }
            }

            if( phi_dev[j] > 0 )
            {
                w[j] -= increment_dev; // Maximize feature
            }
            else if( phi_dev[j] < 0 )
            {
                w[j] += increment_dev; // Minimize feature
            }

            if( w[j] < 0 ) {
                w[j] = EPS6;
            }
        }

        // cout << w.transpose() << endl;

        cost[i] = w.transpose() * getFeatures( *t[i] );
    }

    // cout << cost.transpose() << endl;

    return cost;
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

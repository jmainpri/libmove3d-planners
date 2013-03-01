#ifndef HRICS_CLASSIFYMOTION_HPP
#define HRICS_CLASSIFYMOTION_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API

#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace HRICS
{
    class ClassifyMotion
    {
    public:
        ClassifyMotion();
        ~ClassifyMotion();

        bool load_model();
        std::vector<double> classify_motion( const Eigen::MatrixXd& motion );

    private:
        Eigen::VectorXd gauss_pdf( const Eigen::MatrixXd&, int id_class, int id_state );
        Eigen::MatrixXd load_from_csv( const std::string& filename );
        Eigen::MatrixXd convert_to_matrix( const std::vector< std::vector< std::string > >& matrix );

        int m_nb_classes;
        int m_nb_states;
        std::vector<Eigen::VectorXd> m_priors;
        std::vector<Eigen::MatrixXd> m_mu;
        std::vector< std::vector<Eigen::MatrixXd> > m_sigma;
    };
}

extern HRICS::ClassifyMotion* global_classifyMotion;

#endif // HRICS_CLASSIFYMOTION_HPP

#ifndef HRICS_CLASSIFYMOTION_HPP
#define HRICS_CLASSIFYMOTION_HPP

#include "HRICS_RecordMotion.hpp"

namespace HRICS
{
    class ClassifyMotion
    {
    public:
        ClassifyMotion() { }
        ~ClassifyMotion() { }

        void load_model();
        void classify_motion(const std::vector<motion_t>& motion, int end_idx=-1);

    private:
        void gauss_pdf(Eigen::Matrix&, int class_id);
    };
}


#endif // HRICS_CLASSIFYMOTION_HPP

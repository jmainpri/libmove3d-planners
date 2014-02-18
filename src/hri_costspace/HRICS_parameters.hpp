#ifndef HRICS_PARAMETERS_HPP
#define HRICS_PARAMETERS_HPP

#include <libmove3d/p3d/ParametersEnv.hpp>

#ifdef QT_LIBRARY
class HricsParam : public QObject
{

  Q_OBJECT;
  Q_ENUMS(boolParameter);
  Q_ENUMS(intParameter);
  Q_ENUMS(doubleParameter);
  Q_ENUMS(stringParameter);
  Q_ENUMS(vectorParameter);

public:

  HricsParam();
  ~HricsParam();

#else
namespace HricsParam
{
#endif
        enum boolParameter
        {
            init_spheres_cost,
            ioc_single_iteration,
            ioc_load_samples_from_file,
            ioc_draw_demonstrations,
            ioc_draw_samples,
            ioc_sample_around_demo,
            ioc_exit_after_run
        };

        enum intParameter
        {
            ioc_phase,
            ioc_sample_iteration,
            ioc_nb_of_way_points,
            ioc_planner_type
        };

        enum doubleParameter
        {
            ioc_spheres_power,
            ioc_sample_std_dev,
            ioc_cost_factor
        };

        enum stringParameter
        {
             titi
        };

        enum vectorParameter
        {
             tutu
        };
};

// Object that holds all parameters
// Of the planner Environment
extern Parameters<HricsParam::boolParameter,HricsParam::intParameter,HricsParam::doubleParameter,
HricsParam::stringParameter,HricsParam::vectorParameter>* HriEnv;

// Functions that initializes the planner
// Parameters
void initHricsParameters();

#ifdef QT_LIBRARY
extern HricsParam* EnumHricsParameterObject;
#endif


#endif // HRICS_PARAMETERS_HPP

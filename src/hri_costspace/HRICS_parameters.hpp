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
            init_spheres_cost
        };

        enum intParameter
        {
            ioc_phase,
            ioc_sample_iteration
        };

        enum doubleParameter
        {
            ioc_spheres_power
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

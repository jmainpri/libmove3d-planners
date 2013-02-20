#ifndef SAMPLING_API_HPP
#define SAMPLING_API_HPP

#ifndef CONFIGURATION_HPP
class Configuration;
#endif
#ifndef LOCALPATH_HPP
class LocalPath;
#endif

#include <tr1/memory>

class Robot;

/**
    @ingroup CONFIG_SPACE
    @brief The sampling API
    */
class SamplingAPI
{
protected:
	Robot* mR;

public:
	SamplingAPI(Robot* r) : mR(r) {}

	virtual ~SamplingAPI();

    virtual MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> sample(bool samplePassive = true);

    MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> shootCollisionFree();
};

#endif

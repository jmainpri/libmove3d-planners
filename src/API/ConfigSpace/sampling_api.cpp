#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"

using std::tr1::shared_ptr;

SamplingAPI::~SamplingAPI() {}

shared_ptr<Configuration> SamplingAPI::sample(bool samplePassive)
{
	return(mR->shoot(samplePassive));
}

shared_ptr<Configuration> SamplingAPI::shootCollisionFree()
{
	bool collision(true);
	shared_ptr<Configuration> q;
	while(collision)
	{
		q = this->sample();
		collision = mR->isInCollision(*q);
	}
	return(q);
}

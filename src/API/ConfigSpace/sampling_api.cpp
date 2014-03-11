#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"

using MOVE3D_PTR_NAMESPACE::shared_ptr;

SamplingMove3D::~SamplingAPI() {}

shared_ptr<Configuration> SamplingMove3D::sample(bool samplePassive)
{
	return(mR->shoot(samplePassive));
}

shared_ptr<Configuration> SamplingMove3D::shootCollisionFree()
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

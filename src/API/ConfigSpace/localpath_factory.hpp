#ifndef LOCALPATH_FACTORY_HPP
#define LOCALPATH_FACTORY_HPP

#ifndef CONFIGURATION_HPP
class Configuration;
#endif
#ifndef LOCALPATH_HPP
class LocalPath;
#endif


/**
    @ingroup CONFIG_SPACE
    @brief Creares local paths
    */
class LocalpathFactory
{
public:
	LocalpathFactory();

	virtual ~LocalpathFactory();

	virtual Localpath* create(
			MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> q1,
			MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> q2);
};

#endif

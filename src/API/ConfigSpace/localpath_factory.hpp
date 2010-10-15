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
			std::tr1::shared_ptr<Configuration> q1,
			std::tr1::shared_ptr<Configuration> q2);
};

#endif

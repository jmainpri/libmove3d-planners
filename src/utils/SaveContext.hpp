/*
 * SaveContext.hpp
 *
 *  Created on: Sep 11, 2009
 *      Author: jmainpri
 */

#ifndef SAVECONTEXT_HPP_
#define SAVECONTEXT_HPP_

#include "../p3d/env.hpp"
#include "planner/planEnvironment.hpp"

#include <iostream>
#include <vector>
#include <iterator>
#include <map>

/**
 * @ingroup USER_APPLI
 * Saves the context describe in an Env structure of one run and
 * associates a Statistic Data structure
 */
class SaveContext {

public:
	/**
	 * Constructor
	 */
	SaveContext();

	/**
	 * Destructor
	 */
	~SaveContext();

	/**
	 * Erases all contexts
	 */
	void clear();

	/**
	 * returns the number of environments
	 * Stored in the data structures
	 */
	unsigned int getNumberStored();

	/**
	 * Saves Current Environment
	 * to Stack
	 */
	unsigned int saveCurrentEnvToStack();

	/**
	 * Changes the Current Environment
	 * to a stored one
	 */
	void switchCurrentEnvTo(unsigned int i);
	
	/**
	 * Delete Env
	 */
	void deleteEnv(unsigned int i);

	/**
	 * Prints the variables of
	 * The environment
	 */
	void printVariables(unsigned int id);

	/**
	 * Print all variables of an environment
	 */
	void printData(unsigned int i);

	/**
	 * Get Time Vector
	 */
	std::vector<double>& getTime(int id) { return _Time.at(id); }

	/**
	  * Add Time Vector
	  */
	void addTime(std::vector<double> time) { _Time.push_back(time); }

	/**
	 * Copy constructors of the 4 type of maps
	 */
	std::map<Env::boolParameter, boolContainer*> copyMap( std::map<Env::boolParameter, boolContainer*> map );
	std::map<Env::intParameter, intContainer*> copyMap( std::map<Env::intParameter, intContainer*> map );
	std::map<Env::doubleParameter, doubleContainer*> copyMap( std::map<Env::doubleParameter, doubleContainer*> map );
	std::map<Env::stringParameter, stringContainer*> copyMap( std::map<Env::stringParameter, stringContainer*> map );

private:
	std::vector< std::map<Env::boolParameter, boolContainer*> > _MapBool;
	std::vector< std::map<Env::intParameter, intContainer*> > _MapInt;
	std::vector< std::map<Env::doubleParameter, doubleContainer*> > _MapDouble;
	std::vector< std::map<Env::stringParameter, stringContainer*> > _MapString;

	std::vector< std::vector<double> > _Time;
//	std::vector< std::vector<Statistics*> > _Statistics;
};

extern SaveContext storedContext;

/**
 * @ingroup USER_APPLI
 * Saves the parameter environement described in an Env structure of one run and
 * associates a Statistic Data structure
 */
template <typename enumBool,
					typename enumInt,
					typename enumDouble,
					typename enumString,
					typename enumVector>
class SaveParameterEnv 
{	
public:
	
	/**
	 * Constructor
	 */
	SaveParameterEnv(Parameters<enumBool,
									 enumInt,
									 enumDouble,
									 enumString,
									 enumVector>* env) : m_ParamEnv(env)
	{
		//	clear();
		//	saveCurrentEnvToStack();
		clear();
		m_Time.clear();
	}
	
	/**
	 * Destructor
	 */
	~SaveParameterEnv()
	{
		clear();
	}
	
	/**
	 * Erases all contexts
	 */
	void clear()
	{
		std::cout << "Deleting " << m_MapBool.size() << " context" << std::endl;
		
		m_MapBool.clear();
		m_MapInt.clear();
		m_MapDouble.clear();
		m_MapString.clear();
		
		for(unsigned int i=0;i<m_Time.size();i++)
		{
			m_Time.at(i).clear();
		}
		
		m_Time.clear();
	}
	
	/**
	 * returns the number of environments
	 * Stored in the data structures
	 */
	unsigned int getNumberStored()
	{
		return m_MapBool.size();
	}
	
	/**
	 * Saves Current Environment
	 * to Stack
	 */
	unsigned int saveCurrentEnvToStack()
	{
		m_MapBool.push_back(		copyMap(m_ParamEnv->getBoolMap()) );
		m_MapInt.push_back(			copyMap(m_ParamEnv->getIntMap()) );
		m_MapDouble.push_back(	copyMap(m_ParamEnv->getDoubleMap()) );
		m_MapString.push_back(	copyMap(m_ParamEnv->getStringMap()) );
		
		return getNumberStored();
	}
	
	
	/**
	 * Changes the Current Environment
	 * to a stored one
	 */
	void switchCurrentEnvTo(unsigned int i)
	{
		std::cout << "Switching to " <<  i << " context" <<  std::endl;
		std::cout << "       " <<  getNumberStored() << " are stored" <<  std::endl;
		
		for(typename std::map<enumBool,boolContainer*>::iterator iter = m_MapBool[i].begin();
				iter != m_MapBool[i].end();
				iter++)
		{
			m_ParamEnv->setBool(iter->first,iter->second->get());
		}
		
		for(typename std::map<enumInt, intContainer*>::iterator iter = m_MapInt[i].begin();
				iter != m_MapInt[i].end();
				iter++)
		{
			m_ParamEnv->setInt(iter->first,iter->second->get());
		}
		
		for(typename std::map<enumDouble, doubleContainer*>::iterator iter = m_MapDouble[i].begin();
				iter != m_MapDouble[i].end();
				iter++)
		{
			m_ParamEnv->setDouble(iter->first,iter->second->get());
		}
		
#ifdef QT_LIBRARY
		for(typename std::map<enumString, stringContainer*>::iterator iter = m_MapString[i].begin();
				iter != m_MapString[i].end();
				iter++)
		{
			m_ParamEnv->setString(iter->first,iter->second->get());
		}
#endif
	}
	
	
	/**
	 * Delete Env
	 */
	void deleteEnv(unsigned int i)
	{
		
		m_MapBool.erase (	m_MapBool.begin()	+ i );
		m_MapInt.erase (	m_MapInt.begin() + i );
		m_MapDouble.erase (	m_MapDouble.begin()	+ i );
		m_MapString.erase (	m_MapString.begin()	+ i );
	}
	
	/**
	 * Prints the variables of
	 * The environment
	 */
	void printVariables(unsigned int id)
	{
		std::cout << "Bool -------------------------------" <<  std::endl;
		
		typename std::map<enumBool, boolContainer*>::iterator iter1;
		iter1 = m_MapBool[id].begin();
		
		for(; iter1 != m_MapBool[id].end(); iter1++)
		{
			std::cout << iter1->second->get() << std::endl;
		}
		
		
		std::cout << "Integers ---------------------------" <<  std::endl;
		typename std::map<enumInt, intContainer*>::iterator iter2 = m_MapInt[id].begin();
		//	cout << "size " << d->getIntMap().size() << endl;
		
		for(; iter2 != m_MapInt[id].end(); iter2++)
		{
			std::cout << iter2->second->get() <<  std::endl;
		}
		
		std::cout << "Doubles ----------------------------" <<  std::endl;
		typename std::map<enumDouble, doubleContainer*>::iterator iter3 = m_MapDouble[id].begin();
		
		for(; iter3 != m_MapDouble[id].end(); iter3++)
		{
			std::cout << iter3->second->get() <<  std::endl;
		}
		
#ifdef QT_LIBRARY
		std::cout << "String -----------------------------" << std::endl;
		typename std::map<enumString, stringContainer*>::iterator iter4 = m_MapString[id].begin();
		
		for(; iter4 != m_MapString[id].end(); iter4++)
		{
			std::cout << iter4->second->get().toStdString() <<  std::endl;
		}
#endif
		std::cout << "------------ end --------------" <<  std::endl;
	}
	
	/**
	 * Print all variables of an environment
	 */
	void printData(unsigned int i)
	{
		printVariables(i);
		
		std::cout << "m_Map["<<i<<"]"<< std::endl;
	}
	
	/**
	 * Get Time Vector
	 */
	std::vector<double>& getTime(int id) { return m_Time.at(id); }
	
	/**
	 * Add Time Vector
	 */
	void addTime(std::vector<double> time) { m_Time.push_back(time); }
	
	/**
	 * Copy constructors of the 4 type of maps
	 */
	std::map<enumBool,		boolContainer*>		copyMap( std::map<enumBool,			boolContainer*>		myMap )
	{
		typename std::map<enumBool, boolContainer*>::iterator iter = myMap.begin();
		
		for(; iter != myMap.end(); iter++)
		{
			//		cout << iter->second->get() << endl;
			iter->second = new boolContainer(iter->second->get());
		}
		
		return myMap;
	}
	
	
	std::map<enumInt,			intContainer*>		copyMap( std::map<enumInt,			intContainer*>		myMap )
	{
		typename std::map<enumInt, intContainer*>::iterator iter = myMap.begin();
		
		for(; iter != myMap.end(); iter++)
		{
			//		cout << iter->second->get() << endl;
			iter->second = new intContainer(iter->second->get());
		}
		
		return myMap;
	}
	
	
	std::map<enumDouble,	doubleContainer*> copyMap( std::map<enumDouble,		doubleContainer*> myMap )
	{
		typename std::map<enumDouble, doubleContainer*>::iterator iter = myMap.begin();
		
		for(; iter != myMap.end(); iter++)
		{
			//		cout << iter->second->get() << endl;
			iter->second = new doubleContainer(iter->second->get());
		}
		
		return myMap;
	}
	
	std::map<enumString,	stringContainer*> copyMap( std::map<enumString,		stringContainer*> myMap )
	{
		typename std::map<enumString, stringContainer*>::iterator iter = myMap.begin();
		
		for(; iter != myMap.end(); iter++)
		{
			//		cout << iter->second->get() << endl;
#ifdef QT_LIBRARY
			iter->second = new stringContainer(iter->second->get());
#endif
		}
		return myMap;
	}
	
private:
	Parameters<	enumBool,
							enumInt,
							enumDouble,
							enumString,
							enumVector>* m_ParamEnv;
	
	std::vector< std::map<enumBool,							boolContainer*> >			m_MapBool;
	std::vector< std::map<enumInt,							intContainer*> >			m_MapInt;
	std::vector< std::map<enumDouble,						doubleContainer*> >		m_MapDouble;
	std::vector< std::map<enumString,						stringContainer*> >		m_MapString;
	
	std::vector< std::vector<double> > m_Time;
	//	std::vector< std::vector<Statistics*> > _Statistics;
};

extern SaveParameterEnv<
PlanParam::boolParameter,
PlanParam::intParameter,
PlanParam::doubleParameter,
PlanParam::stringParameter,
PlanParam::vectorParameter>* storedPlannerContext;

#endif /* SAVECONTEXT_HPP_ */
